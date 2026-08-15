/*
 * SPDX-License-Identifier: MIT
 * Dual-band (sub-GHz + 2.4 GHz) LR2021 adapter — TDM scheduler.
 *
 * See DualBandRadio.h for the design.  The window state machine itself is
 * the pure, dependency-free dm_step() from dualband_tdm.h (sandbox-tested);
 * this file only glues it to Zephyr work scheduling + the driver switch.
 */

#include "DualBandRadio.h"
#include "power_debug.h"

#include <string.h>

/* LR20xx driver extension API (extern "C" — see LR2021Radio.cpp) */
extern "C" {
#include "lr20xx_lora.h"
}

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dualband_radio, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

/* Backstop for the _hf_tx_active latch (see dmWork): if a completion
 * never reaches onTxComplete, force-clear after this long so the HF
 * window can always close and the chip return to the primary band.
 * 5000 ms deliberately matches the base TX wait thread's TX_TIMEOUT_MS
 * (radio_common.h), so a legitimately pending TX always clears its own
 * latch (via onTxComplete) before this ever fires.
 * (ROOTCAUSE 2026-08-11 §9 candidate 2.) */
#define DM_HF_TX_LATCH_TIMEOUT_MS 5000u

DualBandRadio::DualBandRadio(const struct device *lora_dev, MainBoard &board,
			     NodePrefs *prefs)
	: LR2021Radio(lora_dev, board, prefs),
	  _dm_cfg(dm_default_config()),
	  _hf_open(false),
	  _tdm_enabled(false),
	  _dm_state(DM_STATE_IDLE),
	  _dm_state_start_ms(0),
	  _hf_tx_pending(false),
	  _hf_tx_active(false),
	  _hf_tx_active_ms(0),
	  _hf_tx_len(0)
{
	k_work_init_delayable(&_dm_work, dmWorkStatic);
	memset(_hf_tx_buf, 0, sizeof(_hf_tx_buf));
}

void DualBandRadio::begin()
{
	LR2021Radio::begin();

	/* Power debug: the base begin() leaves the radio in RX on the
	 * primary band.  State attribution starts here (covers both the
	 * TDM-enabled and self-disabled cases below). */
	power_debug_enter_state(POWER_STATE_RX_SUBGHZ);

	/* TDM makes sense only when the PRIMARY band is sub-GHz.  If prefs
	 * are configured for 2.4 GHz as the primary (ZEPHCORE_BAND_2G4),
	 * opening a 2.4 GHz window on top of a 2.4 GHz primary would be a
	 * pointless same-band switch — disable the scheduler. */
	if (getActiveFrequencyHz() > 1500000000u) {
		LOG_WRN("TDM disabled: primary band is already 2.4 GHz");
		return;
	}

	_tdm_enabled = true;
	_dm_state = DM_STATE_IDLE;
	LOG_INF("TDM: dual-band scheduler on — primary %u Hz, HF window %u ms / %u ms",
		(unsigned)getActiveFrequencyHz(), (unsigned)_dm_cfg.hf_window_ms,
		(unsigned)_dm_cfg.hold_ms);

	dmStart();
}

void DualBandRadio::dmStart()
{
	_dm_state_start_ms = (uint32_t)k_uptime_get_32();
	dmSchedule(_dm_cfg.hold_ms);
}

void DualBandRadio::dmSchedule(uint32_t delay_ms)
{
	k_work_reschedule(&_dm_work, K_MSEC(delay_ms));
}

void DualBandRadio::dmWorkStatic(struct k_work *work)
{
	DualBandRadio *self = CONTAINER_OF(work, DualBandRadio, _dm_work);
	self->dmWork();
}

uint32_t DualBandRadio::dmTimeInState() const
{
	return (uint32_t)k_uptime_get_32() - _dm_state_start_ms;
}

void DualBandRadio::dmLogFreq(const char *what, uint32_t freq_hz)
{
	LOG_INF("TDM: %s freq=%u", what, (unsigned)freq_hz);
}

void DualBandRadio::dmOpenHfWindow()
{
	/* lean band switch + continuous RX on the HF preset (L3-U4A:
	 * no 3-bin FE cal / 50 ms — boot FE cal already covers 2441 MHz) */
	int ret = lr20xx_switch_band(_dev, DM_HF_FREQ_HZ, DM_HF_SF,
				     BW_500_KHZ, DM_HF_CR, DM_HF_TX_PWR_DM);
	if (ret != 0) {
		/* Switch bailed (TX in flight / unconfigured): the chip
		 * never left the primary band — keep _hf_open/_dm_state
		 * TRUTHFUL and retry shortly.  Pre-fix (01b3df9) the
		 * bail was silent and this flip happened anyway, leaving
		 * the TDM believing the chip was on HF while it was
		 * transmitting sub-GHz — the entry wedge of every hang.
		 * (ROOTCAUSE 2026-08-11 §9 candidate 1.) */
		LOG_WRN("TDM: HF open switch bailed (%d) — window stays closed",
			ret);
		dmSchedule(DM_SKIP_RETRY_MS);
		return;
	}
	power_debug_enter_state(POWER_STATE_STANDBY);
	_hf_open = true;
	_dm_state = DM_STATE_HF_OPEN;
	_dm_state_start_ms = (uint32_t)k_uptime_get_32();
	setActiveRxBand(1); /* L4-U5: tag HF RX packets with band=1 */
	power_debug_enter_state(POWER_STATE_RX_HF);
	dmLogFreq("HF window open", DM_HF_FREQ_HZ);

	/* L4-U5: a flood/unknown packet stashed while the window was closed
	 * goes out on HF first, then the window continues in HF RX for its
	 * nominal length (plan §1.3: flood -> both). */
	if (_hf_tx_pending) {
		_hf_tx_pending = false;
		startHfTx(_hf_tx_buf, _hf_tx_len);
	}

	dmSchedule(_dm_cfg.hf_window_ms);
}

void DualBandRadio::dmCloseHfWindow()
{
	/* Rebuild the PRIMARY band config from prefs (what the mesh core
	 * uses for TX/RX) and switch back.  invalidateConfigCache() (called
	 * here after the switch) forces the next configureTx() to run the
	 * FULL lora_config() on the primary band instead of the
	 * direction-only fast path, which would otherwise TX on the chip's
	 * still-loaded HF modem params. */
	struct lora_modem_config cfg;

	/* A close must ALWAYS rebuild the PRIMARY config: a leftover
	 * forced-HF modem (lost-completion race / latch timeout path)
	 * would otherwise make this switch a same-band no-op on HF. */
	setForceHfModem(false);
	buildModemConfig(cfg, false);

	int ret = lr20xx_switch_band(_dev, cfg.frequency, (uint8_t)cfg.datarate,
				     cfg.bandwidth, (uint8_t)cfg.coding_rate,
				     cfg.tx_power);
	if (ret != 0) {
		/* Switch bailed (an HF TX still in flight): the chip is
		 * still on HF — keep the window state truthful and retry
		 * shortly.  (ROOTCAUSE 2026-08-11 §9 candidate 1.) */
		LOG_WRN("TDM: HF close switch bailed (%d) — window stays open",
			ret);
		dmSchedule(DM_SKIP_RETRY_MS);
		return;
	}
	power_debug_enter_state(POWER_STATE_STANDBY);
	_hf_open = false;
	_dm_state = DM_STATE_IDLE;
	_dm_state_start_ms = (uint32_t)k_uptime_get_32();
	setActiveRxBand(0); /* L4-U5: back to primary-band RX tagging */
	power_debug_enter_state(POWER_STATE_RX_SUBGHZ);
	invalidateConfigCache();
	dmLogFreq("back to sub-GHz", cfg.frequency);
	dmSchedule(_dm_cfg.hold_ms);
}

void DualBandRadio::dmWork()
{
	if (!_tdm_enabled) {
		return;
	}

	/* Primary band drift guard (prefs changed to 2.4 GHz mid-run). */
	if (getActiveFrequencyHz() > 1500000000u) {
		if (_tdm_enabled) {
			LOG_WRN("TDM: primary band moved to 2.4 GHz — scheduler off");
			_tdm_enabled = false;
			k_work_cancel_delayable(&_dm_work);
		}
		return;
	}

	/* L4-U5: never tear an in-flight in-window HF TX out from under the
	 * step — the CLOSE switch-back would run while the chip is still
	 * transmitting.  Re-check shortly after the TX completes (onTxComplete
	 * drops the latch); a few extra ms of window overrun is harmless.
	 *
	 * The latch has a timestamp + force-clear backstop: if a completion
	 * never reaches onTxComplete (lost DIO1 TX_DONE edge / the
	 * completion-before-latch race), the window must still be able to
	 * close and the chip to return to the primary band — otherwise the
	 * TDM stalls forever and the radio stays pinned on 2.4 GHz (hang #4).
	 * (ROOTCAUSE 2026-08-11 §9 candidate 2.) */
	if (_hf_tx_active) {
		const uint32_t latch_age =
			(uint32_t)k_uptime_get_32() - _hf_tx_active_ms;
		if (latch_age < DM_HF_TX_LATCH_TIMEOUT_MS) {
			dmSchedule(DM_SKIP_RETRY_MS);
			return;
		}
		LOG_WRN("TDM: HF TX latch timeout (%u ms) — force clearing",
			(unsigned)latch_age);
		_hf_tx_active = false;
		/* The forced-HF modem existed only for that TX — drop it so
		 * the CLOSE below rebuilds the PRIMARY band config. */
		setForceHfModem(false);
	}

	const bool in_rx = isInRecvMode() && !isTxActive();
	/* Bounded read (preamble-park guard, 2026-08-16, WORKPLAN §8.1):
	 * a stuck chip latch must not starve the TDM windows — use the
	 * same ignore semantics the TX gate uses. */
	const bool receiving = isReceiving();

	dm_decision_t dec = dm_step(_dm_state, in_rx, receiving,
				    dmTimeInState(), &_dm_cfg);

	switch (dec) {
	case DM_DECISION_OPEN:
		dmOpenHfWindow();
		break;
	case DM_DECISION_EXTEND:
		_dm_state = DM_STATE_HF_EXTEND;
		dmSchedule(_dm_cfg.hf_extend_step_ms);
		break;
	case DM_DECISION_CLOSE:
		dmCloseHfWindow();
		break;
	case DM_DECISION_SKIP:
	default:
		/* Radio busy (mid primary TX/RX) — retry soon, don't wait a
		 * full hold period so the HF window isn't starved. */
		_dm_state_start_ms = (uint32_t)k_uptime_get_32();
		dmSchedule(DM_SKIP_RETRY_MS);
		break;
	}
}

bool DualBandRadio::isRadioReady()
{
	if (!_tdm_enabled) {
		return LR2021Radio::isRadioReady();
	}

	/* While an in-window HF TX is in flight, refuse new TX (the mesh
	 * loop defers until onTxComplete() drops the latch). */
	if (_hf_tx_active) {
		return false;
	}

	if (_hf_open) {
		/* Window open: the chip is on HF — only an HF-only packet
		 * (mask == DB_BAND_HF) can TX now.  A BOTH flood defers so
		 * it goes out on the primary band right after the window and
		 * still stashes the HF copy for the NEXT window (plan §1.3). */
		return _tx_band_mask == DB_BAND_HF;
	}
	/* Window closed: everything with a sub-GHz component TXes on the
	 * primary band (HF-only packets defer until the next window). */
	return (_tx_band_mask & DB_BAND_SUBGHZ) != 0;
}

bool DualBandRadio::startSendRaw(const uint8_t *bytes, int len)
{
	if (!_tdm_enabled) {
		bool ok = LR2021Radio::startSendRaw(bytes, len);
		if (ok) {
			power_debug_enter_state(POWER_STATE_TX_SUBGHZ);
		}
		return ok;
	}

	if (_hf_open) {
		/* The isRadioReady() gate only lets HF-only packets reach
		 * here; the chip is already on the HF preset (window). */
		return startHfTx(bytes, len);
	}

	/* Window closed / primary band.  A BOTH (flood/unknown) packet also
	 * stashes an HF copy so the next window relays it on 2.4 GHz. */
	if ((_tx_band_mask & DB_BAND_HF) != 0) {
		queueHfCopy(bytes, len);
	}
	bool ok = LR2021Radio::startSendRaw(bytes, len);
	if (ok) {
		power_debug_enter_state(POWER_STATE_TX_SUBGHZ);
	}
	return ok;
}

void DualBandRadio::queueHfCopy(const uint8_t *bytes, int len)
{
	if (len > (int)sizeof(_hf_tx_buf)) {
		len = (int)sizeof(_hf_tx_buf);
	}
	memcpy(_hf_tx_buf, bytes, (size_t)len);
	_hf_tx_len = (uint16_t)len;
	_hf_tx_pending = true;
	LOG_DBG("TDM: stashed HF copy len=%u for next window", (unsigned)len);
}

bool DualBandRadio::startHfTx(const uint8_t *bytes, int len)
{
	/* Chip busy — let checkSend re-queue/defer. */
	if (!LR2021Radio::isRadioReady()) {
		return false;
	}

	/* Force buildModemConfig() to the HF preset so base's configureTx()
	 * programs the chip for the HF band (2450/BW500/SF8/CR4/5/@+12) and
	 * cannot rebuild the sub-GHz config onto the open window (the
	 * L3-U4B config-cache trap).  The latch is set AFTER base accepts so
	 * its internal isRadioReady() (mask==HF while window open) still
	 * passes. */
	setForceHfModem(true);

	/* The stashed HF copy was queued under the ORIGINAL packet's mask
	 * (DB_BAND_BOTH for a flood), but base's startSendRaw() re-checks
	 * the virtual isRadioReady(), which while the window is open
	 * accepts only mask == DB_BAND_HF (L4-U5 reference: equality, not
	 * &).  Pin the mask to HF for this call — an in-window HF TX is
	 * HF-only by construction — and restore it right after so the next
	 * checkSend() still routes subsequent packets on the original
	 * bands. */
	uint8_t saved_mask = _tx_band_mask;
	setTxBand(DB_BAND_HF);

	bool ok = LR2021Radio::startSendRaw(bytes, len);

	setTxBand(saved_mask);

	if (ok) {
		/* Completion-race guard (ROOTCAUSE 2026-08-11 §8 cand. 3):
		 * if the TX finished — and onTxComplete already ran —
		 * before the latch could be set (a very short/failed TX,
		 * or the txWait thread winning the scheduling race), then
		 * latching it now would wedge dmWork forever (nothing
		 * would ever clear it).  Only latch a TX that is still in
		 * flight.  The 5 s latch timeout is the backstop if this
		 * guard ever misses. */
		if (!LR2021Radio::isTxActive()) {
			LOG_WRN("TDM: HF TX completed before latch — not latching");
			setForceHfModem(false);
			return true;
		}
		_hf_tx_active = true;
		_hf_tx_active_ms = (uint32_t)k_uptime_get_32();
		power_debug_enter_state(POWER_STATE_TX_HF);
		LOG_INF("TDM: HF TX started len=%u", (unsigned)len);
		return true;
	}

	setForceHfModem(false);
	_hf_tx_active = false;
	return false;
}

void DualBandRadio::onTxComplete()
{
	/* Called by the base TX wait thread after every completed TX.
	 * The forced-HF modem was only for the in-window HF TX — drop it.
	 * Ordering: base called startReceive() BEFORE this hook, so a just
	 * finished in-window HF TX already re-armed HF RX (forced HF config);
	 * this clears the force for the next window's sub-GHz config.  For a
	 * sub-GHz TX the flag was never set — no-op. */
	setForceHfModem(false);
	_hf_tx_active = false;

	/* Power debug: base already re-armed RX (startReceive()) before this
	 * hook — attribute back to whichever band the chip is listening on. */
	power_debug_enter_state(_hf_open ? POWER_STATE_RX_HF
					 : POWER_STATE_RX_SUBGHZ);
}

} /* namespace mesh */
