/*
 * SPDX-License-Identifier: MIT
 * Dual-band (sub-GHz + 2.4 GHz) LR2021 adapter — TDM scheduler.
 *
 * See DualBandRadio.h for the design.  The window state machine itself is
 * the pure, dependency-free dm_step() from dualband_tdm.h (sandbox-tested);
 * this file only glues it to Zephyr work scheduling + the driver switch.
 */

#include "DualBandRadio.h"

#include <string.h>

/* LR20xx driver extension API (extern "C" — see LR2021Radio.cpp) */
extern "C" {
#include "lr20xx_lora.h"
}

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dualband_radio, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

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
	  _hf_tx_len(0)
{
	k_work_init_delayable(&_dm_work, dmWorkStatic);
	memset(_hf_tx_buf, 0, sizeof(_hf_tx_buf));
}

void DualBandRadio::begin()
{
	LR2021Radio::begin();

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
	lr20xx_switch_band(_dev, DM_HF_FREQ_HZ, DM_HF_SF,
			   BW_500_KHZ, DM_HF_CR, DM_HF_TX_PWR_DM);
	_hf_open = true;
	_dm_state = DM_STATE_HF_OPEN;
	_dm_state_start_ms = (uint32_t)k_uptime_get_32();
	setActiveRxBand(1); /* L4-U5: tag HF RX packets with band=1 */
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
	buildModemConfig(cfg, false);

	lr20xx_switch_band(_dev, cfg.frequency, (uint8_t)cfg.datarate,
			   cfg.bandwidth, (uint8_t)cfg.coding_rate,
			   cfg.tx_power);
	_hf_open = false;
	_dm_state = DM_STATE_IDLE;
	_dm_state_start_ms = (uint32_t)k_uptime_get_32();
	setActiveRxBand(0); /* L4-U5: back to primary-band RX tagging */
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
	 * drops the latch); a few extra ms of window overrun is harmless. */
	if (_hf_tx_active) {
		dmSchedule(DM_SKIP_RETRY_MS);
		return;
	}

	const bool in_rx = isInRecvMode() && !isTxActive();
	const bool receiving = hwIsReceiving();

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
		return LR2021Radio::startSendRaw(bytes, len);
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
	return LR2021Radio::startSendRaw(bytes, len);
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
		_hf_tx_active = true;
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
}

} /* namespace mesh */
