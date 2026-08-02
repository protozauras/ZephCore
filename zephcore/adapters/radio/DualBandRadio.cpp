/*
 * SPDX-License-Identifier: MIT
 * Dual-band (sub-GHz + 2.4 GHz) LR2021 adapter — TDM scheduler.
 *
 * See DualBandRadio.h for the design.  The window state machine itself is
 * the pure, dependency-free dm_step() from dualband_tdm.h (sandbox-tested);
 * this file only glues it to Zephyr work scheduling + the driver switch.
 */

#include "DualBandRadio.h"

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
	  _dm_state_start_ms(0)
{
	k_work_init_delayable(&_dm_work, dmWorkStatic);
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
	dmLogFreq("HF window open", DM_HF_FREQ_HZ);
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
	if (_hf_open) {
		return false; /* HF window open — defer mesh TX */
	}
	return LR2021Radio::isRadioReady();
}

} /* namespace mesh */
