/*
 * SPDX-License-Identifier: MIT
 * ZephCore dual-band (sub-GHz + 2.4 GHz) radio adapter for the LR2021.
 *
 * A single LR2021 radio (one synthesizer) time-shares two bands using the
 * TDM scheduler in dualband_tdm.h: the primary band (prefs config, sub-GHz
 * 869.618 MHz by default) is listened to continuously, and every ~1.5 s a
 * short (~70 ms) 2.4 GHz RX window is opened via lr20xx_switch_band() —
 * the lean driver switch that does NOT reload the 3-bin FE calibration +
 * 50 ms (L3-U4A).  Mesh TX is deferred while the HF window is open (the
 * dispatcher re-queues), so sub-GHz traffic is never transmitted on HF.
 */

#pragma once

#include "LR2021Radio.h"
#include "dualband_tdm.h"
#include <zephyr/kernel.h>

namespace mesh {

class DualBandRadio : public LR2021Radio {
public:
	DualBandRadio(const struct device *lora_dev, MainBoard &board,
		      NodePrefs *prefs = nullptr);

	void begin() override;

	/* While the HF (2.4 GHz) RX window is open, report the radio as not
	 * ready so mesh TX is deferred (Dispatcher::checkSend re-queues) and
	 * never starts with the chip on the HF band. */
	bool isRadioReady() override;

	/* True while the TDM window is open on the HF band. */
	bool tdmHfWindowOpen() const { return _hf_open; }

	/* True when the TDM scheduler is active (primary band is sub-GHz). */
	bool tdmEnabled() const { return _tdm_enabled; }

private:
	struct k_work_delayable _dm_work;
	dm_config_t _dm_cfg;
	volatile bool _hf_open;      /* true during the HF window */
	volatile bool _tdm_enabled;  /* false if primary band is already HF */
	dm_state_t _dm_state;
	uint32_t _dm_state_start_ms; /* k_uptime_get_32() at state entry */

	void dmStart();
	void dmSchedule(uint32_t delay_ms);
	static void dmWorkStatic(struct k_work *work);
	void dmWork();
	void dmOpenHfWindow();
	void dmCloseHfWindow();
	uint32_t dmTimeInState() const;
	void dmLogFreq(const char *what, uint32_t freq_hz);
};

} /* namespace mesh */
