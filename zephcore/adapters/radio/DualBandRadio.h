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
#include "dualband_route.h"
#include <zephyr/kernel.h>

namespace mesh {

class DualBandRadio : public LR2021Radio {
public:
	DualBandRadio(const struct device *lora_dev, MainBoard &board,
		      NodePrefs *prefs = nullptr);

	void begin() override;

	/* While the HF (2.4 GHz) RX window is open, report the radio as not
	 * ready so mesh TX is deferred (Dispatcher::checkSend re-queues) and
	 * never starts with the chip on the HF band — UNLESS the next TX is
	 * HF-bound (plan §L4-U5: the TDM window doubles as the HF TX slot).
	 * Base decides via the DB_BAND_* mask set by setTxBand(). */
	bool isRadioReady() override;

	/* L4-U5 TX routing: an HF-bound packet TXes on the open window;
	 * a both/flood packet TXes on the primary band now and stashes an
	 * HF copy for the next window. */
	bool startSendRaw(const uint8_t *bytes, int len) override;

	/* Clears the forced-HF modem + HF-TX latch after an in-window TX. */
	void onTxComplete() override;

	/* True while the TDM window is open on the HF band. */
	bool tdmHfWindowOpen() const { return _hf_open; }

	/* True when the TDM scheduler is active (primary band is sub-GHz). */
	bool tdmEnabled() const { return _tdm_enabled; }

	/* L4-U6: this radio has a real secondary 2.4 GHz band, so the mesh
	 * layer may send HF beacons and register HF neighbours. */
	bool isDualBand() const override { return true; }

	/* HF diag log channel (2026-08-16, WORKPLAN §8.3): stash a small
	 * diagnostics packet for the next TDM window.  One slot — a second
	 * stash while one is pending returns false (caller drops). */
	bool sendDiagHf(const uint8_t *bytes, int len) override;

private:
	struct k_work_delayable _dm_work;
	dm_config_t _dm_cfg;
	volatile bool _hf_open;      /* true during the HF window */
	volatile bool _tdm_enabled;  /* false if primary band is already HF */
	dm_state_t _dm_state;
	uint32_t _dm_state_start_ms; /* k_uptime_get_32() at state entry */

	/* ── L4-U5: pending HF retransmit copy ───────────────────── */
	/* A flood/unknown packet TXed on the primary band while the window
	 * was closed stashes a copy here; dmOpenHfWindow() emits it on HF at
	 * the next window so flood/discovery reaches both bands (plan §1.3). */
	volatile bool _hf_tx_pending;
	volatile bool _hf_tx_active; /* HF TX in flight (re-arms HF RX) */
	uint32_t _hf_tx_active_ms;   /* k_uptime_get_32() at latch set —
				      * dmWork's latch-timeout backstop
				      * (DM_HF_TX_LATCH_TIMEOUT_MS) */
	uint8_t _hf_tx_buf[256];
	uint16_t _hf_tx_len;
	void queueHfCopy(const uint8_t *bytes, int len);
	bool startHfTx(const uint8_t *bytes, int len);

	/* ── HF diag log channel (2026-08-16, WORKPLAN §8.3) ──────── */
	/* Small secondary stash slot: error lines ride the next TDM window
	 * after any flood HF copy.  Kept separate from _hf_tx_* so a diag
	 * stash can never clobber a pending flood copy. */
	volatile bool _diag_pending;
	uint8_t _diag_buf[64];
	uint16_t _diag_len;

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
