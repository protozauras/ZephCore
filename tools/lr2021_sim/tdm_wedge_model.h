/*
 * SPDX-License-Identifier: MIT
 *
 * tdm_wedge_model.h — behavioural model of the DualBandRadio TDM glue
 * and the LR2021 lr20xx_switch_band() guard, built to prove the hang
 * mechanism of ROOTCAUSE §9 (TDM state divergence -> chip pinned on
 * 2.4 GHz -> 868 deaf -> TX gated -> queue 32) and to validate the
 * three fix candidates BEFORE they touch the firmware.
 *
 * What is modelled (mirrors the production code paths):
 *   - lr20xx_switch_band(): the silent-bail-on-TX-in-flight guard
 *     (commit 01b3df9).  Pre-fix it returns void -> the adapter can
 *     never learn the switch did not happen; post-fix it returns a
 *     status and the adapter gates its state flips on it (fix 1).
 *   - dmOpenHfWindow()/dmCloseHfWindow(): the _hf_open flips.
 *   - dmWork(): the _hf_tx_active latch gate; post-fix the latch
 *     force-clears after TDMW_LATCH_TIMEOUT_MS (fix 2).
 *   - startHfTx(): the forced-HF modem + latch set.
 *   - recoverRxState(): the checkSend CAD-timeout recovery; post-fix
 *     it always rebuilds the PRIMARY band config (fix 3).
 *
 * `fix_enabled` selects pre-fix (01b3df9) vs post-fix behaviour, so
 * the same scenarios demonstrate the wedge AND its healing.
 *
 * This is a pure-C deterministic model (no Zephyr) — it is NOT a
 * byte-for-byte copy of DualBandRadio.cpp (that file is C++/Zephyr
 * and cannot compile here); the production glue code must be kept in
 * sync manually.  The pure helpers (dm_step, dm_default_config) are
 * already shared verbatim via dualband_tdm.h.
 */

#ifndef ZEPHCORE_TDM_WEDGE_MODEL_H
#define ZEPHCORE_TDM_WEDGE_MODEL_H

#include <stdint.h>
#include <stdbool.h>

#define TDMW_PRIMARY_FREQ_HZ 869618000u
#define TDMW_HF_FREQ_HZ      2450000000u
/* Mirrors DM_HF_TX_LATCH_TIMEOUT_MS in DualBandRadio.cpp (5 s — the
 * base TX wait thread's own TX_TIMEOUT_MS, so a legitimately pending
 * TX always clears the latch itself before this fires). */
#define TDMW_LATCH_TIMEOUT_MS 5000u

typedef enum {
    TDMW_BAND_PRIMARY = 0,
    TDMW_BAND_HF,
} tdwm_band_t;

typedef struct {
    /* model knobs */
    bool fix_enabled;            /* false = 01b3df9 behaviour */

    /* chip state (LR2021 driver data) */
    tdwm_band_t chip_band;       /* where the chip is actually tuned */
    bool chip_tx_active;         /* driver data->tx_active (TX in flight) */
    bool rx_868_armed;           /* true = sub-GHz RX armed (868 hears) */

    /* adapter state (DualBandRadio) */
    bool hf_open;                /* TDM believes the HF window is open */
    bool hf_tx_active;           /* _hf_tx_active latch */
    uint32_t hf_tx_active_ms;    /* k_uptime at latch set */
    bool force_hf_modem;         /* _force_hf_modem (buildModemConfig HF) */
    bool hf_tx_pending;          /* stashed HF copy awaiting a window */

    /* clock */
    uint32_t now_ms;

    /* counters (for assertions) */
    int switch_attempts;
    int switch_bails;
    int latch_force_clears;
} tdwm_model_t;

void tdwm_init(tdwm_model_t *m, bool fix_enabled);

/* driver: lr20xx_switch_band() — 0 ok; -1 if TX in flight (post-fix
 * reports it; pre-fix silently "succeeds" while the band never moves) */
int tdwm_switch_band(tdwm_model_t *m, tdwm_band_t band);

/* adapter glue (mirrors DualBandRadio.cpp) */
void tdwm_dm_open_hf_window(tdwm_model_t *m);
void tdwm_dm_close_hf_window(tdwm_model_t *m);
void tdwm_dm_work(tdwm_model_t *m);
void tdwm_start_hf_tx(tdwm_model_t *m);
void tdwm_on_tx_complete(tdwm_model_t *m);

/* checkSend CAD-timeout recovery (LoRaRadioBase::recoverRxState) */
void tdwm_recover_rx_state(tdwm_model_t *m);

/* scenario helpers */
void tdwm_subghz_tx_start(tdwm_model_t *m); /* mesh TX begins on primary */
void tdwm_subghz_tx_done(tdwm_model_t *m);  /* mesh TX completes normally */
void tdwm_hf_tx_lost(tdwm_model_t *m);      /* chip finishes HF TX, the
                                             * completion NEVER reaches
                                             * onTxComplete (lost DIO1
                                             * edge / completion race) */

/* True when the TDM state matches the chip (the invariant the fixes
 * restore): window open <=> chip on HF. */
bool tdwm_state_truthful(const tdwm_model_t *m);

#endif /* ZEPHCORE_TDM_WEDGE_MODEL_H */
