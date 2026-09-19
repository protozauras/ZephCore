/*
 * SPDX-License-Identifier: MIT
 *
 * tdm_wedge_model.c — model implementation.  See the header for the
 * mapping to production code.  The pre-fix vs post-fix behaviour is
 * selected by `fix_enabled` so one scenario suite proves both the
 * wedge (pre-fix assertions) and the healing (post-fix assertions).
 */

#include "tdm_wedge_model.h"

void tdwm_init(tdwm_model_t *m, bool fix_enabled)
{
    m->fix_enabled = fix_enabled;
    m->chip_band = TDMW_BAND_PRIMARY;
    m->chip_tx_active = false;
    m->rx_868_armed = true;
    m->hf_open = false;
    m->hf_tx_active = false;
    m->hf_tx_active_ms = 0u;
    m->force_hf_modem = false;
    m->hf_tx_pending = false;
    m->now_ms = 0u;
    m->switch_attempts = 0;
    m->switch_bails = 0;
    m->latch_force_clears = 0;
}

/* lr20xx_switch_band(): the guard is identical pre/post fix — a TX in
 * flight means the band never moves.  What changed (fix 1) is that the
 * failure is REPORTED instead of being a silent void return, so the
 * adapter can stop lying about _hf_open/_dm_state. */
int tdwm_switch_band(tdwm_model_t *m, tdwm_band_t band)
{
    m->switch_attempts++;
    if (m->chip_tx_active) {
        m->switch_bails++;
        return m->fix_enabled ? -1 : 0;  /* pre-fix: caller cannot see it */
    }
    m->chip_band = band;
    m->rx_868_armed = (band == TDMW_BAND_PRIMARY);
    return 0;
}

/* dmOpenHfWindow(): lean band switch to the HF preset, then declare the
 * window open.  Fix 1: the state flips happen ONLY when the switch
 * actually moved the chip; otherwise retry later (window stays closed,
 * the chip is still on the primary band). */
void tdwm_dm_open_hf_window(tdwm_model_t *m)
{
    int ret = tdwm_switch_band(m, TDMW_BAND_HF);
    if (m->fix_enabled && ret != 0) {
        return;  /* caller reschedules — chip still on primary */
    }
    m->hf_open = true;
    if (m->hf_tx_pending) {
        m->hf_tx_pending = false;
        tdwm_start_hf_tx(m);
    }
}

/* dmCloseHfWindow(): switch back to the primary band and declare the
 * window closed.  Fix 2 (defensive): close ALWAYS rebuilds the primary
 * config — a leftover forced-HF modem would otherwise make the "close"
 * switch a no-op on the same band.  Fix 1: if the switch bailed (e.g.
 * an HF TX still in flight), the window MUST stay open — the chip is
 * still on HF. */
void tdwm_dm_close_hf_window(tdwm_model_t *m)
{
    if (m->fix_enabled) {
        m->force_hf_modem = false;
    }
    int ret = tdwm_switch_band(m, TDMW_BAND_PRIMARY);
    if (m->fix_enabled && ret != 0) {
        return;  /* chip still HF — keep the window state truthful */
    }
    m->hf_open = false;
}

/* dmWork(): one TDM tick.  The _hf_tx_active latch gate used to defer
 * FOREVER (no log, no timeout) — the confirmed wedge: dmWork stalls,
 * the window is never closed, the chip stays on 2.4 GHz.  Fix 2: the
 * latch carries a timestamp and force-clears after
 * TDMW_LATCH_TIMEOUT_MS (also dropping the forced-HF modem), so the
 * machine can always reach the CLOSE decision. */
void tdwm_dm_work(tdwm_model_t *m)
{
    if (m->hf_tx_active) {
        if (!m->fix_enabled) {
            return;  /* pre-fix: the latch defers FOREVER (the wedge) */
        }
        uint32_t age = m->now_ms - m->hf_tx_active_ms;
        if (age < TDMW_LATCH_TIMEOUT_MS) {
            return;  /* retry shortly (DM_SKIP_RETRY_MS) */
        }
        m->hf_tx_active = false;
        m->force_hf_modem = false;
        m->latch_force_clears++;
    }

    /* in_rx = radio armed for continuous RX && not mid TX;
     * receiving = preamble mid-capture (false in these scenarios) */
    bool in_rx = !m->chip_tx_active;
    bool receiving = false;

    if (!m->hf_open) {
        if (!in_rx || receiving) {
            return;  /* DM_DECISION_SKIP */
        }
        tdwm_dm_open_hf_window(m);  /* DM_DECISION_OPEN */
    } else {
        tdwm_dm_close_hf_window(m); /* DM_DECISION_CLOSE (nominal window) */
    }
}

/* startHfTx(): force the HF modem preset, kick the chip TX, set the
 * latch.  The latch is set AFTER the base call returns (production),
 * so a completion that already ran (fast/failed TX) must never be
 * latched — post-fix the TX-in-flight probe guards the set. */
void tdwm_start_hf_tx(tdwm_model_t *m)
{
    if (m->chip_tx_active) {
        return;  /* base isRadioReady() gate */
    }
    m->force_hf_modem = true;
    m->chip_tx_active = true;  /* base startSendRaw: TX in flight */

    /* Fix (race guard): if the TX already completed before we could
     * latch, latching it would wedge dmWork forever. */
    if (m->fix_enabled && !m->chip_tx_active) {
        m->force_hf_modem = false;
        return;
    }

    m->hf_tx_active = true;
    m->hf_tx_active_ms = m->now_ms;
}

/* onTxComplete(): base TX-wait thread hook — always runs for a normal
 * TX (all k_poll paths), clears the latch and the forced-HF modem. */
void tdwm_on_tx_complete(tdwm_model_t *m)
{
    m->force_hf_modem = false;
    m->hf_tx_active = false;
    m->chip_tx_active = false;
}

/* checkSend CAD-timeout recovery (LoRaRadioBase::recoverRxState()):
 * walks the chip back through REST and re-arms RX.  Pre-fix, with the
 * forced-HF modem still set, the re-arm re-applies the 2.4 GHz preset
 * — the exact `configureRx: freq=2450000000` loop of hang #4.  Fix 3:
 * the recovery ALWAYS rebuilds the PRIMARY band config. */
void tdwm_recover_rx_state(tdwm_model_t *m)
{
    if (m->fix_enabled) {
        m->force_hf_modem = false;
        m->chip_band = TDMW_BAND_PRIMARY;
        m->rx_868_armed = true;
    } else {
        if (m->force_hf_modem) {
            m->chip_band = TDMW_BAND_HF;  /* re-pins 2.4 GHz — hang loop */
            m->rx_868_armed = false;
        } else {
            m->chip_band = TDMW_BAND_PRIMARY;
            m->rx_868_armed = true;
        }
    }
}

/* scenario helpers */

void tdwm_subghz_tx_start(tdwm_model_t *m)
{
    m->chip_tx_active = true;
    m->rx_868_armed = false;
}

void tdwm_subghz_tx_done(tdwm_model_t *m)
{
    m->chip_tx_active = false;
    m->rx_868_armed = true;
}

/* The chip finishes an HF TX but the completion never reaches
 * onTxComplete() — the lost DIO1 TX_DONE edge / completion-before-latch
 * race.  The chip stops TXing; the adapter latch stays set. */
void tdwm_hf_tx_lost(tdwm_model_t *m)
{
    m->chip_tx_active = false;
    /* deliberately NOT calling tdwm_on_tx_complete() */
}

bool tdwm_state_truthful(const tdwm_model_t *m)
{
    return m->hf_open == (m->chip_band == TDMW_BAND_HF);
}