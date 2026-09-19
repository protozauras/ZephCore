/*
 * SPDX-License-Identifier: MIT
 *
 * test_tdm_wedge.c — sandbox tests for the TDM wedge mechanism and the
 * three fix candidates (ROOTCAUSE §9, hang #4 2026-08-11):
 *
 *   1. switch_band bail -> TDM state truthfulness (fix 1)
 *   2. _hf_tx_active latch timeout -> dmWork can always close (fix 2)
 *   3. CAD-timeout recoverRxState -> PRIMARY band config (fix 3)
 *
 * Each scenario runs TWICE: once on the pre-fix behaviour (fix_enabled
 * = false — asserts the wedge reproduces, i.e. the bug the sandbox
 * documents) and once on the post-fix behaviour (fix_enabled = true —
 * asserts the system heals).  The production fix is accepted only when
 * the post-fix assertions hold AND the pre-fix assertions reproduce the
 * exact hang signature.
 *
 * Model: tdm_wedge_model.{h,c} — pure C, no Zephyr, deterministic.
 */

#include "tdm_wedge_model.h"
#include <stdio.h>
#include <stdlib.h>

/* Same tiny assert helpers as test_lr2021_driver.c (kept local so this
 * file stays self-contained). */

static int g_failures = 0;
static int g_tests_run = 0;

#define LOG(fmt, ...)  do { printf(fmt "\n", ##__VA_ARGS__); } while (0)
#define PASS(name)     do { LOG("  [PASS] %s", name); g_tests_run++; } while (0)
#define FAIL(name, fmt, ...) do {                                              \
        LOG("  [FAIL] %s: " fmt, name, ##__VA_ARGS__);                         \
        g_failures++; g_tests_run++;                                           \
    } while (0)
#define CHECK(cond, name, fmt, ...) do {                                        \
        if (!(cond)) { FAIL(name, fmt, ##__VA_ARGS__); return; }                \
    } while (0)

/* ── Scenario 1: switch_band bail while a sub-GHz TX is in flight ──── */

static void test_wedge_switch_bail_state_integrity(void)
{
    /* POST-FIX: the OPEN decision was made while the radio was idle,
     * but a mesh TX starts in the microsecond window BEFORE the switch
     * executes (the 01b3df9 race: chip-level tx_active set between the
     * dm_step gate and lr20xx_switch_band).  The driver guard bails
     * (-EBUSY); the adapter must NOT flip _hf_open — the chip never
     * left the primary band. */
    tdwm_model_t m;
    tdwm_init(&m, true);
    tdwm_subghz_tx_start(&m);   /* TX in flight when the switch runs */
    tdwm_dm_open_hf_window(&m); /* the already-decided OPEN executes */

    CHECK(m.switch_bails == 1, "fix_switch_bail_detected",
          "post-fix must report the bail, got %d attempts %d",
          m.switch_bails, m.switch_attempts);
    CHECK(!m.hf_open, "fix_window_stays_closed",
          "window must stay closed when the switch bailed");
    CHECK(m.chip_band == TDMW_BAND_PRIMARY, "fix_chip_on_primary",
          "chip must still be on the primary band");
    CHECK(tdwm_state_truthful(&m), "fix_state_truthful",
          "TDM state must match the chip after a bail");

    /* PRE-FIX: same race — the switch bails SILENTLY (void return)
     * but dmOpenHfWindow flips _hf_open anyway: TDM believes the chip
     * is on HF while it is actually still on the primary band.  This
     * divergence is the entry wedge of every hang. */
    tdwm_model_t p;
    tdwm_init(&p, false);
    tdwm_subghz_tx_start(&p);
    tdwm_dm_open_hf_window(&p);

    CHECK(p.hf_open, "prefix_window_lies_open",
          "pre-fix: _hf_open set despite the chip never moving (bug)");
    CHECK(p.chip_band == TDMW_BAND_PRIMARY, "prefix_chip_still_primary",
          "pre-fix: chip is still on primary while TDM says HF");
    CHECK(!tdwm_state_truthful(&p), "prefix_divergence_demonstrated",
          "pre-fix: TDM state diverges from the chip (REPRODUCED)");

    PASS("tdm_wedge_switch_bail_state_integrity");
}

/* ── Scenario 2: lost TX completion -> latch timeout heals ─────────── */

static void test_wedge_latch_timeout_heals(void)
{
    /* POST-FIX: an in-window HF TX starts (forced-HF modem + latch),
     * the chip finishes it, but the completion is LOST (never reaches
     * onTxComplete — the DIO1 edge loss / completion-before-latch
     * race).  The latch must force-clear after TDMW_LATCH_TIMEOUT_MS
     * so dmWork can close the window and the chip returns to 868. */
    tdwm_model_t m;
    tdwm_init(&m, true);
    m.hf_tx_pending = true;
    tdwm_dm_work(&m);   /* OPEN + startHfTx: chip -> HF, TX in flight */

    CHECK(m.hf_open, "fix_window_open", "window must be open");
    CHECK(m.hf_tx_active, "fix_latch_set", "latch must be set");
    CHECK(m.force_hf_modem, "fix_force_set", "forced-HF modem must be set");
    CHECK(m.chip_band == TDMW_BAND_HF, "fix_chip_on_hf", "chip on HF");
    CHECK(tdwm_state_truthful(&m), "fix_truthful_during_tx",
          "state truthful while TX in flight");

    /* chip finishes; completion lost */
    tdwm_hf_tx_lost(&m);
    /* latch still set -> dmWork defers (this is the wedge) */
    m.now_ms += TDMW_LATCH_TIMEOUT_MS - 100u;
    tdwm_dm_work(&m);
    CHECK(m.hf_tx_active, "fix_latch_holds_before_timeout",
          "latch must hold until the timeout");
    CHECK(m.hf_open, "fix_window_open_before_timeout",
          "window still open before the timeout fires");

    /* past the timeout -> force-clear -> machine reaches CLOSE */
    m.now_ms += 200u;
    tdwm_dm_work(&m);
    CHECK(!m.hf_tx_active, "fix_latch_force_cleared",
          "latch must force-clear after the timeout");
    CHECK(m.latch_force_clears == 1, "fix_latch_cleared_once",
          "exactly one force-clear, got %d", m.latch_force_clears);
    CHECK(!m.hf_open, "fix_window_closed", "window must close");
    CHECK(m.chip_band == TDMW_BAND_PRIMARY, "fix_chip_back_primary",
          "chip must return to the primary band");
    CHECK(m.rx_868_armed, "fix_rx_rearmed", "868 RX must be re-armed");
    CHECK(tdwm_state_truthful(&m), "fix_truthful_after_heal",
          "state truthful after healing");

    /* PRE-FIX: the same lost completion wedges FOREVER — the latch has
     * no timeout, dmWork defers endlessly, the window never closes,
     * the chip stays on 2.4 GHz and 868 stays deaf. */
    tdwm_model_t p;
    tdwm_init(&p, false);
    p.hf_tx_pending = true;
    tdwm_dm_work(&p);
    tdwm_hf_tx_lost(&p);

    int i;
    for (i = 0; i < 50; i++) {   /* 50 work ticks, 100 ms apart */
        p.now_ms += 100u;
        tdwm_dm_work(&p);
    }
    CHECK(p.hf_tx_active, "prefix_latch_stuck_forever",
          "pre-fix: latch never clears (bug)");
    CHECK(p.hf_open, "prefix_window_stuck_open",
          "pre-fix: window never closes (bug)");
    CHECK(p.chip_band == TDMW_BAND_HF, "prefix_chip_pinned_hf",
          "pre-fix: chip pinned on 2.4 GHz (bug)");
    CHECK(!p.rx_868_armed, "prefix_868_deaf",
          "pre-fix: 868 deaf = hang reproduced (REPRODUCED)");

    PASS("tdm_wedge_latch_timeout_heals");
}

/* ── Scenario 3: CAD-timeout recovery returns to the primary band ──── */

static void test_wedge_cad_recovery_returns_primary(void)
{
    /* POST-FIX: stuck state (chip on HF, forced-HF modem still set,
     * 868 deaf) + a checkSend CAD timeout -> recoverRxState MUST
     * rebuild the PRIMARY band config, never the cached HF one. */
    tdwm_model_t m;
    tdwm_init(&m, true);
    m.hf_open = true;
    m.chip_band = TDMW_BAND_HF;
    m.force_hf_modem = true;
    m.rx_868_armed = false;

    tdwm_recover_rx_state(&m);

    CHECK(!m.force_hf_modem, "fix_cad_clears_force",
          "recovery must drop the forced-HF modem");
    CHECK(m.chip_band == TDMW_BAND_PRIMARY, "fix_cad_back_primary",
          "recovery must re-tune the chip to the primary band");
    CHECK(m.rx_868_armed, "fix_cad_rearms_868",
          "recovery must re-arm 868 RX");

    /* The window flag is closed by the next dmWork tick (the latch
     * timeout or the normal CLOSE decision) — after that the TDM state
     * is truthful again and the system is fully healed. */
    tdwm_dm_work(&m);
    CHECK(!m.hf_open, "fix_cad_closes_window",
          "the next TDM tick must close the stale window flag");
    CHECK(tdwm_state_truthful(&m), "fix_cad_truthful",
          "TDM state truthful after recovery + one tick");
    CHECK(m.chip_band == TDMW_BAND_PRIMARY && m.rx_868_armed,
          "fix_cad_healed", "system healed: 868 armed, no wedge");

    /* PRE-FIX: same recovery re-applies the HF preset (the
     * `configureRx: freq=2450000000` loop of hang #4) — the chip is
     * re-pinned on 2.4 GHz and 868 stays deaf forever. */
    tdwm_model_t p;
    tdwm_init(&p, false);
    p.hf_open = true;
    p.chip_band = TDMW_BAND_HF;
    p.force_hf_modem = true;
    p.rx_868_armed = false;

    tdwm_recover_rx_state(&p);

    CHECK(p.chip_band == TDMW_BAND_HF, "prefix_cad_repins_hf",
          "pre-fix: recovery re-pins 2.4 GHz (bug)");
    CHECK(!p.rx_868_armed, "prefix_cad_868_still_deaf",
          "pre-fix: 868 stays deaf after recovery (bug)");

    PASS("tdm_wedge_cad_recovery_returns_primary");
}

/* ── End-to-end: full hang cycle with the post-fix firmware ────────── */

static void test_wedge_end_to_end_heals(void)
{
    /* Complete hang #4 cycle: traffic burst -> sub-GHz TX in flight ->
     * OPEN decision collides -> HF TX starts -> completion lost ->
     * latch timeout -> CAD recovery.  The post-fix invariant: TDM state
     * stays truthful at every step and the system returns to 868. */
    tdwm_model_t m;
    tdwm_init(&m, true);

    /* quiet ether, normal window cycle */
    tdwm_dm_work(&m);                 /* OPEN: chip -> HF */
    CHECK(m.hf_open && m.chip_band == TDMW_BAND_HF, "e2e_window_opens",
          "window opens, chip follows");
    m.now_ms += 70u;
    tdwm_dm_work(&m);                 /* CLOSE: chip -> primary */
    CHECK(!m.hf_open && m.chip_band == TDMW_BAND_PRIMARY, "e2e_window_closes",
          "window closes, chip follows");
    CHECK(m.rx_868_armed, "e2e_rx_armed", "868 armed in idle");

    /* traffic burst: a sub-GHz TX starts in the window between the
     * OPEN decision and the switch (the 01b3df9 race) */
    tdwm_subghz_tx_start(&m);
    tdwm_dm_open_hf_window(&m);   /* already-decided OPEN executes */
    CHECK(!m.hf_open, "e2e_bail_keeps_closed",
          "switch bail must leave the window closed");
    CHECK(m.chip_band == TDMW_BAND_PRIMARY, "e2e_chip_primary",
          "chip never left the primary band");
    tdwm_subghz_tx_done(&m);

    /* next window: stashed HF copy TXes, completion lost */
    m.hf_tx_pending = true;
    tdwm_dm_work(&m);                 /* OPEN + startHfTx */
    CHECK(m.hf_open && m.hf_tx_active, "e2e_hftx_started",
          "HF TX started inside the window");
    tdwm_hf_tx_lost(&m);

    /* CAD-timeout recovery fires first (checkSend ~3 s loop): with fix
     * 3 it re-arms 868 even while the latch is still counting down. */
    m.now_ms += 3000u;
    tdwm_recover_rx_state(&m);
    CHECK(m.chip_band == TDMW_BAND_PRIMARY, "e2e_cad_back_primary",
          "CAD recovery returns the chip to 868");
    CHECK(m.rx_868_armed, "e2e_cad_868_armed",
          "CAD recovery re-arms 868 RX");

    /* then the latch timeout lets dmWork close the window state */
    m.now_ms += 2500u;                /* total 5500 ms > 5000 ms */
    tdwm_dm_work(&m);
    CHECK(!m.hf_tx_active, "e2e_latch_cleared", "latch force-cleared");
    CHECK(!m.hf_open, "e2e_window_closed", "window state closed");
    CHECK(tdwm_state_truthful(&m), "e2e_truthful",
          "TDM state truthful at the end");
    CHECK(m.chip_band == TDMW_BAND_PRIMARY && m.rx_868_armed,
          "e2e_healed", "system healed: 868 armed, no wedge");

    PASS("tdm_wedge_end_to_end_heals");
}

void run_tdm_wedge_tests(void)
{
    LOG("---- TDM wedge mechanism (ROOTCAUSE 2026-08-11 hang #4) ----");
    test_wedge_switch_bail_state_integrity();
    test_wedge_latch_timeout_heals();
    test_wedge_cad_recovery_returns_primary();
    test_wedge_end_to_end_heals();
    LOG("---- tdm wedge: %d run, %d failures ----", g_tests_run, g_failures);
}