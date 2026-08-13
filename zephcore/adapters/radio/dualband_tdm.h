/*
 * SPDX-License-Identifier: MIT
 * Pure dual-band TDM (time-division) window scheduler — no Zephyr / no
 * hardware dependencies.
 *
 * Shared VERBATIM between the DualBandRadio adapter (production, compiled
 * into the app) and the LR2021 sandbox (tools/lr2021_sim, which #includes
 * this header directly) so the tested logic can never drift from the
 * production logic (pitfall #21).
 *
 * Model (LR2021_DUALBAND_RESEARCH.md §L3-U4B / §1.1): a single LR2021
 * radio time-shares two bands.  The primary band (sub-GHz, from prefs)
 * is listened to continuously; every hold_ms the machine opens a short
 * HF (2.4 GHz) RX window for hf_window_ms, then returns to the primary
 * band.  If a packet is mid-capture when the window ends, the window is
 * extended in hf_extend_step_ms steps up to hf_max_extend_ms so in-flight
 * HF packets are never chopped.
 */

#ifndef ZEPHCORE_DUALBAND_TDM_H
#define ZEPHCORE_DUALBAND_TDM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* HF (2.4 GHz) RX window preset — L3-U4B, plan §1.4.  The secondary band
 * is FIXED per the architecture; the primary band comes from prefs. */
#define DM_HF_FREQ_HZ   2450000000UL /* 2450.0 MHz, ISM centre */
#define DM_HF_BW_KHZ    500u         /* BW_500_KHZ — LR2021 native 2.4 GHz */
#define DM_HF_SF        8u
#define DM_HF_CR        1u           /* CR_4_5 (Zephyr enum == chip code) */
#define DM_HF_TX_PWR_DM 12           /* LR2021 HF absolute max +12 dBm */

typedef enum {
	DM_STATE_IDLE = 0,  /* listening on the primary band */
	DM_STATE_HF_OPEN,   /* HF window open (at nominal length) */
	DM_STATE_HF_EXTEND, /* HF window extended (RX was busy) */
} dm_state_t;

typedef enum {
	DM_DECISION_OPEN = 0,  /* switch to HF now */
	DM_DECISION_CLOSE,     /* switch back to the primary band now */
	DM_DECISION_EXTEND,    /* hold HF a little longer, re-check */
	DM_DECISION_SKIP,      /* skip this window, stay on primary (busy) */
} dm_decision_t;

typedef struct {
	uint32_t hf_window_ms;      /* nominal HF listen window */
	uint32_t hf_extend_step_ms; /* extra us before re-checking a busy HF */
	uint32_t hf_max_extend_ms;  /* hard cap on TOTAL time in HF window */
	uint32_t hold_ms;           /* primary dwell between windows */
} dm_config_t;

/* Default timing (per plan: HF window 70 ms every 1-2 s; extend cap covers
 * a worst-case HF packet ~29 ms plus margin; SKIP retry so a busy primary
 * doesn't starve the HF window for a full hold period).
 *
 * Diagnostic builds may override only the nominal RX window with
 * CONFIG_ZEPHCORE_TDM_HF_WINDOW_MS=<N> (the production default remains 70 ms).
 * This is intentionally a compile-time escape hatch: it lets a companion
 * listen wide enough to distinguish fixed-phase TDM non-overlap from an HF
 * RF-path failure without changing the normal scheduler or radio parameters. */
#ifndef CONFIG_ZEPHCORE_TDM_HF_WINDOW_MS
/* The production Kconfig symbol exists in firmware builds; the sandbox
 * includes this header without Zephyr/Kconfig, so keep the same default. */
#define CONFIG_ZEPHCORE_TDM_HF_WINDOW_MS 70u
#endif

static inline dm_config_t dm_default_config(void)
{
	dm_config_t cfg;
	cfg.hf_window_ms      = (uint32_t)CONFIG_ZEPHCORE_TDM_HF_WINDOW_MS;
	cfg.hf_extend_step_ms = 25u;
	cfg.hf_max_extend_ms  = 150u;
	cfg.hold_ms           = 1500u;
	return cfg;
}

/* How long to wait before re-trying after a SKIP (radio was busy).  Kept
 * much shorter than hold_ms so a noisy primary band does not starve the HF
 * window; 125 ms << 1.5 s. */
#define DM_SKIP_RETRY_MS 125u

/*
 * One scheduler step.
 *
 * @param state            current dm_state_t
 * @param radio_in_rx      true when the radio is armed for continuous RX
 *                         (false while TX is active / mid-config)
 * @param radio_receiving  true when a preamble/packet is mid-capture
 *                         (chip reports RX activity) — non-destructive read
 * @param time_in_state_ms elapsed ms in the current state (caller tracks)
 * @param cfg              timing config (never NULL)
 */
static inline dm_decision_t dm_step(dm_state_t state, bool radio_in_rx,
				    bool radio_receiving,
				    uint32_t time_in_state_ms,
				    const dm_config_t *cfg)
{
	switch (state) {
	case DM_STATE_IDLE:
		/* Open the HF window only when the radio is continuously
		 * listening on the primary band AND not mid primary capture
		 * (switching mid capture would chop a sub-GHz packet). */
		if (!radio_in_rx || radio_receiving) {
			return DM_DECISION_SKIP;
		}
		return DM_DECISION_OPEN;

	case DM_STATE_HF_OPEN:
	case DM_STATE_HF_EXTEND:
		/* Is an HF packet mid-capture?  Hold the window open (and
		 * keep extending) until it clears or the cap is reached. */
		if (radio_receiving &&
		    time_in_state_ms < cfg->hf_max_extend_ms) {
			return DM_DECISION_EXTEND;
		}
		return DM_DECISION_CLOSE;

	default:
		return DM_DECISION_CLOSE;
	}
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_DUALBAND_TDM_H */
