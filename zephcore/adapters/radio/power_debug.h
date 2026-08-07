/*
 * SPDX-License-Identifier: MIT
 * ZephCore power-debug instrumentation (L5 elektra, POWER_DEBUG_PLANAS.md).
 *
 * Tracks radio state residency times (sub-GHz RX / HF TDM window RX /
 * sub-GHz TX / HF TX / standby), TX counts and main-loop wakeups, and
 * prints a parseable POWER_SUMMARY every 10 s so the agent can compute
 * where the current draw goes (formula in POWER_DEBUG_PLANAS.md §2.4).
 *
 * Entirely compiled out when CONFIG_ZEPHCORE_POWER_DEBUG=n — headers
 * collapse to inline no-ops, the .cpp becomes an empty TU, and normal
 * builds see zero code/RAM/time impact.
 */

#pragma once

#include <zephyr/kernel.h>

/* Radio residency states.  HF window RX and HF TX both happen while the
 * TDM window is open; hf_window_ms in the summary counts RX_HF only,
 * HF TX time is reported separately (tx_hf_ms). */
enum power_debug_state {
	POWER_STATE_STANDBY = 0,
	POWER_STATE_RX_SUBGHZ,
	POWER_STATE_RX_HF,
	POWER_STATE_TX_SUBGHZ,
	POWER_STATE_TX_HF,
};

#if IS_ENABLED(CONFIG_ZEPHCORE_POWER_DEBUG)

/* Transition the radio into `state` (accumulates time spent in the
 * previous state since its entry).  Safe from any thread/ISR context. */
void power_debug_enter_state(enum power_debug_state state);

/* +1 main-loop wakeup counter (repeater_event_loop iteration). */
void power_debug_main_loop_tick(void);

/* Battery voltage source (mV) — passed by main so the summary can log
 * charger cycling without power_debug depending on the board adapter. */
typedef uint16_t (*power_debug_batt_fn)(void);

/* Start the 10 s POWER_SUMMARY + GPIO-state reporter.  Call once from
 * main().  batt_mv_fn may be NULL (battery line omitted). */
void power_debug_init(power_debug_batt_fn batt_mv_fn);

#else /* CONFIG_ZEPHCORE_POWER_DEBUG=n: no-ops */

static inline void power_debug_enter_state(enum power_debug_state state)
{
	(void)state;
}
static inline void power_debug_main_loop_tick(void) {}
typedef uint16_t (*power_debug_batt_fn)(void);
static inline void power_debug_init(power_debug_batt_fn batt_mv_fn)
{
	(void)batt_mv_fn;
}

#endif /* CONFIG_ZEPHCORE_POWER_DEBUG */
