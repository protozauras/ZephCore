/*
 * SPDX-License-Identifier: MIT
 * ZephCore power-debug instrumentation — implementation.
 *
 * See power_debug.h.  State transitions are called from three contexts
 * (system workqueue — TDM scheduler, mesh thread — startSendRaw, TX wait
 * thread — onTxComplete), so all state mutation is spinlock-protected.
 * printk() is used for POWER_SUMMARY so it survives LOG_DEFAULT_LEVEL
 * reductions (B2 experiment in POWER_DEBUG_PLANAS.md).
 */

#include "power_debug.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(power_debug, LOG_LEVEL_INF);

#if IS_ENABLED(CONFIG_ZEPHCORE_POWER_DEBUG)

#define POWER_SUMMARY_PERIOD_MS 10000

static struct k_spinlock _lock;
static struct k_work_delayable _summary_work;

static enum power_debug_state _state = POWER_STATE_STANDBY;
static uint32_t _state_enter_ms; /* k_uptime_get_32() at state entry */
static uint32_t _acc[5];         /* accumulated ms per power_debug_state */
static uint32_t _tx_subghz_count;
static uint32_t _tx_hf_count;
static uint32_t _main_loops;

/* Accumulate the time spent in the current state (caller holds _lock). */
static void pd_accumulate(uint32_t now_ms)
{
	uint32_t elapsed = now_ms - _state_enter_ms; /* wraparound-safe */
	_acc[_state] += elapsed;
	_state_enter_ms = now_ms;
}

void power_debug_enter_state(enum power_debug_state state)
{
	k_spinlock_key_t key = k_spin_lock(&_lock);

	pd_accumulate((uint32_t)k_uptime_get_32());
	if (state == POWER_STATE_TX_SUBGHZ) {
		_tx_subghz_count++;
	} else if (state == POWER_STATE_TX_HF) {
		_tx_hf_count++;
	}
	_state = state;

	k_spin_unlock(&_lock, key);
}

void power_debug_main_loop_tick(void)
{
	k_spinlock_key_t key = k_spin_lock(&_lock);

	_main_loops++;

	k_spin_unlock(&_lock, key);
}

static void pd_print_pct10(uint64_t ms, uint64_t total_ms)
{
	/* One decimal: 1234 = 12.3 % */
	uint64_t permille = (ms * 1000u) / total_ms;

	printk("%llu.%llu", (unsigned long long)(permille / 10u),
	       (unsigned long long)(permille % 10u));
}

static void pd_summary_work_fn(struct k_work *work)
{
	uint32_t rx_ms, hf_ms, tx_ms, tx_hf_ms, stby_ms;
	uint32_t tx_sub, tx_hf, loops, uptime_ms;
	uint64_t total_ms;

	ARG_UNUSED(work);

	k_spinlock_key_t key = k_spin_lock(&_lock);
	pd_accumulate((uint32_t)k_uptime_get_32());

	uptime_ms = (uint32_t)k_uptime_get_32();
	rx_ms = _acc[POWER_STATE_RX_SUBGHZ];
	hf_ms = _acc[POWER_STATE_RX_HF];
	tx_ms = _acc[POWER_STATE_TX_SUBGHZ];
	tx_hf_ms = _acc[POWER_STATE_TX_HF];
	stby_ms = _acc[POWER_STATE_STANDBY];
	tx_sub = _tx_subghz_count;
	tx_hf = _tx_hf_count;
	loops = _main_loops;

	k_spin_unlock(&_lock, key);

	total_ms = (uint64_t)rx_ms + hf_ms + tx_ms + tx_hf_ms + stby_ms;
	if (total_ms == 0) {
		total_ms = 1;
	}

	printk("=== POWER_SUMMARY START ===\n");
	printk("uptime_ms=%u\n", (unsigned)uptime_ms);
	printk("rx_ms=%u hf_window_ms=%u tx_ms=%u tx_hf_ms=%u stby_ms=%u\n",
	       (unsigned)rx_ms, (unsigned)hf_ms, (unsigned)tx_ms,
	       (unsigned)tx_hf_ms, (unsigned)stby_ms);
	printk("tx_count_subghz=%u tx_count_hf=%u\n",
	       (unsigned)tx_sub, (unsigned)tx_hf);
	printk("main_loops=%u\n", (unsigned)loops);
	printk("radio_pct_rx=");
	pd_print_pct10(rx_ms, total_ms);
	printk(" radio_pct_hf=");
	pd_print_pct10(hf_ms, total_ms);
	printk(" radio_pct_tx=");
	pd_print_pct10(tx_ms, total_ms);
	printk(" radio_pct_txhf=");
	pd_print_pct10(tx_hf_ms, total_ms);
	printk(" radio_pct_stby=");
	pd_print_pct10(stby_ms, total_ms);
	printk("\n");
	printk("=== POWER_SUMMARY END ===\n");

	k_work_reschedule(&_summary_work, K_MSEC(POWER_SUMMARY_PERIOD_MS));
}

void power_debug_init(void)
{
	k_spinlock_key_t key = k_spin_lock(&_lock);

	_state = POWER_STATE_STANDBY;
	_state_enter_ms = (uint32_t)k_uptime_get_32();

	k_spin_unlock(&_lock, key);

	k_work_init_delayable(&_summary_work, pd_summary_work_fn);
	k_work_schedule(&_summary_work, K_MSEC(POWER_SUMMARY_PERIOD_MS));

	LOG_INF("power debug: POWER_SUMMARY every %d ms",
		POWER_SUMMARY_PERIOD_MS);
}

#endif /* CONFIG_ZEPHCORE_POWER_DEBUG */
