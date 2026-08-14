/*
 * SPDX-License-Identifier: MIT
 * ZephCore - Link Test (neutral 2.4 GHz range tester, no mesh)
 *
 * Minimal app for the 2.4 GHz field range test (RANGE_TEST_PLAN.md):
 *   - NO mesh, NO TDM, NO sub-GHz TX — pure 2.4 GHz RF only.
 *   - TX role  (CONFIG_ZEPHCORE_LINK_TEST_ROLE="tx"):
 *       every CONFIG_ZEPHCORE_LINK_TEST_TX_MS (default 300) sends an 8-byte
 *       frame (magic 0x5A + uint32 LE sequence) on the fixed preset
 *       2450 MHz / BW500 / SF8 / CR 4/5 / +12 dBm.
 *   - RX role  (CONFIG_ZEPHCORE_LINK_TEST_ROLE="rx"):
 *       continuous RX; per packet prints "RSSI=-XX SNR=+Y seq=NNN";
 *       after 1.5 s of silence prints "NO SIGNAL (tyla X s)" once per second.
 *
 * The 2.4 GHz preset is hard-pinned in pin_2g4_preset() — this build must
 * NEVER transmit on sub-GHz 868 (range-test rule, RANGE_TEST_PLAN.md §6:
 * "Test-buildas NIEKADA nesiunčia 868").
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_link_test, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

/* Radio + adapter includes (same shared header the mesh roles use;
 * pulls LR2021Radio via CONFIG_ZEPHCORE_RADIO_LR2021). */
#include <mesh/RadioIncludes.h>
#include <NodePrefs.h>

#ifdef ZEPHCORE_LORA

/* Same construction path as main_companion.cpp / main_repeater.cpp:
 * devicetree lora0 alias + NodePrefs + LR2021 adapter.  No mesh — the
 * radio adapter is used directly. */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::ZephyrBoard zephyr_board;
static NodePrefs link_prefs;
static mesh::LR2021Radio lora_radio(lora_dev, zephyr_board, &link_prefs);

#define LINK_TEST_MAGIC      0x5A
#define LINK_TEST_FRAME_LEN  8

static bool is_tx_role(void)
{
#if defined(CONFIG_ZEPHCORE_LINK_TEST_ROLE)
	return strcmp(CONFIG_ZEPHCORE_LINK_TEST_ROLE, "tx") == 0;
#else
	return true;
#endif
}

/* Hard-pin the 2.4 GHz preset — mirror of the initNodePrefs
 * CONFIG_ZEPHCORE_BAND_2G4 branch, applied unconditionally as defence in
 * depth: even if the Kconfig flag is missing, the link tester stays on
 * 2.4 GHz and never touches sub-GHz 868. */
static void pin_2g4_preset(void)
{
	link_prefs.freq = 2450.0f;          /* 2450 MHz ISM */
	link_prefs.bw = 500.0f;             /* BW500 */
	link_prefs.sf = 8;                  /* SF8 */
	link_prefs.cr = 5;                  /* CR 4/5 */
	link_prefs.tx_power_dbm = 12;       /* LR2021 HF PA absolute max */
}

/* ── TX role ──────────────────────────────────────────────────────────── */

static void link_tx_loop(void)
{
	uint8_t frame[LINK_TEST_FRAME_LEN] = { LINK_TEST_MAGIC };
	uint32_t seq = 0;

	printk("=== Link Test TX: 2450/BW500/SF8/CR4/5 @ +12 dBm ===\n");
	printk("frame %u B magic=0x%02X seq=u32LE interval=%d ms\n",
	       LINK_TEST_FRAME_LEN, LINK_TEST_MAGIC,
	       CONFIG_ZEPHCORE_LINK_TEST_TX_MS);

	for (;;) {
		/* Skip the slot while the previous TX is still in flight;
		 * startSendRaw itself gates on radio-busy / channel-active. */
		if (lora_radio.isSendComplete()) {
			frame[1] = (uint8_t)(seq & 0xFF);
			frame[2] = (uint8_t)((seq >> 8) & 0xFF);
			frame[3] = (uint8_t)((seq >> 16) & 0xFF);
			frame[4] = (uint8_t)((seq >> 24) & 0xFF);
			/* bytes 5..7 stay 0 (reserved) */
			if (lora_radio.startSendRaw(frame, sizeof(frame))) {
				printk("TX seq=%u\n", seq);
				seq++;
			}
		}
		k_sleep(K_MSEC(CONFIG_ZEPHCORE_LINK_TEST_TX_MS));
	}
}

/* ── RX role ──────────────────────────────────────────────────────────── */

static void link_rx_loop(void)
{
	uint8_t buf[64];
	int64_t last_pkt_ms = k_uptime_get();

	printk("=== Link Test RX: 2450/BW500/SF8/CR4/5 ===\n");

	for (;;) {
		int len = lora_radio.recvRaw(buf, sizeof(buf));
		if (len > 0) {
			last_pkt_ms = k_uptime_get();
			int rssi = (int)lora_radio.getLastRSSI();
			int snr = (int)lora_radio.getLastSNR();
			if (len >= 5 && buf[0] == LINK_TEST_MAGIC) {
				uint32_t seq = (uint32_t)buf[1]
					     | ((uint32_t)buf[2] << 8)
					     | ((uint32_t)buf[3] << 16)
					     | ((uint32_t)buf[4] << 24);
				printk("RSSI=%d SNR=%d seq=%u\n", rssi, snr, seq);
			} else {
				printk("RSSI=%d SNR=%d len=%d (foreign)\n",
				       rssi, snr, len);
			}
		} else {
			int64_t silent_ms = k_uptime_get() - last_pkt_ms;
			if (silent_ms > 1500) {
				printk("NO SIGNAL (tyla %lld s)\n",
				       (long long)(silent_ms / 1000));
				k_sleep(K_MSEC(1000));
			} else {
				k_sleep(K_MSEC(50));
			}
		}
	}
}

int main(void)
{
	printk("=== ZephCore Link Test starting (role=%s) ===\n",
	       is_tx_role() ? "tx" : "rx");

	initNodePrefs(&link_prefs);
	pin_2g4_preset();

	if (!device_is_ready(lora_dev)) {
		printk("ERROR: LoRa device not ready\n");
		return -1;
	}

	lora_radio.setPrefs(&link_prefs);
	lora_radio.begin();
	lora_radio.setRxBoost(true);           /* max sensitivity */
	lora_radio.enableRxDutyCycle(false);   /* continuous RX */

	printk("radio up: %u Hz BW%u SF%u CR4/%u pwr=%d dBm\n",
	       lora_radio.getActiveFrequencyHz(),
	       lora_radio.getActiveBandwidthKHzX10() / 10u,
	       lora_radio.getActiveSpreadingFactor(),
	       lora_radio.getActiveCodingRate(),
	       lora_radio.getConfiguredTxPower());

	if (is_tx_role()) {
		link_tx_loop();
	} else {
		link_rx_loop();
	}
	return 0;
}

#else /* !ZEPHCORE_LORA */

int main(void)
{
	printk("Link Test requires an LR2021 radio (lora0 alias)\n");
	return -1;
}

#endif /* ZEPHCORE_LORA */
