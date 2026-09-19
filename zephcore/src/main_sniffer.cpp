/*
 * SPDX-License-Identifier: MIT
 * ZephCore RF Sniffer — passive dual-band listener (NO TX, ever)
 *
 * Sniffer role for the NAMAI conversion (ACTION_PLAN.md, sniferis/):
 *   - 868 leg  ("lora868"): continuous LoRa RX on the NodePrefs default
 *     band (869.618 MHz); per-packet "RSSI= SNR= len= <hex>" lines.
 *   - 433 leg   ("ook433"): OOK packet-mode RX at 433.92 MHz via the
 *     lr20xx_sniffer_ook_* extension (poll mode — the LR2021 has no
 *     direct mode; the OOK envelope is captured as a bit stream in the
 *     RX FIFO, pulse run-lengths decode on the host).
 *   - wM-Bus leg ("mbus868"): native WM-BUS modem RX at 868.95 MHz
 *     (T1/C1 one-way meter mode per ZEPHCORE_SNIFFER_WMBUS_MODE) via
 *     the lr20xx_sniffer_wmbus_* extension; one "WMBUS JSON {...}" line
 *     per decoded frame + stats every 10 s.
 *   - BLE leg   ("ble24"): native BLE PHY RX on advertising channels
 *     37/38/39 (2402/2426/2480 MHz), rotating every
 *     ZEPHCORE_SNIFFER_BLE_DWELL_MS; "BLE JSON {...}" per PDU with the
 *     advertiser address + stats every 10 s.
 *   - hop leg   ("hop"):    alternate 868-LoRa / 433-OOK every
 *     CONFIG_ZEPHCORE_SNIFFER_HOP_S seconds (600 s default).
 *
 * This main NEVER calls startSendRaw / lora_send / any TX path — the
 * sniffer is receive-only by design (acceptance criterion: zero RF TX).
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_sniffer, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

/* Radio + adapter includes (same shared header the mesh roles use;
 * pulls LR2021Radio via CONFIG_ZEPHCORE_RADIO_LR2021). */
#include <mesh/RadioIncludes.h>
#include <NodePrefs.h>

/* Sniffer chip-level extension (lr20xx driver). */
extern "C" {
#include "lr20xx_lora.h"
}

/* On-chip rtl_433 OOK decode (vendored upstream, see src/rtl433/VENDOR.md). */
extern "C" {
#include "sniffer_rtl433.h"
}

/* wM-Bus block-1 header parse (native WM-BUS modem leg). */
extern "C" {
#include "sniffer_wmbus_parse.h"
}

#ifdef ZEPHCORE_LORA

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::ZephyrBoard zephyr_board;
static NodePrefs sniff_prefs;
static mesh::LR2021Radio lora_radio(lora_dev, zephyr_board, &sniff_prefs);

/* ── 433.92 MHz OOK probe preset ─────────────────────────────────────── */
#define SNIFFER_OOK_FREQ_HZ    433920000u
/* Demod bit clock: 10 kbps → 100 µs/bit.  Legacy 433 sensor PWM pulses
 * (300-1200 µs) become 3-12-bit runs — reconstructable on the host. */
#define SNIFFER_OOK_BR_BPS     10000u
/* RX BW ≤ ~5× bitrate (datasheet rule): 48 kHz code 0x1C. */
#define SNIFFER_OOK_RX_BW_CODE 0x1Cu
#define SNIFFER_OOK_PLD_LEN    240u

static bool leg_is(const char *name)
{
	return strcmp(CONFIG_ZEPHCORE_SNIFFER_LEG, name) == 0;
}

static void print_hex(const uint8_t *buf, uint16_t len)
{
	uint16_t n = (len > 24) ? 24 : len;

	for (uint16_t i = 0; i < n; i++) {
		printk("%02x", buf[i]);
	}
	if (len > n) {
		printk("...(+%u)", len - n);
	}
	printk("\n");
}

/* ── 868 LoRa leg — passive RX, per-packet log ───────────────────────── */

static void lora_rx_loop(void)
{
	uint8_t buf[64];
	int64_t last_pkt_ms = k_uptime_get();

	for (;;) {
		int len = lora_radio.recvRaw(buf, sizeof(buf));
		if (len > 0) {
			last_pkt_ms = k_uptime_get();
			int rssi = (int)lora_radio.getLastRSSI();
			/* SNR=0 is this Wio-LR2021's quirk, not a parse bug:
			 * lr_get_lora_pkt_status already has RadioLib parity
			 * (0.25 dB steps); the chip reports 0 here — same
			 * quirk family as the empty rssi_pkt field that
			 * lr_rssi_effective works around.  RSSI is real. */
			int snr = (int)lora_radio.getLastSNR();
			printk("LORA RSSI=%d SNR=%d len=%d hex=",
			       rssi, snr, len);
			print_hex(buf, (uint16_t)len);
		} else {
			int64_t silent_ms = k_uptime_get() - last_pkt_ms;
			if (silent_ms > 1500) {
				printk("LORA NO SIGNAL (tyla %lld s)\n",
				       (long long)(silent_ms / 1000));
				k_sleep(K_MSEC(1000));
			} else {
				k_sleep(K_MSEC(50));
			}
		}
	}
}

/* ── 433 OOK leg — poll-mode probe ───────────────────────────────────── */

static void ook_log_stats(void)
{
	uint16_t pkt_rx = 0, crc_err = 0, len_err = 0;
	int16_t rssi_inst = lr20xx_get_rssi_inst(lora_dev);

	if (lr20xx_sniffer_ook_stats(lora_dev, &pkt_rx, &crc_err,
				     &len_err) == 0) {
		printk("OOK stats rx=%u crc_err=%u len_err=%u rssi_inst=%d\n",
		       pkt_rx, crc_err, len_err, rssi_inst);
	}
}

static void ook_rx_loop(void)
{
	static uint8_t buf[SNIFFER_OOK_PLD_LEN];

	int ret = lr20xx_sniffer_ook_arm(lora_dev, SNIFFER_OOK_FREQ_HZ,
					 SNIFFER_OOK_BR_BPS,
					 SNIFFER_OOK_RX_BW_CODE,
					 SNIFFER_OOK_PLD_LEN);
	printk("OOK armed: 433.92 MHz br=%u bps ret=%d\n",
	       SNIFFER_OOK_BR_BPS, ret);
	if (ret != 0) {
		printk("OOK arm FAILED — probe loop exits\n");
		return;
	}

	int64_t last_stats_ms = k_uptime_get();
	for (;;) {
		k_sleep(K_MSEC(100));
		uint16_t len = 0;
		int16_t rssi = 0;
		ret = lr20xx_sniffer_ook_poll(lora_dev, buf, sizeof(buf),
					      &len, &rssi);
		if (ret != 0) {
			printk("OOK poll error %d\n", ret);
			k_sleep(K_SECONDS(1));
			continue;
		}
		if (len > 0) {
			printk("OOK RSSI=%d len=%u hex=", rssi, len);
			print_hex(buf, len);
			/* Feed the captured bitstream to the on-chip rtl_433
			 * decode path; it prints "OOK JSON ..." lines when a
			 * known sensor frame is recognized. */
			snf_rtl433_feed(buf, len);
		}
		if (k_uptime_get() - last_stats_ms >= 10000) {
			last_stats_ms = k_uptime_get();
			ook_log_stats();
		}
	}
}

/* ── wM-Bus leg — native WM-BUS modem probe (868.95 MHz T1/C1) ───────── */

#define SNIFFER_WMBUS_PLD_LEN  255u

static uint8_t wmbus_mode_code(void)
{
	const char *m = CONFIG_ZEPHCORE_SNIFFER_WMBUS_MODE;

	if (strcmp(m, "S") == 0) {
		return 0x0;
	}
	if (strcmp(m, "T2") == 0) {
		return 0x3;   /* meterTx direction = what a sniffer hears */
	}
	if (strcmp(m, "R2") == 0) {
		return 0x4;
	}
	if (strcmp(m, "C1") == 0) {
		return 0x5;
	}
	if (strcmp(m, "C2") == 0) {
		return 0x7;
	}
	if (strcmp(m, "N") == 0) {
		return 0x8;   /* N 4.8 kbps */
	}
	if (strcmp(m, "F2") == 0) {
		return 0xC;
	}
	return 0x1;           /* T1 (default) */
}

static uint8_t wmbus_format_code(uint8_t mode)
{
	/* STD_WM-BUS A = T1/C1/T2/C2/N, B = S/R2/F2.  On RX the chip
	 * auto-detects both formats by syncword (DS §12.2) — this only
	 * picks the standard the modem is configured for. */
	switch (mode) {
	case 0x0:  /* S  */
	case 0x4:  /* R2 */
	case 0xC:  /* F2 */
		return 0x1;
	default:
		return 0x0;
	}
}

static uint32_t wmbus_freq_for_mode(uint8_t mode)
{
	/* EN 13757-4 centre frequencies (TheClams cmd_wmbus.rs parity). */
	switch (mode) {
	case 0x0:  return 868300000u;   /* S          */
	case 0x4:  return 868030000u;   /* R2         */
	case 0xC:  return 433820000u;   /* F2         */
	case 0x6:  return 868525000u;   /* C2 meterRx */
	case 0x2:  return 868300000u;   /* T2 meterRx */
	case 0x8:                       /* N modes (169 MHz) */
	case 0x9:
	case 0xA:
	case 0xB:  return 169406250u;
	default:   return 868950000u;   /* T1 / C1 / T2 meterTx / C2 meterTx */
	}
}

static uint8_t crc_fail_count(uint32_t mask)
{
	uint8_t n = 0;

	while (mask) {
		n = (uint8_t)(n + (mask & 1u));
		mask >>= 1;
	}
	return n;
}

static void wmbus_log_stats(void)
{
	uint16_t pkt_rx = 0, crc_err = 0, len_err = 0;
	int16_t rssi_inst = lr20xx_get_rssi_inst(lora_dev);

	if (lr20xx_sniffer_wmbus_stats(lora_dev, &pkt_rx, &crc_err,
				       &len_err) == 0) {
		printk("WMBUS stats rx=%u crc_err=%u len_err=%u rssi_inst=%d\n",
		       pkt_rx, crc_err, len_err, rssi_inst);
	}
}

static void wmbus_rx_loop(void)
{
	static uint8_t buf[SNIFFER_WMBUS_PLD_LEN + 16];
	uint8_t mode = wmbus_mode_code();
	uint8_t fmt = wmbus_format_code(mode);
	uint32_t freq = wmbus_freq_for_mode(mode);

	int ret = lr20xx_sniffer_wmbus_arm(lora_dev, freq, mode, fmt,
					   SNIFFER_WMBUS_PLD_LEN);
	printk("WMBUS armed: %u.%03u MHz mode=%s (0x%X) fmt=%c ret=%d\n",
	       freq / 1000000u, (freq / 1000u) % 1000u,
	       CONFIG_ZEPHCORE_SNIFFER_WMBUS_MODE, mode,
	       fmt ? 'B' : 'A', ret);
	if (ret != 0) {
		printk("WMBUS arm FAILED — probe loop exits\n");
		return;
	}

	int64_t last_stats_ms = k_uptime_get();

	for (;;) {
		k_sleep(K_MSEC(100));

		uint16_t len = 0;
		struct lr20xx_sniffer_wmbus_status st;

		ret = lr20xx_sniffer_wmbus_poll(lora_dev, buf, sizeof(buf),
						&len, &st);
		if (ret != 0) {
			printk("WMBUS poll error %d\n", ret);
			k_sleep(K_SECONDS(1));
			continue;
		}

		if (len > 0) {
			struct snf_wmbus_meta meta;

			if (snf_wmbus_parse(buf, len, &meta) == 0) {
				printk("WMBUS JSON {\"mode\":\"%s\","
				       "\"fmt\":\"%c\",\"man\":\"%s\","
				       "\"serial\":\"%s\",\"ver\":%u,"
				       "\"type\":%u,\"ci\":%u,\"len\":%u,"
				       "\"rssi\":%d,\"lqi\":%u,\"crc\":%u,"
				       "\"crc_mask\":%u}\n",
				       CONFIG_ZEPHCORE_SNIFFER_WMBUS_MODE,
				       st.syncword_idx ? 'B' : 'A',
				       meta.man, meta.serial, meta.version,
				       meta.dev_type, meta.ci, meta.l_field,
				       (int)st.rssi_avg_dbm, st.lqi,
				       crc_fail_count(st.crc_err_mask),
				       (unsigned)st.crc_err_mask);
			} else {
				printk("WMBUS RAW len=%u hex=", len);
				print_hex(buf, len);
			}
		}

		if (k_uptime_get() - last_stats_ms >= 10000) {
			last_stats_ms = k_uptime_get();
			wmbus_log_stats();
		}
	}
}

/* ── BLE leg — advertising channel sniffer (37/38/39) ───────────────── */

#define SNIFFER_BLE_PLD_LEN 255u

struct sniffer_ble_chan {
	uint8_t ch;
	uint32_t freq;
	uint8_t whit;
};

/* Advertising channels + per-channel whitening inits (TheClams
 * lr2021-apps ble_txrx.rs, hardware-proven; fallback candidates if
 * adv_a decodes to garbage: 0x71 / 0x8F). */
static const struct sniffer_ble_chan ble_chans[3] = {
	{ 37, 2402000000u, 0x53u },
	{ 38, 2426000000u, 0x33u },
	{ 39, 2480000000u, 0x73u },
};

static void ble_log_stats(void)
{
	uint16_t pkt_rx = 0, crc_err = 0, len_err = 0;
	int16_t rssi_inst = lr20xx_get_rssi_inst(lora_dev);

	if (lr20xx_sniffer_ble_stats(lora_dev, &pkt_rx, &crc_err,
				     &len_err) == 0) {
		printk("BLE stats rx=%u crc_err=%u len_err=%u rssi_inst=%d\n",
		       pkt_rx, crc_err, len_err, rssi_inst);
	}
}

static void ble_rx_loop(void)
{
	static uint8_t buf[SNIFFER_BLE_PLD_LEN];
	uint8_t idx = 0;
	int64_t last_stats_ms = k_uptime_get();

	printk("BLE start: ch37/38/39, dwell %d ms\n",
	       CONFIG_ZEPHCORE_SNIFFER_BLE_DWELL_MS);

	for (;;) {
		const struct sniffer_ble_chan *c = &ble_chans[idx];
		int64_t dwell_end;

		int ret = lr20xx_sniffer_ble_arm(lora_dev, c->freq,
						 c->whit, 0 /* advertiser */);
		printk("BLE -> ch%u (%u MHz) ret=%d\n", c->ch,
		       c->freq / 1000000u, ret);
		if (ret != 0) {
			k_sleep(K_SECONDS(1));
			idx = (uint8_t)((idx + 1) % 3);
			continue;
		}

		dwell_end = k_uptime_get() +
			    (int64_t)CONFIG_ZEPHCORE_SNIFFER_BLE_DWELL_MS;

		while (k_uptime_get() < dwell_end) {
			uint16_t len = 0;
			struct lr20xx_sniffer_ble_status st;

			k_sleep(K_MSEC(50));
			ret = lr20xx_sniffer_ble_poll(lora_dev, buf,
						      sizeof(buf), &len, &st);
			if (ret != 0) {
				printk("BLE poll error %d\n", ret);
				k_sleep(K_MSEC(200));
				continue;
			}
			if (len > 0) {
				if (len >= 8) {
					/* PDU: [0]=header (type|flags),
					 * [1]=length, [2..7]=AdvA. */
					printk("BLE JSON {\"ch\":%u,"
					       "\"adv_a\":\"%02x:%02x:%02x:"
					       "%02x:%02x:%02x\","
					       "\"type\":%u,\"len\":%u,"
					       "\"rssi\":%d,\"lqi\":%u}\n",
					       c->ch, buf[2], buf[3], buf[4],
					       buf[5], buf[6], buf[7],
					       (unsigned)(buf[0] & 0x0F),
					       (unsigned)buf[1],
					       (int)st.rssi_avg_dbm, st.lqi);
				} else {
					printk("BLE RAW ch=%u len=%u hex=",
					       c->ch, len);
					print_hex(buf, len);
				}
			}

			if (k_uptime_get() - last_stats_ms >= 10000) {
				last_stats_ms = k_uptime_get();
				ble_log_stats();
			}
		}

		idx = (uint8_t)((idx + 1) % 3);
	}
}

/* ── hop leg — 600 s band TDM ────────────────────────────────────────── */

static uint8_t cr_enum_for(uint8_t cr_prefs)
{
	/* NodePrefs.cr is 5..8 (CR 4/5..4/8); the switch_band helper wants
	 * the Zephyr CR enum (CR_4_5=1 .. CR_4_8=4). */
	uint8_t cr = (cr_prefs >= 5 && cr_prefs <= 8) ? (cr_prefs - 4) : 1;

	return cr;
}

static void hop_to_ook(void)
{
	int ret = lr20xx_sniffer_ook_arm(lora_dev, SNIFFER_OOK_FREQ_HZ,
					 SNIFFER_OOK_BR_BPS,
					 SNIFFER_OOK_RX_BW_CODE,
					 SNIFFER_OOK_PLD_LEN);
	printk("HOP -> 433.92 OOK (ret=%d)\n", ret);
}

static void hop_to_lora(void)
{
	uint32_t freq_hz =
		(uint32_t)(sniff_prefs.freq * 1000000.0f + 0.5f);
	/* Bandwidth MUST use the adapter's exact kHz→enum mapping
	 * (radio_common.h bw_khz_to_enum) — the previous lossy helper
	 * (>=500→BW500, >=250→BW250, else BW125) collapsed the mesh's
	 * 62.5 kHz to BW_125_KHZ, and the post-hop LoRa leg listened on
	 * the wrong bandwidth → deaf to the whole mesh (observed live
	 * 2026-09-19: boot leg receives ~12 pkt/min, post-hop leg zero
	 * packets for 8+ min while the observer kept receiving on the
	 * same band). */
	enum lora_signal_bandwidth bw = bw_khz_to_enum((uint16_t)sniff_prefs.bw);
	int ret = lr20xx_switch_band(lora_dev, freq_hz, sniff_prefs.sf,
				     bw,
				     cr_enum_for(sniff_prefs.cr),
				     sniff_prefs.tx_power_dbm);
	/* Short settle after the OOK→LoRa return (AGC/PLL re-lock; the
	 * lean switch has no 3-bin FE cal by design). */
	k_msleep(50);
	printk("HOP -> %u.%03u LoRa SF%u BW%u (ret=%d)\n",
	       freq_hz / 1000000u, (freq_hz / 1000u) % 1000u,
	       sniff_prefs.sf, (unsigned)sniff_prefs.bw, ret);
}

static void hop_loop(void)
{
	bool ook_phase = false;
	int64_t deadline = k_uptime_get() +
			   (int64_t)CONFIG_ZEPHCORE_SNIFFER_HOP_S * 1000;
	int64_t last_stats_ms = k_uptime_get();
	uint32_t hops = 0;

	/* Boot on the 868 LoRa leg (begin() already armed RX there). */
	printk("HOP start: 868 LoRa, switch every %d s\n",
	       CONFIG_ZEPHCORE_SNIFFER_HOP_S);

	static uint8_t ook_buf[SNIFFER_OOK_PLD_LEN];

	for (;;) {
		if (ook_phase) {
			uint16_t len = 0;
			int16_t rssi = 0;
			int ret = lr20xx_sniffer_ook_poll(
				lora_dev, ook_buf, sizeof(ook_buf),
				&len, &rssi);
			if (ret == 0 && len > 0) {
				printk("OOK RSSI=%d len=%u hex=",
				       rssi, len);
				print_hex(ook_buf, len);
				/* Same on-chip rtl_433 decode feed as the
				 * ook433 leg. */
				snf_rtl433_feed(ook_buf, len);
			}
			if (k_uptime_get() - last_stats_ms >= 10000) {
				last_stats_ms = k_uptime_get();
				ook_log_stats();
			}
			k_sleep(K_MSEC(100));
		} else {
			uint8_t buf[64];
			int len = lora_radio.recvRaw(buf, sizeof(buf));
			if (len > 0) {
				int rssi = (int)lora_radio.getLastRSSI();
				int snr = (int)lora_radio.getLastSNR();
				printk("LORA RSSI=%d SNR=%d len=%d hex=",
				       rssi, snr, len);
				print_hex(buf, (uint16_t)len);
			}
			k_sleep(K_MSEC(50));
		}

		if (k_uptime_get() >= deadline) {
			hops++;
			if (ook_phase) {
				hop_to_lora();
			} else {
				hop_to_ook();
			}
			printk("HOP #%u done (t=%lld s)\n", hops,
			       (long long)(k_uptime_get() / 1000));
			ook_phase = !ook_phase;
			deadline = k_uptime_get() +
				   (int64_t)CONFIG_ZEPHCORE_SNIFFER_HOP_S *
					   1000;
			last_stats_ms = k_uptime_get();
		}
	}
}

int main(void)
{
	printk("=== ZephCore RF Sniffer (PASSIVE — no TX) leg=%s ===\n",
	       CONFIG_ZEPHCORE_SNIFFER_LEG);

	initNodePrefs(&sniff_prefs);

	snf_rtl433_init();

	if (!device_is_ready(lora_dev)) {
		printk("ERROR: LoRa device not ready\n");
		return -1;
	}

	lora_radio.setPrefs(&sniff_prefs);
	lora_radio.begin();
	lora_radio.setRxBoost(true);           /* max sensitivity */
	lora_radio.enableRxDutyCycle(false);   /* continuous RX */

	printk("radio up: %u Hz BW%u SF%u CR4/%u (RX only)\n",
	       lora_radio.getActiveFrequencyHz(),
	       lora_radio.getActiveBandwidthKHzX10() / 10u,
	       lora_radio.getActiveSpreadingFactor(),
	       lora_radio.getActiveCodingRate());

	if (leg_is("lora868")) {
		lora_rx_loop();
	} else if (leg_is("ook433")) {
		ook_rx_loop();
	} else if (leg_is("mbus868")) {
		wmbus_rx_loop();
	} else if (leg_is("ble24")) {
		ble_rx_loop();
	} else if (leg_is("hop")) {
		hop_loop();
	} else {
		printk("unknown leg '%s' — falling back to hop\n",
		       CONFIG_ZEPHCORE_SNIFFER_LEG);
		hop_loop();
	}
	return 0;
}

#else /* !ZEPHCORE_LORA */

int main(void)
{
	printk("RF Sniffer requires an LR2021 radio (lora0 alias)\n");
	return -1;
}

#endif /* ZEPHCORE_LORA */
