/*
 * SPDX-License-Identifier: MIT
 *
 * driver_under_test.h — API of the lr_cmd() wrapper layer under test.
 * Matches signatures in zephcore/patches/zephyr-new/drivers/lora/lr20xx/lr20xx_lora.c
 * (function names are the same, types reduced to plain C).
 */
#ifndef DRIVER_UNDER_TEST_H
#define DRIVER_UNDER_TEST_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Mirrors Zephyr's struct spi_buf so we can use the same calling convention
 * as the real driver without pulling in <zephyr/kernel.h>. */
struct spi_buf { void *buf; size_t len; };
struct spi_buf_set { const struct spi_buf *buffers; size_t count; };

/* The single function the test harness exercises in isolation. */
int lr_cmd(uint16_t opcode, const uint8_t *params, size_t param_len,
           uint8_t *resp, size_t resp_len);

/* Wrappers (subset used by dio_work_handler / start_rx on real driver). */
int lr_get_version(uint8_t *major, uint8_t *minor);
int lr_clear_irq(uint32_t mask);
int lr_get_and_clear_irq(uint32_t *irq);
int lr_get_irq_status(uint32_t *irq);
int lr_set_standby(uint8_t mode);
int lr_set_rx(uint32_t timeout_rtc);
int lr_set_tx(uint32_t timeout_rtc);
int lr_set_dio_function(uint8_t dio, uint8_t func, uint8_t drive);
int lr_set_dio_irq_cfg(uint8_t dio, uint32_t mask);
int lr_get_rx_packet_length(uint16_t *pkt_len);
int lr_get_lora_packet_status(uint8_t *st_len, int16_t *rssi,
                              int16_t *rssi_signal, int8_t *snr);
int lr_fifo_read(uint8_t *data, size_t len);
int lr_clear_rx_fifo(void);

/* start_rx tail: re-poll DIO8 after the IRQ clear; re-submit the DIO
 * work item if the line is still HIGH (edge-race port). */
int lr20xx_start_rx_edge_recheck(void);

/* RX read flows under comparison (see test_meshcore_flow_burst):
 * lr_rx_flow_current = the driver as it is on HEAD (length authority =
 * GetLoRaPacketStatus); lr_rx_flow_meshcore = MeshCore/RadioLib parity
 * (length authority = GetRxPktLength remaining-FIFO total). */
int lr_rx_flow_current(uint8_t *data, size_t maxlen, size_t *out_len);
int lr_rx_flow_meshcore(uint8_t *data, size_t maxlen, size_t *out_len);

/* ── Dual-band helpers (L1-U1, 2026-08-02) ───────────────────────────
 * Mirrors of lr20xx_lora.c constants + pure helpers (band split /
 * HF PA duty / RX path selection). Keep in lockstep with the driver
 * (see tools/lr2021_sim README + zephcore-firmware pitfall #21). */

#define LR20XX_LF_CUTOFF_HZ        1500000000u
#define LR20XX_RX_PATH_LF          0x00
#define LR20XX_RX_PATH_HF          0x01
#define LR20XX_RX_BOOST_NONE       0x00
#define LR20XX_RX_BOOST_LF         0x01
#define LR20XX_RX_BOOST_HF         0x04
#define LR20XX_PA_SEL_LF           0x00
#define LR20XX_PA_SEL_HF           0x01
#define LR20XX_PA_LF_DUTY_UNUSED   0x06
#define LR20XX_PA_LF_SLICES_UNUSED 0x07
#define LR20XX_PA_HF_DUTY_UNUSED   16

/* RadioLib LR2021 band split: above 1500 MHz → HF (2.4 GHz ISM / S-band). */
bool lr_is_hf(uint32_t freq_hz);
/* HF chip max TX power clamp (+12 dBm, RadioLib checkOutputPower HF). */
int8_t lr_clamp_hf_power(int8_t power_dbm);
/* HF PA hf_duty field = paOptTableHf duty + PA_HF_DUTY_UNUSED(16). */
uint8_t lr_pa_hf_duty_for_power(int8_t power_dbm);
/* Sub-GHz vs 2.4 GHz RX path + boost for the given frequency. */
void lr_rx_path_for_freq(uint32_t freq_hz, bool boost,
                         uint8_t *path, uint8_t *boost_val);
/* RSSI source selection (lr20xx_lora.c mirror, §9.9.9): fall back to the
 * despread signal RSSI when the packet-RSSI field is empty (0). */
int16_t lr_rssi_effective(int16_t rssi, int16_t rssi_signal);

/* ── TDM band-switch pure helpers (L3-U4, 2026-08-02) ───────────────
 * Mirrors of lr20xx_lora.c static functions.  Keep in lockstep. */

/* Single FE-calibration bin: nearest 4 MHz, | 0x8000 when HF (>1500 MHz).
 * RadioLib: (uint16_t)((freq/4.0f)+0.5f) then |= HF-path marker. */
uint16_t lr_cal_fe_single_bin_hz(uint32_t freq_hz);
/* Image-cal trigger: |old-new| >= 20 MHz (RadioLib CAL_IMG_FREQ_TRIG_MHZ). */
bool lr_band_switch_needs_cal(uint32_t old_hz, uint32_t new_hz);

/* ── Sniffer OOK poll-mode extension (2026-09-19) ────────────────────
 * Sequence mirrors of lr20xx_sniffer_ook_arm / _ook_poll in
 * lr20xx_lora.c (keep in lockstep — sandbox-first rule). */

#define LR20XX_PKT_TYPE_LORA    0x00   /* spec SetPacketType enum: LoRa=0 */
#define LR20XX_PKT_TYPE_OOK    0x0A   /* spec SetPacketType enum: OOK=10 */
#define LR20XX_RX_TIMEOUT_INF  0xFFFFFFu

int lr_set_rf_frequency(uint32_t freq_hz);
int lr_set_pkt_type(uint8_t pkt_type);

int lr_sniffer_ook_arm(uint32_t freq_hz, uint32_t br_bps,
                       uint8_t rx_bw_code, uint16_t pld_len);
int lr_sniffer_ook_poll(uint8_t *buf, uint16_t cap, uint16_t *out_len,
                        int16_t *rssi_avg_dbm);
int lr_sniffer_ook_stats(uint16_t *pkt_rx, uint16_t *crc_error,
                         uint16_t *len_error);

/* Mirror of lr_get_rssi_inst (lr20xx_lora.c): 9-bit raw, -raw/2 dBm. */
int lr_get_rssi_inst(int16_t *rssi);

/* Mirror of lr20xx_switch_band() (lr20xx_lora.c) — the OOK→LoRa hop-back
 * sequence.  Fixed values match the sniffer's 868 leg (SF8, BW62.5 code,
 * CR4/8 code, 8-symbol preamble, private syncword, CRC on, IQ standard,
 * rx-boosted).  THE regression point: after sniffer_ook_arm() left the
 * chip in the OOK modem, this sequence MUST re-issue SetPacketType(LoRa)
 * (compile with -DLR2021_SIM_OLD_SWITCH_BAND for the pre-fix variant
 * that skips the restore — the pkt-type test must fail on it). */
int lr_sniffer_switch_band_lora(uint32_t freq_hz);

#endif /* DRIVER_UNDER_TEST_H */
