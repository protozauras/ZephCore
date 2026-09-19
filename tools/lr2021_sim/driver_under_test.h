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
/* Raw-response diagnostics (2026-09-20): raw_out (optional 8 bytes)
 * receives the exact [stat16][counters] bytes — the same view the
 * firmware logs as raw8= for the live counter-anomaly investigation. */
int lr_sniffer_ook_stats(uint16_t *pkt_rx, uint16_t *crc_error,
                         uint16_t *len_error, uint8_t *raw_out);

/* ── Sniffer WM-BUS poll-mode extension (F1, 2026-09-19) ─────────────
 * Sequence mirrors of lr20xx_sniffer_wmbus_arm/_poll/_stats in
 * lr20xx_lora.c (keep in lockstep — sandbox-first rule).
 * Opcodes: SetWmbusParams 0x026A, GetWmbusRxStats 0x026C,
 * GetWmbusPacketStatus 0x026D; packet type WM-BUS = 8 (spec enum).
 *
 * Negative control: compile the sandbox with
 * -DLR2021_SIM_OLD_WMBUS_SINGLE_READ (single-read poll, the pre-fix
 * shape) — test_sniffer_wmbus_poll_survives_irq_echo must FAIL on that
 * variant. */

#define LR20XX_PKT_TYPE_WMBUS  0x08   /* spec SetPacketType enum: WM-BUS=8 */

/* WM-BUS packet status (mirror of lr20xx_sniffer_wmbus_status). */
struct lr_sniffer_wmbus_status {
    uint8_t  l_field;        /* demodulated L-field */
    uint16_t pkt_len;        /* status-reported length (read fallback) */
    int16_t  rssi_avg_dbm;   /* -rssi_avg/2 dBm (rounded up) */
    int16_t  rssi_sync_dbm;  /* -rssi_sync/2 dBm (rounded up) */
    uint32_t crc_err_mask;   /* 17-bit per-CRC failure bitmap */
    uint8_t  syncword_idx;   /* 0 = format A, 1 = format B */
    uint8_t  lqi;            /* 0.25 dB steps */
};

/* pbl_len_tx per WM-BUS mode (mirror of lr_wmbus_pbl_len_tx_for_mode). */
uint16_t lr_wmbus_pbl_len_tx(uint8_t mode);

/* SetWmbusParams payload builder (mirror; DS Table 12-2 byte order:
 * [mode][rx_bw=0xFF auto][pkt_format][addr_comp=0][pld_len]
 * [pbl hi][pbl lo][pbl_len_detect=0xFF auto]). */
void lr_sniffer_wmbus_build_params(uint8_t mode, uint8_t pkt_format,
                                   uint8_t pld_len, uint8_t out[8]);

int lr_sniffer_wmbus_arm(uint32_t freq_hz, uint8_t mode, uint8_t pkt_format,
                         uint8_t pld_len);
int lr_sniffer_wmbus_poll(uint8_t *buf, uint16_t cap, uint16_t *out_len,
                          struct lr_sniffer_wmbus_status *st);
int lr_sniffer_wmbus_stats(uint16_t *pkt_rx, uint16_t *crc_error,
                           uint16_t *len_error, uint8_t *raw_out);

/* ── Sniffer BLE poll-mode extension (F2, 2026-09-19) ────────────────
 * Sequence mirrors of lr20xx_sniffer_ble_arm/_poll/_stats in
 * lr20xx_lora.c (keep in lockstep — sandbox-first rule).
 * Opcodes: SetBleModulationParams 0x0260, SetBleChannelParams 0x0261,
 * GetBleRxStats 0x0264, GetBlePacketStatus 0x0265; packet type BLE = 3.
 *
 * Negative control: compile the sandbox with
 * -DLR2021_SIM_OLD_BLE_SINGLE_READ (single-read poll, the pre-fix
 * shape) — test_sniffer_ble_poll_survives_irq_echo must FAIL on that
 * variant. */

#define LR20XX_PKT_TYPE_BLE    0x03   /* spec SetPacketType enum: BLE=3 */
#define LR20XX_BLE_ADV_CRC_INIT 0x555555u
#define LR20XX_BLE_ADV_SYNCWORD 0x8E89BED6u

/* BLE packet status (mirror of lr20xx_sniffer_ble_status). */
struct lr_sniffer_ble_status {
    uint16_t pkt_len;        /* status-reported length (read fallback) */
    int16_t  rssi_avg_dbm;   /* -rssi_avg/2 dBm (rounded up) */
    int16_t  rssi_sync_dbm;  /* -rssi_sync/2 dBm (rounded up) */
    uint8_t  lqi;            /* 0.25 dB steps */
};

/* SetBleChannelParams payload builder (mirror; DS Table 14-2 byte
 * order: [crc_in_fifo(bit4)|channel_type][whit_init][crc_init 3B BE]
 * [syncword 4B BE]). */
void lr_sniffer_ble_build_channel_params(uint8_t whit_init,
                                         uint8_t channel_type,
                                         uint8_t out[9]);

int lr_sniffer_ble_arm(uint32_t freq_hz, uint8_t whit_init,
                       uint8_t channel_type);
int lr_sniffer_ble_poll(uint8_t *buf, uint16_t cap, uint16_t *out_len,
                        struct lr_sniffer_ble_status *st);
int lr_sniffer_ble_stats(uint16_t *pkt_rx, uint16_t *crc_error,
                         uint16_t *len_error, uint8_t *raw_out);

/* ── Multi-leg helpers (F3, 2026-09-19) ──────────────────────────────
 * lr_sniffer_switch_band_cfg mirrors lr20xx_switch_band with dynamic
 * SF / BW-kHz / CR; lr_sniffer_cad_probe mirrors lr20xx_cad_probe
 * (1 = activity, 0 = silent, <0 = error/-ETIMEDOUT).  The multi
 * scheduler itself lives in main_sniffer.cpp (not sandbox-compiled) —
 * these mirrors pin the per-leg primitives to the driver sequences. */

int lr_sniffer_switch_band_cfg(uint32_t freq_hz, uint8_t sf,
                               uint16_t bw_khz, uint8_t cr_code);
int lr_sniffer_cad_probe(uint32_t freq_hz, uint8_t sf, uint16_t bw_khz,
                         int8_t peak_offset);

/* Packing helpers (mirror lr_bw_to_code / lr_ldro_for) — exposed for
 * the payload pins in the multi-cycle tests. */
uint8_t lr_bw_code_dut(uint16_t bw_khz);
uint8_t lr_ldro_dut(uint8_t sf, uint16_t bw_khz);

/* Test hook: reset the frequency tracker to the 869.618 MHz boot band
 * (state persists across tests otherwise). */
void lr_sniffer_reset_freq_tracker(void);

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
