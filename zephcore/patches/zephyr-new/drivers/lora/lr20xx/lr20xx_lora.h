/*
 * SPDX-License-Identifier: MIT
 * LR20xx Zephyr LoRa driver — extension API
 *
 * Functions extending the standard Zephyr lora_driver_api with
 * LR20xx-specific features (RX boost, RSSI readout, preamble detection,
 * duty cycle, AGC reset).
 */

#ifndef LR20XX_LORA_H
#define LR20XX_LORA_H

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get instantaneous RSSI (for noise floor calibration)
 *
 * @param dev LoRa device
 * @return RSSI in dBm, or -128 on error
 */
int16_t lr20xx_get_rssi_inst(const struct device *dev);

/**
 * @brief Check if radio is actively receiving a packet
 *
 * Checks IRQ status for preamble/sync word detection.
 * Uses non-blocking mutex — returns false if SPI is busy.
 *
 * @param dev LoRa device
 * @return true if preamble or sync word detected
 */
bool lr20xx_is_receiving(const struct device *dev);

/**
 * @brief Enable/disable RX boosted mode
 *
 * @param dev LoRa device
 * @param enable true to enable boost
 */
void lr20xx_set_rx_boost(const struct device *dev, bool enable);

/**
 * @brief Get a random number from the radio hardware RNG
 *
 * @param dev LoRa device
 * @return Random 32-bit value
 */
uint32_t lr20xx_get_random(const struct device *dev);

/**
 * @brief Reset AGC by performing warm sleep + full recalibration
 *
 * @param dev LoRa device
 */
void lr20xx_reset_agc(const struct device *dev);

/**
 * @brief Set the adaptive-CAD operating detPeak offset
 *
 * Signed delta applied to the per-SF base cadDetPeak on every LBT CAD.
 * Takes effect on the next CAD — no reconfigure needed.  Clamped
 * in-driver to the LR20xx scale (48-90).
 *
 * @param dev    LoRa device
 * @param offset Signed offset from the base table value
 */
void lr20xx_cad_set_peak_offset(const struct device *dev, int8_t offset);

/**
 * @brief Per-SF base cadDetPeak for the currently configured SF
 *
 * @param dev LoRa device
 * @return Base detPeak (56-68 on this family)
 */
uint8_t lr20xx_cad_base_peak(const struct device *dev);

/**
 * @brief Run one blocking calibration CAD at base detPeak + peak_offset
 *
 * Uses the operating modem config (SF/BW/symbol count).  Leaves the chip
 * in STANDBY — the caller must restart RX afterwards.  Mesh loop thread
 * only.
 *
 * @param dev         LoRa device
 * @param peak_offset Signed offset from the base table value
 * @return 1 = activity detected, 0 = channel free, <0 = error
 */
int lr20xx_cad_probe(const struct device *dev, int8_t peak_offset);

/**
 * @brief Lean frequency/band switch for TDM dual-band operation
 *
 * Applies the per-band sequence RadioLib uses on setFrequency/setOutputPower
 * (single-bin image cal only when |Δf| >= 20 MHz, then frequency, modem
 * params, syncword, RX path, PA config) and re-arms RX.  Does NOT run the
 * 3-bin FE calibration + 50 ms sleep — that would reintroduce the deaf
 * window the re-arm fix removed (LR2021_RADIO_STATUS.md §1 fix A).
 * Typical switch cost ~1-2 ms.
 *
 * The cached modem_cfg.frequency doubles as the current-frequency tracker.
 * The radio is left listening on the new band (RX re-armed).
 *
 * @param dev     LoRa device
 * @param freq_hz New frequency in Hz (e.g. 2450000000 for 2.4 GHz ISM)
 * @param sf      Spreading factor (5-12)
 * @param bw      Bandwidth enum (e.g. BW_500_KHZ / BW_62_KHZ)
 * @param cr      Coding rate (4/5) — as CR_4_5 enum value
 * @param tx_power TX power in dBm (clamped to +12 dBm on HF in-driver)
 *
 * @retval 0      band switched, radio left listening on the new band
 * @retval -EBUSY TX in flight — band unchanged (the 01b3df9 guard)
 * @retval -EIO   device not configured yet
 */
int lr20xx_switch_band(const struct device *dev, uint32_t freq_hz,
		       uint8_t sf, enum lora_signal_bandwidth bw,
		       uint8_t cr, int8_t tx_power);

/* ── Passive RF-sniffer extension (OOK probe, poll-mode RX) ────────────
 *
 * The sniffer role never transmits.  These functions arm the chip for
 * OOK packet-mode RX (LR2021 has NO direct mode — DIO pins carry IRQs
 * only, DS.LR2021 §5.1.1) and poll for completed packets over SPI, so
 * the whole OOK leg runs with the DIO IRQ machinery silenced.
 *
 * Command set per DS.LR2021 Rev 1.1 §20 / LR20xx Rev 2.1 §16
 * (opcodes cross-checked against TheClams/lr2021 spec/commands.yaml:
 * SetOokModulationParams 0x0281, SetOokPacketParams 0x0282,
 * SetOokSyncWord 0x0284, GetOokRxStats 0x0286,
 * GetOokPacketStatus 0x0287, SetOokDetector 0x0288, packet type OOK=10).
 */

/**
 * @brief Arm OOK packet-mode RX at the given frequency (poll mode)
 *
 * Lean switch sequence (standby → IRQ/FIFO clear → image cal when the
 * band moved ≥ 20 MHz → freq → pkt type OOK → OOK mod/packet params →
 * sync/detector → RX path → DIO silenced → RX continuous).  The chip is
 * left listening; packets are retrieved with lr20xx_sniffer_ook_poll().
 *
 * @param dev        LoRa device
 * @param freq_hz    Center frequency (e.g. 433920000)
 * @param br_bps     OOK bitrate in bps (demod bit clock — bitstream capture
 *                   of PWM envelopes wants br ≥ ~10× the pulse rate)
 * @param rx_bw_code FSK RX-bandwidth code (lr20xx_radio_fsk_common_types.h;
 *                   keep ≤ ~5× bitrate per datasheet)
 * @param pld_len    Fixed payload length to accept per packet (bytes)
 *
 * @retval 0 armed, listening
 * @retval -EBUSY TX in flight (never on the sniffer, guard parity)
 * @retval -EIO   device not configured yet
 */
int lr20xx_sniffer_ook_arm(const struct device *dev, uint32_t freq_hz,
			   uint32_t br_bps, uint8_t rx_bw_code,
			   uint16_t pld_len);

/**
 * @brief Poll for one completed OOK packet (non-blocking, one SPI pass)
 *
 * Reads + clears the IRQ; on RX_DONE reads GetRxPacketLength, pulls the
 * FIFO, fetches GetOokPacketStatus RSSI, clears the FIFO and re-arms RX
 * (DIO stays silenced — poll mode owns the chip while armed).
 *
 * @param dev          LoRa device
 * @param buf          Output buffer
 * @param cap          Buffer capacity (bytes)
 * @param out_len      Set to the packet length (0 = nothing this poll)
 * @param rssi_avg_dbm Set to packet avg RSSI in dBm when a packet was read
 *
 * @retval 0 polled (check out_len), <0 on error
 */
int lr20xx_sniffer_ook_poll(const struct device *dev, uint8_t *buf,
			    uint16_t cap, uint16_t *out_len,
			    int16_t *rssi_avg_dbm);

/**
 * @brief Read the chip's OOK RX statistics (GetOokRxStats)
 *
 * RadioLib getOokRxStats parity: the response carries exactly three
 * u16 counters — pkt_rx, crc_error, len_error (DS Table 16-7).  There
 * are no preamble/sync counters in this command.
 *
 * @param dev       LoRa device
 * @param pkt_rx    Total received packets
 * @param crc_error Packets with CRC error
 * @param len_error Packets with length error
 * @param raw_out   Optional 8-byte buffer receiving the RAW response
 *                  [stat16][counters] for live counter diagnostics;
 *                  pass NULL to skip
 *
 * @retval 0 on success, <0 on error
 */
int lr20xx_sniffer_ook_stats(const struct device *dev, uint16_t *pkt_rx,
			     uint16_t *crc_error, uint16_t *len_error,
			     uint8_t *raw_out);

/* ── Passive RF-sniffer extension (WM-BUS probe, poll-mode RX) ────────
 *
 * Native WM-BUS modem leg (DS.LR20xx §12; encodings cross-checked
 * against TheClams/lr2021 cmd_wmbus.rs + spec/commands.yaml):
 * SetWmbusParams 0x026A, GetWmbusRxStats 0x026C, GetWmbusPacketStatus
 * 0x026D, packet type WM-BUS = 8.  The chip decodes the EN13757-4 PHY
 * (3-of-6 / Manchester, whitening, per-block CRCs) and auto-detects the
 * A/B frame format by syncword on RX — no syncword command needed.
 * One mode per configuration (T1/C1 = one-way meters @ 868.95 MHz,
 * S = 868.3, R2 = 868.03, F2 = 433.82, N = 169 MHz).
 */

/**
 * @brief WM-BUS packet-status view (GetWmbusPacketStatus, DS Table 12-6)
 */
struct lr20xx_sniffer_wmbus_status {
	uint8_t l_field;        /* demodulated L-field (may differ from FIFO
				 * total in mode A) */
	uint16_t pkt_len;       /* status-reported length (read fallback) */
	int16_t rssi_avg_dbm;   /* -rssi_avg/2 dBm (rounded up) */
	int16_t rssi_sync_dbm;  /* -rssi_sync/2 dBm (rounded up) */
	uint32_t crc_err_mask;  /* 17-bit per-CRC failure bitmap; bit0 =
				 * header CRC in format A */
	uint8_t syncword_idx;   /* 0 = format A received, 1 = format B */
	uint8_t lqi;            /* link quality indicator, 0.25 dB steps */
};

/**
 * @brief Arm WM-BUS packet-mode RX at the given frequency (poll mode)
 *
 * Lean switch sequence (standby → IRQ/FIFO clear → image cal when the
 * band moved ≥ 20 MHz → freq → pkt type WM-BUS → SetWmbusParams →
 * RX path → DIO silenced → RX continuous).  The chip is left listening;
 * frames are retrieved with lr20xx_sniffer_wmbus_poll().
 *
 * @param dev        LoRa device
 * @param freq_hz    Center frequency (e.g. 868950000 for T1/C1)
 * @param mode       WM-BUS mode (DS §12.3.1): 0x0 S, 0x1 T1, 0x2 T2
 *                   meterRx, 0x3 T2 meterTx, 0x4 R2, 0x5 C1, 0x6 C2
 *                   meterRx, 0x7 C2 meterTx, 0x8..0xB N, 0xC F2
 * @param pkt_format 0 = STD_WM-BUS A, 1 = STD_WM-BUS B (RX auto-detects
 *                   both once a standard is set)
 * @param pld_len    Max accepted L-field size (payload incl. C/M/A/CI)
 *
 * @retval 0 armed, listening
 * @retval -EBUSY TX in flight (never on the sniffer, guard parity)
 * @retval -EIO   device not configured yet
 */
int lr20xx_sniffer_wmbus_arm(const struct device *dev, uint32_t freq_hz,
			     uint8_t mode, uint8_t pkt_format,
			     uint8_t pld_len);

/**
 * @brief Poll for one completed WM-BUS frame (non-blocking, one SPI pass)
 *
 * Reads + clears the IRQ; on RX_DONE runs the 3-read dance (echo victim
 * → GetWmbusPacketStatus real → GetRxPacketLength authority), pulls the
 * FIFO (max `cap` bytes), clears the FIFO and re-arms RX (DIO stays
 * silenced — poll mode owns the chip while armed).
 *
 * @param dev     LoRa device
 * @param buf     Output buffer
 * @param cap     Buffer capacity (bytes)
 * @param out_len Set to the frame length (0 = nothing this poll)
 * @param st      Optional packet status (L-field, RSSIs, CRC mask,
 *                format index, LQI); zeroed each poll
 *
 * @retval 0 polled (check out_len), <0 on error
 */
int lr20xx_sniffer_wmbus_poll(const struct device *dev, uint8_t *buf,
			      uint16_t cap, uint16_t *out_len,
			      struct lr20xx_sniffer_wmbus_status *st);

/**
 * @brief Read the chip's WM-BUS RX statistics (GetWmbusRxStats)
 *
 * DS Table 12-4 parity: the response carries exactly three u16 counters
 * — pkt_rx, pkt_crc_error, LenError.
 *
 * @param dev       LoRa device
 * @param pkt_rx    Total received packets
 * @param crc_error Packets with a CRC error
 * @param len_error Packets with a length error
 * @param raw_out   Optional 8-byte buffer receiving the RAW response
 *                  [stat16][counters] for live counter diagnostics;
 *                  pass NULL to skip
 *
 * @retval 0 on success, <0 on error
 */
int lr20xx_sniffer_wmbus_stats(const struct device *dev, uint16_t *pkt_rx,
			       uint16_t *crc_error, uint16_t *len_error,
			       uint8_t *raw_out);

/* ── Passive RF-sniffer extension (BLE advertising probe, poll-mode) ──
 *
 * Native BLE PHY (DS.LR20xx §14; TheClams/lr2021 spec parity): the chip
 * dewhitens and CRC-checks itself — only the per-channel whitening init
 * and the advertising access code have to be programmed.  Advertising
 * channels 37/38/39 = 2402/2426/2480 MHz; whitening inits 0x53/0x33/
 * 0x73 (lr2021-apps ble_txrx.rs, hardware-proven), CRC init 0x555555,
 * access code 0x8E89BED6.  LE 1M first (2M/coded need extra register
 * patches).  One channel per arm (the app rotates 37/38/39 per dwell).
 * Opcodes: SetBleModulationParams 0x0260, SetBleChannelParams 0x0261,
 * GetBleRxStats 0x0264, GetBlePacketStatus 0x0265; packet type BLE = 3.
 */

/**
 * @brief BLE packet-status view (GetBlePacketStatus, DS Table 14-8)
 */
struct lr20xx_sniffer_ble_status {
	uint16_t pkt_len;       /* status-reported length (read fallback) */
	int16_t rssi_avg_dbm;   /* -rssi_avg/2 dBm (rounded up) */
	int16_t rssi_sync_dbm;  /* -rssi_sync/2 dBm (rounded up) */
	uint8_t lqi;            /* link quality indicator, 0.25 dB steps */
};

/**
 * @brief Arm BLE packet-mode RX at the given frequency (poll mode)
 *
 * Lean switch sequence (standby → IRQ/FIFO clear → image cal when the
 * band moved ≥ 20 MHz → freq → pkt type BLE → SetBleModulationParams
 * (LE 1M) → SetBleChannelParams → RX path → DIO silenced → RX
 * continuous).  The chip is left listening; PDUs are retrieved with
 * lr20xx_sniffer_ble_poll().
 *
 * @param dev          LoRa device
 * @param freq_hz      Advertising channel centre (2402/2426/2480 MHz)
 * @param whit_init    Per-channel whitening init (0x53 / 0x33 / 0x73)
 * @param channel_type 0 = advertiser, 1/2 = data channels
 *
 * @retval 0 armed, listening
 * @retval -EBUSY TX in flight (never on the sniffer, guard parity)
 * @retval -EIO   device not configured yet
 */
int lr20xx_sniffer_ble_arm(const struct device *dev, uint32_t freq_hz,
			   uint8_t whit_init, uint8_t channel_type);

/**
 * @brief Poll for one completed BLE PDU (non-blocking, one SPI pass)
 *
 * Reads + clears the IRQ; on RX_DONE runs the 3-read dance (echo victim
 * → GetBlePacketStatus real → GetRxPacketLength authority), pulls the
 * FIFO (max `cap` bytes), clears the FIFO and re-arms RX (DIO stays
 * silenced — poll mode owns the chip while armed).
 *
 * @param dev     LoRa device
 * @param buf     Output buffer
 * @param cap     Buffer capacity (bytes)
 * @param out_len Set to the PDU length (0 = nothing this poll)
 * @param st      Optional packet status (RSSIs, LQI); zeroed each poll
 *
 * @retval 0 polled (check out_len), <0 on error
 */
int lr20xx_sniffer_ble_poll(const struct device *dev, uint8_t *buf,
			    uint16_t cap, uint16_t *out_len,
			    struct lr20xx_sniffer_ble_status *st);

/**
 * @brief Read the chip's BLE RX statistics (GetBleRxStats)
 *
 * DS Table 14-5/14-6 parity: three u16 counters — pkt_rx, crc_error,
 * len_error.
 *
 * @param dev       LoRa device
 * @param pkt_rx    Total received packets
 * @param crc_error Packets with a CRC error
 * @param len_error Packets with a length error
 * @param raw_out   Optional 8-byte buffer receiving the RAW response
 *                  [stat16][counters] for live counter diagnostics;
 *                  pass NULL to skip
 *
 * @retval 0 on success, <0 on error
 */
int lr20xx_sniffer_ble_stats(const struct device *dev, uint16_t *pkt_rx,
			     uint16_t *crc_error, uint16_t *len_error,
			     uint8_t *raw_out);

#ifdef __cplusplus
}
#endif

#endif /* LR20XX_LORA_H */
