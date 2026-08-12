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

#ifdef __cplusplus
}
#endif

#endif /* LR20XX_LORA_H */
