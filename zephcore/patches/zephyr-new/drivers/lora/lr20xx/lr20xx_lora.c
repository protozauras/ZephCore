/*
 * SPDX-License-Identifier: MIT
 * LR20xx Zephyr LoRa driver — direct-SPI implementation for the Semtech LR2021
 *
 * Wire behavior ported from RadioLib's LR2021 module (verified working on the
 * Wio-LR2021 + XIAO nRF54L15 by the Meshtastic nRF54L15 port, Berlin mesh) and
 * cross-checked against the Semtech lr20xx_driver SDK opcode/param tables and
 * the LR2021 datasheet command spec (TheClams/lr2021 spec/commands.yaml).
 *
 * Differences from the old SDK-wrapper driver (which never entered RX):
 *   - 2-byte opcodes, raw two-phase SPI (CS toggle per phase), NO Semtech SDK
 *   - strict BUSY wait before EVERY SPI transaction
 *   - packed modulation/packet params exactly as RadioLib + datasheet:
 *       SetLoRaModulationParams: [sf<<4|bw, cr<<4|ldro]
 *       SetLoRaPacketParams:     [pre_hi, pre_lo, pld_len, hdr<<2|crc<<1|iq]
 *       SetPaConfig:             [sel<<7|mode, duty<<4|slices, hf_duty]
 *   - TX power in half-dBm (power*2), PA duty/slices per RadioLib LF table
 *   - init sequence: reset -> version -> standby -> TCXO -> reg mode ->
 *     fallback -> clear IRQ -> DIO function (IRQ) -> calibrate(0x6F) ->
 *     wait BUSY -> packet type LoRa   (RadioLib modSetup/config order)
 *   - NO DIO5/DIO6 RF-switch config (on-module switch, GPIO regulators)
 *   - NO LBD/regmem writes (VBAT sense reads ~3 mV on this board — RadioLib
 *     never touches that path and the radio works)
 *   - FE calibration paired with set_rf_frequency (round-to-nearest 4 MHz bin)
 */

#define DT_DRV_COMPAT semtech_lr2021

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <math.h>

#include "lr20xx_lora.h"

LOG_MODULE_REGISTER(lr20xx_lora, CONFIG_LORA_LOG_LEVEL);

/* ── LR2021 2-byte opcodes (verified: SDK == RadioLib == datasheet spec) ── */

#define LR20XX_OP_READ_RX_FIFO           0x0001
#define LR20XX_OP_WRITE_TX_FIFO          0x0002
#define LR20XX_OP_GET_STATUS             0x0100
#define LR20XX_OP_GET_VERSION            0x0101
#define LR20XX_OP_GET_ERRORS             0x0110
#define LR20XX_OP_CLEAR_ERRORS           0x0111
#define LR20XX_OP_SET_DIO_FUNC           0x0112
#define LR20XX_OP_SET_DIO_IRQ_CFG        0x0115
#define LR20XX_OP_CLEAR_IRQ              0x0116
#define LR20XX_OP_GET_AND_CLEAR_IRQ      0x0117
#define LR20XX_OP_SET_TCXO_MODE          0x0120
#define LR20XX_OP_SET_REG_MODE           0x0121
#define LR20XX_OP_CALIBRATE              0x0122
#define LR20XX_OP_CALIBRATE_FRONT_END    0x0123
#define LR20XX_OP_GET_VBAT               0x0124
#define LR20XX_OP_GET_RANDOM_NUMBER      0x0126
#define LR20XX_OP_SET_SLEEP_MODE         0x0127
#define LR20XX_OP_SET_STANDBY            0x0128
#define LR20XX_OP_CLEAR_RX_FIFO          0x011E
#define LR20XX_OP_SET_RF_FREQUENCY       0x0200
#define LR20XX_OP_SET_RX_PATH            0x0201
#define LR20XX_OP_SET_PA_CFG             0x0202
#define LR20XX_OP_SET_TX_PARAMS          0x0203
#define LR20XX_OP_SET_RX_TX_FALLBACK     0x0206
#define LR20XX_OP_SET_PKT_TYPE           0x0207
#define LR20XX_OP_GET_RSSI_INST          0x020B
#define LR20XX_OP_SET_RX                 0x020C
#define LR20XX_OP_SET_TX                 0x020D
#define LR20XX_OP_SET_RX_DUTY_CYCLE      0x0210
#define LR20XX_OP_GET_RX_PACKET_LENGTH   0x0212
#define LR20XX_OP_SET_LORA_MOD_PARAMS    0x0220
#define LR20XX_OP_SET_LORA_PKT_PARAMS    0x0221
#define LR20XX_OP_SET_LORA_SYNCWORD      0x0223
#define LR20XX_OP_SET_LORA_CAD_PARAMS    0x0227
#define LR20XX_OP_SET_LORA_CAD           0x0228
#define LR20XX_OP_GET_LORA_PKT_STATUS    0x022A

/* ── IRQ bits (bit positions identical in SDK, RadioLib and datasheet) ── */

#define LR20XX_IRQ_PREAMBLE_DETECTED     (1u << 5)
#define LR20XX_IRQ_SYNC_WORD_HEADER_VALID (1u << 6)
#define LR20XX_IRQ_CAD_DETECTED          (1u << 7)
#define LR20XX_IRQ_LORA_HEADER_ERROR     (1u << 9)
#define LR20XX_IRQ_LOW_BATTERY           (1u << 10)
#define LR20XX_IRQ_ERROR                 (1u << 16)
#define LR20XX_IRQ_CMD_ERROR             (1u << 17)
#define LR20XX_IRQ_RX_DONE               (1u << 18)
#define LR20XX_IRQ_TX_DONE               (1u << 19)
#define LR20XX_IRQ_CAD_DONE              (1u << 20)
#define LR20XX_IRQ_TIMEOUT               (1u << 21)
#define LR20XX_IRQ_CRC_ERROR             (1u << 22)
#define LR20XX_IRQ_ALL_MASK              0xFFFFFFFFu

/* Terminal-only IRQ mask routed to DIO (no intermediate preamble/header IRQs:
 * they would restart RX mid-packet.  Matches RadioLib/Meshtastic behavior.)
 * CAD_DETECTED|CAD_DONE added 2026-08: without them DIO8 never asserts for
 * CAD completion -> lr20xx_lora_cad always timed out (-116, LBT "proceeding
 * with TX").  RadioLib LR2021.cpp:544 uses CAD_DETECTED|CAD_DONE as the CAD
 * irqFlags default. */
#define LR20XX_DIO_IRQ_MASK \
	(LR20XX_IRQ_RX_DONE | LR20XX_IRQ_TX_DONE | LR20XX_IRQ_TIMEOUT | \
	 LR20XX_IRQ_CRC_ERROR | LR20XX_IRQ_LORA_HEADER_ERROR | \
	 LR20XX_IRQ_CAD_DONE | LR20XX_IRQ_CAD_DETECTED)

/* ── Constants ── */

#define LR20XX_STDBY_RC           0x00
#define LR20XX_PKT_TYPE_LORA      0x00
#define LR20XX_RX_PATH_LF         0x00
#define LR20XX_RX_PATH_HF         0x01
#define LR20XX_RX_BOOST_NONE      0x00
#define LR20XX_RX_BOOST_LF        0x01   /* working FW: default 0, boosted=1 */
#define LR20XX_FALLBACK_STBY_RC   0x01
#define LR20XX_DIO_FUNC_IRQ       0x01
#define LR20XX_DIO_DRIVE_NONE     0x00
#define LR20XX_CALIBRATE_ALL      0x6F
#define LR20XX_PA_SEL_LF          0x00
#define LR20XX_PA_LF_MODE_FSM     0x00
#define LR20XX_PA_HF_DUTY_UNUSED  16
#define LR20XX_RAMP_48_US         0x05
#define LR20XX_PKT_EXPLICIT       0x00
#define LR20XX_CRC_DISABLED       0x00
#define LR20XX_CRC_ENABLED        0x01
#define LR20XX_IQ_STANDARD        0x00
#define LR20XX_IQ_INVERTED        0x01
#define LR20XX_CAD_EXIT_STBY_RC   0x00
#define LR20XX_RX_DC_MODE_RX      0x00
#define LR20XX_VALUE_FORMAT_UNIT  0x01
#define LR20XX_MEAS_RES_12_BITS   0x04
#define LR20XX_RANDOM_SRC_PLL_ADC 0x03

#define LR20XX_RTC_FREQ_HZ        32768u
#define LR20XX_RX_TIMEOUT_INF     0xFFFFFFu   /* continuous RX */
#define LR20XX_TX_TIMEOUT_MS      5000u

/* ── Driver data structures ─────────────────────────────────────────── */

struct lr20xx_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec reset;
	struct gpio_dt_spec busy;
	struct gpio_dt_spec dio1;
	uint16_t tcxo_voltage_mv;
	uint32_t tcxo_startup_delay_ms;
	bool rx_boosted;
	uint8_t irq_dio_num;
};

struct lr20xx_data {
	const struct device *dev;
	struct k_mutex spi_mutex;

	/* Cached modem config from lora_config() */
	struct lora_modem_config modem_cfg;
	bool configured;

	/* Async RX state */
	lora_recv_cb async_rx_cb;
	void *async_rx_user_data;

	/* Async TX state */
	struct k_poll_signal *tx_signal;

	/* DIO1 work — runs on dedicated queue */
	struct k_work dio1_work;
	struct k_work_q dio1_wq;
	struct gpio_callback dio1_cb;
	uint8_t dio1_stuck_count;

	/* Radio state */
	volatile bool tx_active;
	volatile bool in_rx_mode;
	bool hw_initialized;

	/* RX buffer */
	uint8_t rx_buf[256];

	/* Extension features */
	bool rx_boost_enabled;
	bool rx_boost_applied;

	/* Duty-cycle timing (stored, re-used on re-arm) */
	uint32_t dc_rx_ms;
	uint32_t dc_sleep_ms;
	bool rx_duty_cycle_enabled;

	/* CAD state */
	struct k_sem cad_sem;
	int cad_result;
	lora_cad_cb cad_cb;
	void *cad_user_data;
	bool cad_active;
	int8_t cad_peak_offset;
	uint8_t cad_probe_peak;
};

#define LR20XX_DIO1_WQ_STACK_SIZE 2560
K_THREAD_STACK_DEFINE(lr20xx_dio1_wq_stack, LR20XX_DIO1_WQ_STACK_SIZE);

/* ── SPI primitives (two-phase, BUSY-guarded) ───────────────────────── */

static int lr_wait_busy(const struct lr20xx_config *cfg)
{
	int timeout = 1000;

	while (gpio_pin_get_dt(&cfg->busy) != 0) {
		k_msleep(1);
		if (--timeout <= 0) {
			LOG_ERR("BUSY stuck HIGH (1000 ms)");
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static uint8_t lr_cmd_buf_tx[70];
static uint8_t lr_cmd_buf_dummy[70];
static uint8_t lr_cmd_buf_rx[70];

/* FIFO buffers — large enough for a full 255-byte packet + 2 status bytes */
static uint8_t lr_fifo_buf_tx[258];
static uint8_t lr_fifo_buf_rx[258];

/* Send opcode (+ params); optionally read response (data after 2 status bytes).
 * Phase 1: CS low -> [opc_msb, opc_lsb, params...] -> CS high
 * Phase 2: CS low -> clock dummy -> read response -> CS high */
static int lr_cmd(const struct lr20xx_config *cfg, uint16_t opcode,
		  const uint8_t *params, size_t param_len,
		  uint8_t *resp, size_t resp_len)
{
	int ret = lr_wait_busy(cfg);
	if (ret) {
		return ret;
	}

	lr_cmd_buf_tx[0] = (uint8_t)(opcode >> 8);
	lr_cmd_buf_tx[1] = (uint8_t)(opcode >> 0);
	if (params && param_len > 0) {
		memcpy(lr_cmd_buf_tx + 2, params, param_len);
	}

	struct spi_buf tx_buf1 = { .buf = lr_cmd_buf_tx, .len = 2 + param_len };
	struct spi_buf_set tx_set1 = { .buffers = &tx_buf1, .count = 1 };

	ret = spi_transceive_dt(&cfg->bus, &tx_set1, NULL);
	if (ret) {
		return ret;
	}

	if (resp && resp_len > 0) {
		memset(lr_cmd_buf_dummy, 0, resp_len);
		memset(lr_cmd_buf_rx, 0, resp_len);

		struct spi_buf tx_buf2 = { .buf = lr_cmd_buf_dummy, .len = resp_len };
		struct spi_buf rx_buf  = { .buf = lr_cmd_buf_rx,    .len = resp_len };
		struct spi_buf_set tx_set2 = { .buffers = &tx_buf2, .count = 1 };
		struct spi_buf_set rx_set  = { .buffers = &rx_buf,  .count = 1 };

		ret = spi_transceive_dt(&cfg->bus, &tx_set2, &rx_set);
		if (ret) {
			return ret;
		}
		memcpy(resp, lr_cmd_buf_rx, resp_len);
	}

	return 0;
}

/* Direct FIFO write (opcode + payload in one frame) */
static int lr_fifo_write(const struct lr20xx_config *cfg, uint16_t opcode,
			 const uint8_t *data, uint8_t len)
{
	int ret = lr_wait_busy(cfg);
	if (ret) {
		return ret;
	}

	lr_fifo_buf_tx[0] = (uint8_t)(opcode >> 8);
	lr_fifo_buf_tx[1] = (uint8_t)(opcode >> 0);
	if (data && len > 0) {
		memcpy(lr_fifo_buf_tx + 2, data, len);
	}

	struct spi_buf tx_buf = { .buf = lr_fifo_buf_tx, .len = 2 + len };
	struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };

	return spi_transceive_dt(&cfg->bus, &tx_set, NULL);
}

/* Direct FIFO read: two-phase; response is [stat16, data...] */
static int lr_fifo_read(const struct lr20xx_config *cfg, uint16_t opcode,
			uint8_t *data, uint8_t len)
{
	int ret = lr_wait_busy(cfg);
	if (ret) {
		return ret;
	}

	/* Phase 1: opcode */
	lr_fifo_buf_tx[0] = (uint8_t)(opcode >> 8);
	lr_fifo_buf_tx[1] = (uint8_t)(opcode >> 0);
	struct spi_buf tx1 = { .buf = lr_fifo_buf_tx, .len = 2 };
	struct spi_buf_set set1 = { .buffers = &tx1, .count = 1 };
	ret = spi_transceive_dt(&cfg->bus, &set1, NULL);
	if (ret) {
		return ret;
	}

	/* Phase 2: clock 2 stat bytes + len data bytes */
	memset(lr_fifo_buf_tx, 0, 2 + len);
	memset(lr_fifo_buf_rx, 0, 2 + len);
	struct spi_buf tx2 = { .buf = lr_fifo_buf_tx, .len = 2 + len };
	struct spi_buf rx2 = { .buf = lr_fifo_buf_rx, .len = 2 + len };
	struct spi_buf_set set2 = { .buffers = &tx2, .count = 1 };
	struct spi_buf_set setr = { .buffers = &rx2, .count = 1 };
	ret = spi_transceive_dt(&cfg->bus, &set2, &setr);
	if (ret) {
		return ret;
	}
	if (data && len > 0) {
		memcpy(data, lr_fifo_buf_rx + 2, len);
	}
	return 0;
}

/* ── Command wrappers ───────────────────────────────────────────────── */

static int lr_get_status(const struct lr20xx_config *cfg,
			 uint8_t *stat1, uint8_t *stat2, uint32_t *irq)
{
	uint8_t resp[6] = { 0 };
	int ret = lr_cmd(cfg, LR20XX_OP_GET_STATUS, NULL, 0, resp, sizeof(resp));
	if (ret) {
		return ret;
	}
	if (stat1) {
		*stat1 = resp[0];
	}
	if (stat2) {
		*stat2 = resp[1];
	}
	if (irq) {
		*irq = ((uint32_t)resp[2] << 24) | ((uint32_t)resp[3] << 16) |
		       ((uint32_t)resp[4] << 8) | resp[5];
	}
	return 0;
}

static int lr_get_version(const struct lr20xx_config *cfg,
			  uint8_t *major, uint8_t *minor)
{
	/* LR2021 read responses are [stat16, data...] — the 2-byte status
	 * header precedes all read data (datasheet §5.4.1.2, confirmed by the
	 * Semtech SDK HAL which discards the header). */
	uint8_t resp[4] = { 0 };
	int ret = lr_cmd(cfg, LR20XX_OP_GET_VERSION, NULL, 0, resp, sizeof(resp));
	if (ret) {
		return ret;
	}
	if (major) {
		*major = resp[2];
	}
	if (minor) {
		*minor = resp[3];
	}
	LOG_INF("GET_VERSION raw: %02x %02x %02x %02x (major=%u minor=%u)",
		resp[0], resp[1], resp[2], resp[3],
		*major, *minor);
	return 0;
}

static int lr_get_errors(const struct lr20xx_config *cfg, uint16_t *errors)
{
	uint8_t resp[4] = { 0 };
	int ret = lr_cmd(cfg, LR20XX_OP_GET_ERRORS, NULL, 0, resp, sizeof(resp));
	if (ret) {
		return ret;
	}
	if (errors) {
		*errors = ((uint16_t)resp[2] << 8) | resp[3];
	}
	return 0;
}

static int lr_set_standby(const struct lr20xx_config *cfg, uint8_t mode)
{
	uint8_t p[1] = { mode };
	return lr_cmd(cfg, LR20XX_OP_SET_STANDBY, p, 1, NULL, 0);
}

static int lr_set_dio_function(const struct lr20xx_config *cfg, uint8_t dio,
			       uint8_t func, uint8_t drive)
{
	uint8_t p[2] = { dio, (uint8_t)((func << 4) | (drive & 0x0F)) };
	return lr_cmd(cfg, LR20XX_OP_SET_DIO_FUNC, p, 2, NULL, 0);
}

static int lr_set_dio_irq_cfg(const struct lr20xx_config *cfg, uint8_t dio,
			      uint32_t irq_mask)
{
	uint8_t p[5] = { dio,
			 (uint8_t)(irq_mask >> 24), (uint8_t)(irq_mask >> 16),
			 (uint8_t)(irq_mask >> 8),  (uint8_t)(irq_mask >> 0) };
	return lr_cmd(cfg, LR20XX_OP_SET_DIO_IRQ_CFG, p, 5, NULL, 0);
}

static int lr_clear_irq(const struct lr20xx_config *cfg, uint32_t mask)
{
	uint8_t p[4] = { (uint8_t)(mask >> 24), (uint8_t)(mask >> 16),
			 (uint8_t)(mask >> 8),  (uint8_t)(mask >> 0) };
	return lr_cmd(cfg, LR20XX_OP_CLEAR_IRQ, p, 4, NULL, 0);
}

static int lr_get_and_clear_irq(const struct lr20xx_config *cfg, uint32_t *irq)
{
	uint8_t resp[6] = { 0 };
	int ret = lr_cmd(cfg, LR20XX_OP_GET_AND_CLEAR_IRQ, NULL, 0,
			 resp, sizeof(resp));
	if (ret) {
		return ret;
	}
	if (irq) {
		*irq = ((uint32_t)resp[2] << 24) | ((uint32_t)resp[3] << 16) |
		       ((uint32_t)resp[4] << 8) | resp[5];
	}
	return 0;
}

static int lr_calibrate(const struct lr20xx_config *cfg, uint8_t blocks)
{
	uint8_t p[1] = { blocks };
	return lr_cmd(cfg, LR20XX_OP_CALIBRATE, p, 1, NULL, 0);
}

static int lr_calibrate_fe(const struct lr20xx_config *cfg)
{
	/* CalibFe 0x0123 — 3 fixed 16-bit freq entries (6 bytes).
	 * The WORKING Meshtastic firmware calibrates exactly these three
	 * bins (from the official Wio-LR2021 DTS): 470 MHz LF, 897.5 MHz LF,
	 * 2441 MHz HF — then waits 50 ms.  Calibrating only the operating
	 * bin (869.6/4 = 217) left the front end uncalibrated for RX. */
	uint8_t p[6] = { 0x00, 0x76,  /* 470   MHz / 4 = 0x0076 (LF) */
			 0x00, 0xE0,  /* 897.5 MHz / 4 = 0x00E0 (LF) */
			 0x82, 0x63 }; /* 2441 MHz / 4 | 0x8000 = 0x8263 (HF) */
	return lr_cmd(cfg, LR20XX_OP_CALIBRATE_FRONT_END, p, 6, NULL, 0);
}

static int lr_get_vbat(const struct lr20xx_config *cfg, uint16_t *vbat_mv)
{
	uint8_t p[1] = { (uint8_t)((LR20XX_VALUE_FORMAT_UNIT << 3) |
				  LR20XX_MEAS_RES_12_BITS) };
	uint8_t resp[4] = { 0 };
	int ret = lr_cmd(cfg, LR20XX_OP_GET_VBAT, p, 1, resp, sizeof(resp));
	if (ret) {
		return ret;
	}
	if (vbat_mv) {
		*vbat_mv = ((uint16_t)resp[2] << 8) | resp[3];
	}
	return 0;
}

static int lr_get_random(const struct lr20xx_config *cfg, uint32_t *random)
{
	uint8_t p[1] = { LR20XX_RANDOM_SRC_PLL_ADC };
	uint8_t resp[6] = { 0 };
	int ret = lr_cmd(cfg, LR20XX_OP_GET_RANDOM_NUMBER, p, 1,
			 resp, sizeof(resp));
	if (ret) {
		return ret;
	}
	if (random) {
		*random = ((uint32_t)resp[2] << 24) | ((uint32_t)resp[3] << 16) |
			  ((uint32_t)resp[4] << 8) | resp[5];
	}
	return 0;
}

static int lr_set_rf_frequency(const struct lr20xx_config *cfg, uint32_t freq_hz)
{
	uint8_t p[4] = { (uint8_t)(freq_hz >> 24), (uint8_t)(freq_hz >> 16),
			 (uint8_t)(freq_hz >> 8),  (uint8_t)(freq_hz >> 0) };
	return lr_cmd(cfg, LR20XX_OP_SET_RF_FREQUENCY, p, 4, NULL, 0);
}

static int lr_set_rx_path(const struct lr20xx_config *cfg, uint8_t path,
			  uint8_t boost)
{
	uint8_t p[2] = { path, boost };
	return lr_cmd(cfg, LR20XX_OP_SET_RX_PATH, p, 2, NULL, 0);
}

static int lr_set_pa_cfg(const struct lr20xx_config *cfg, uint8_t sel,
			 uint8_t lf_mode, uint8_t lf_duty, uint8_t lf_slices,
			 uint8_t hf_duty)
{
	uint8_t p[3] = { (uint8_t)((sel << 7) | lf_mode),
			 (uint8_t)((lf_duty << 4) | lf_slices), hf_duty };
	return lr_cmd(cfg, LR20XX_OP_SET_PA_CFG, p, 3, NULL, 0);
}

static int lr_set_tx_params(const struct lr20xx_config *cfg, int8_t power_half_dbm,
			    uint8_t ramp_time)
{
	uint8_t p[2] = { (uint8_t)power_half_dbm, ramp_time };
	return lr_cmd(cfg, LR20XX_OP_SET_TX_PARAMS, p, 2, NULL, 0);
}

static int lr_set_rx_tx_fallback(const struct lr20xx_config *cfg, uint8_t mode)
{
	uint8_t p[1] = { mode };
	return lr_cmd(cfg, LR20XX_OP_SET_RX_TX_FALLBACK, p, 1, NULL, 0);
}

static int lr_set_pkt_type(const struct lr20xx_config *cfg, uint8_t pkt_type)
{
	uint8_t p[1] = { pkt_type };
	return lr_cmd(cfg, LR20XX_OP_SET_PKT_TYPE, p, 1, NULL, 0);
}

static int lr_get_rssi_inst(const struct lr20xx_config *cfg, int16_t *rssi)
{
	uint8_t resp[4] = { 0 };
	int ret = lr_cmd(cfg, LR20XX_OP_GET_RSSI_INST, NULL, 0, resp, sizeof(resp));
	if (ret) {
		return ret;
	}
	if (rssi) {
		*rssi = -(int16_t)resp[2];
	}
	return 0;
}

static int lr_set_rx(const struct lr20xx_config *cfg, uint32_t timeout_rtc)
{
	uint8_t p[3] = { (uint8_t)(timeout_rtc >> 16),
			 (uint8_t)(timeout_rtc >> 8),
			 (uint8_t)(timeout_rtc >> 0) };
	return lr_cmd(cfg, LR20XX_OP_SET_RX, p, 3, NULL, 0);
}

static int lr_set_tx(const struct lr20xx_config *cfg, uint32_t timeout_rtc)
{
	uint8_t p[3] = { (uint8_t)(timeout_rtc >> 16),
			 (uint8_t)(timeout_rtc >> 8),
			 (uint8_t)(timeout_rtc >> 0) };
	return lr_cmd(cfg, LR20XX_OP_SET_TX, p, 3, NULL, 0);
}

static int lr_set_rx_duty_cycle(const struct lr20xx_config *cfg,
				uint32_t rx_rtc, uint32_t sleep_rtc)
{
	uint8_t p[7] = { (uint8_t)(rx_rtc >> 16), (uint8_t)(rx_rtc >> 8),
			 (uint8_t)(rx_rtc >> 0),
			 (uint8_t)(sleep_rtc >> 16), (uint8_t)(sleep_rtc >> 8),
			 (uint8_t)(sleep_rtc >> 0),
			 (uint8_t)(LR20XX_RX_DC_MODE_RX << 4) };
	return lr_cmd(cfg, LR20XX_OP_SET_RX_DUTY_CYCLE, p, 7, NULL, 0);
}

static int lr_get_rx_packet_length(const struct lr20xx_config *cfg,
				   uint16_t *pkt_len)
{
	uint8_t resp[4] = { 0 };
	int ret = lr_cmd(cfg, LR20XX_OP_GET_RX_PACKET_LENGTH, NULL, 0,
			 resp, sizeof(resp));
	if (ret) {
		return ret;
	}
	if (pkt_len) {
		*pkt_len = ((uint16_t)resp[2] << 8) | resp[3];
	}
	return 0;
}

static int lr_set_lora_mod_params(const struct lr20xx_config *cfg,
				  uint8_t sf, uint8_t bw, uint8_t cr,
				  uint8_t ldro)
{
	uint8_t p[2] = { (uint8_t)(((sf & 0x0F) << 4) | (bw & 0x0F)),
			 (uint8_t)(((cr & 0x0F) << 4) | (ldro & 0x0F)) };
	return lr_cmd(cfg, LR20XX_OP_SET_LORA_MOD_PARAMS, p, 2, NULL, 0);
}

static int lr_set_lora_pkt_params(const struct lr20xx_config *cfg,
				  uint16_t preamble, uint8_t payload_len,
				  uint8_t header_type, uint8_t crc_en,
				  uint8_t iq)
{
	uint8_t p[4] = { (uint8_t)(preamble >> 8), (uint8_t)(preamble >> 0),
			 payload_len,
			 (uint8_t)(((header_type & 0x01) << 2) |
				   ((crc_en & 0x01) << 1) | (iq & 0x01)) };
	return lr_cmd(cfg, LR20XX_OP_SET_LORA_PKT_PARAMS, p, 4, NULL, 0);
}

static int lr_set_lora_syncword(const struct lr20xx_config *cfg, uint8_t syncword)
{
	uint8_t p[1] = { syncword };
	return lr_cmd(cfg, LR20XX_OP_SET_LORA_SYNCWORD, p, 1, NULL, 0);
}

static int lr_set_lora_cad_params(const struct lr20xx_config *cfg,
				  uint8_t num_symbols, uint8_t pnr_delta,
				  uint8_t exit_mode, uint32_t timeout_pll,
				  uint8_t det_peak)
{
	uint8_t p[7] = { num_symbols, pnr_delta, exit_mode,
			 (uint8_t)(timeout_pll >> 16), (uint8_t)(timeout_pll >> 8),
			 (uint8_t)(timeout_pll >> 0), (uint8_t)(det_peak & 0x7F) };
	return lr_cmd(cfg, LR20XX_OP_SET_LORA_CAD_PARAMS, p, 7, NULL, 0);
}

static int lr_set_cad(const struct lr20xx_config *cfg)
{
	return lr_cmd(cfg, LR20XX_OP_SET_LORA_CAD, NULL, 0, NULL, 0);
}

/* RadioLib LR2021 packet-status decode — read 8 bytes: [stat16, 6 data]:
 *   resp[2] = flags (bit4: CRC ok, low nibble: CR)
 *   resp[3] = packet length
 *   resp[4] = SNR (0.25 dB steps, signed)
 *   resp[5] = RSSI packet byte, resp[6] = RSSI signal byte
 *   resp[7] = bit0: signal RSSI LSB, bit1: packet RSSI LSB, [5:2]: detector */
static int lr_get_lora_pkt_status(const struct lr20xx_config *cfg,
				  uint8_t *pkt_len, int16_t *rssi_pkt,
				  int16_t *rssi_signal, int8_t *snr)
{
	uint8_t resp[8] = { 0 };
	int ret = lr_cmd(cfg, LR20XX_OP_GET_LORA_PKT_STATUS, NULL, 0,
			 resp, sizeof(resp));
	if (ret) {
		return ret;
	}
	if (pkt_len) {
		*pkt_len = resp[3];
	}
	if (rssi_pkt) {
		int raw = ((int)resp[5] << 1) | ((resp[7] >> 1) & 1);
		*rssi_pkt = -raw / 2;
	}
	if (rssi_signal) {
		int raw = ((int)resp[6] << 1) | (resp[7] & 1);
		*rssi_signal = -raw / 2;
	}
	if (snr) {
		*snr = ((int8_t)resp[4]) / 4;
	}
	return 0;
}

/* ── Mapping helpers ────────────────────────────────────────────────── */

static uint8_t lr_bw_to_code(enum lora_signal_bandwidth bw)
{
	switch (bw) {
	case BW_7_KHZ:   return 0;   /* 7.81  kHz */
	case BW_15_KHZ:  return 1;   /* 15.63 kHz */
	case BW_31_KHZ:  return 2;   /* 31.25 kHz */
	case BW_62_KHZ:  return 3;   /* 62.5  kHz */
	case BW_125_KHZ: return 4;
	case BW_250_KHZ: return 5;
	case BW_500_KHZ: return 6;
	case BW_1000_KHZ: return 7;
	case BW_10_KHZ:  return 8;   /* 10.42 kHz */
	case BW_20_KHZ:  return 9;   /* 20.83 kHz */
	case BW_41_KHZ:  return 10;  /* 41.67 kHz */
	case BW_200_KHZ: return 13;  /* 203   kHz */
	case BW_400_KHZ: return 14;  /* 406   kHz */
	case BW_800_KHZ: return 15;  /* 812   kHz */
	default:         return 4;
	}
}

static float lr_bw_to_khz(enum lora_signal_bandwidth bw)
{
	switch (bw) {
	case BW_7_KHZ:   return 7.81f;
	case BW_10_KHZ:  return 10.42f;
	case BW_15_KHZ:  return 15.63f;
	case BW_20_KHZ:  return 20.83f;
	case BW_31_KHZ:  return 31.25f;
	case BW_41_KHZ:  return 41.67f;
	case BW_62_KHZ:  return 62.5f;
	case BW_200_KHZ: return 203.0f;
	case BW_400_KHZ: return 406.0f;
	case BW_800_KHZ: return 812.0f;
	default:         return (float)bw;
	}
}

/* RadioLib LDRO rule: enable when symbol time >= 16 ms */
static uint8_t lr_ldro_for(uint8_t sf, enum lora_signal_bandwidth bw)
{
	uint32_t sym_us = ((1u << sf) * 1000000u) /
			  (uint32_t)(lr_bw_to_khz(bw) * 1000.0f);
	return (sym_us >= 16000u) ? 1u : 0u;
}

/* RadioLib LR2021 LF PA table (verified): {duty, slices} per power */
static void lr_pa_cfg_for_power(int8_t power_dbm, uint8_t *duty, uint8_t *slices)
{
	if (power_dbm <= 0) {
		*duty = 0x02; *slices = 0x00;
	} else if (power_dbm <= 10) {
		*duty = 0x04; *slices = 0x01;
	} else if (power_dbm <= 15) {
		*duty = 0x05; *slices = 0x04;
	} else {
		*duty = 0x07; *slices = 0x04;
	}
}

/* RadioLib LR2021 CAD detPeak defaults per SF (SF5..SF12) */
static uint8_t lr_cad_detect_peak(uint8_t sf)
{
	static const uint8_t table[8] = { 48, 48, 50, 55, 55, 59, 61, 65 };
	if (sf < 5 || sf > 12) {
		return 55;
	}
	return table[sf - 5];
}

/* ── Chip-state dump (debug; console is ON in debug builds) ─────────── */

static void lr_dump_state(struct lr20xx_data *data, const char *tag)
{
	const struct lr20xx_config *cfg = data->dev->config;
	uint8_t stat1 = 0, stat2 = 0;
	uint32_t irq = 0;
	uint16_t errors = 0;

	if (lr_get_status(cfg, &stat1, &stat2, &irq) != 0) {
		LOG_INF("[%s] status read FAILED", tag);
		return;
	}
	lr_get_errors(cfg, &errors);
	LOG_INF("[%s] cmd=%d mode=%d st=0x%04x irq=0x%08x err=0x%04x BUSY=%d DIO=%d",
		tag, stat1 & 0x07, stat2 & 0x07,
		((uint16_t)stat1 << 8) | stat2, irq, errors,
		gpio_pin_get_dt(&cfg->busy), gpio_pin_get_dt(&cfg->dio1));
}

/* ── Hardware init (deferred to first config) ───────────────────────── */

static int lr20xx_hw_init(struct lr20xx_data *data,
			  const struct lr20xx_config *cfg)
{
	int ret;

	LOG_INF("LR20xx hardware init starting");

	/* HW reset: assert (active low) 10 ms, deassert, then wait for the
	 * typical 273 ms transition (RadioLib reset()) + BUSY low. */
	if (gpio_pin_set_dt(&cfg->reset, 1) != 0) {
		LOG_ERR("reset assert failed");
		return -EIO;
	}
	k_msleep(10);
	if (gpio_pin_set_dt(&cfg->reset, 0) != 0) {
		LOG_ERR("reset deassert failed");
		return -EIO;
	}
	k_msleep(300);
	ret = lr_wait_busy(cfg);
	if (ret) {
		LOG_WRN("BUSY still HIGH after reset — continuing");
	}

	/* Chip identity: expected firmware 1.24 (RadioLib findChip) */
	{
		uint8_t major = 0, minor = 0;
		for (int attempt = 0; attempt < 3; attempt++) {
			if (lr_get_version(cfg, &major, &minor) == 0) {
				break;
			}
			k_msleep(10);
		}
		LOG_INF("LR2021 GET_VERSION: %u.%u", major, minor);
	}

	/* RadioLib modSetup/config order.  NOTE: the working Meshtastic
	 * firmware sets lora.XTAL=true — the Wio-LR2021 TCXO is always-on and
	 * SetTcxoMode makes BUSY stick HIGH; we do NOT send SetTcxoMode. */
	ret = lr_set_standby(cfg, LR20XX_STDBY_RC);
	if (ret) {
		LOG_ERR("standby failed: %d", ret);
		return ret;
	}

	/* No SetRegMode: RadioLib (verified working) never switches the
	 * regulator mode; the old driver's DC-DC 0x01 did not help. */

	ret = lr_set_rx_tx_fallback(cfg, LR20XX_FALLBACK_STBY_RC);
	if (ret) {
		LOG_ERR("set_rx_tx_fallback failed: %d", ret);
		return ret;
	}

	lr_clear_irq(cfg, LR20XX_IRQ_ALL_MASK);

	/* DIO8 -> IRQ function (RadioLib config() does this) */
	ret = lr_set_dio_function(cfg, cfg->irq_dio_num,
				  LR20XX_DIO_FUNC_IRQ, LR20XX_DIO_DRIVE_NONE);
	if (ret) {
		LOG_ERR("set_dio_function failed: %d", ret);
		return ret;
	}

	/* Full calibration (RadioLib CALIBRATE_ALL = 0x6F) + BUSY wait */
	ret = lr_calibrate(cfg, LR20XX_CALIBRATE_ALL);
	if (ret) {
		LOG_ERR("calibrate(0x6F) failed: %d", ret);
		return ret;
	}
	k_msleep(5);
	ret = lr_wait_busy(cfg);
	if (ret) {
		LOG_ERR("BUSY stuck HIGH after calibrate");
		return -ETIMEDOUT;
	}

	ret = lr_set_pkt_type(cfg, LR20XX_PKT_TYPE_LORA);
	if (ret) {
		LOG_ERR("set_pkt_type failed: %d", ret);
		return ret;
	}

	/* Diagnostic only: chip VBAT sense (reads ~3 mV on this board — the
	 * sense path is not wired; RadioLib never reads it and works). */
	{
		uint16_t vbat = 0;
		int vrc = lr_get_vbat(cfg, &vbat);
		LOG_INF("init: chip VBAT reads %u mV (rc=%d)", vbat, vrc);
	}

	lr_clear_irq(cfg, LR20XX_IRQ_ALL_MASK);

	data->rx_boost_enabled = cfg->rx_boosted;
	data->rx_boost_applied = false;
	data->hw_initialized = true;
	LOG_INF("LR20xx driver ready");
	return 0;
}

/* ── Modem configuration (applied on every RX start / TX) ───────────── */

static void lr20xx_apply_modem_config(struct lr20xx_data *data,
				      bool tx_mode)
{
	const struct lr20xx_config *cfg = data->dev->config;
	struct lora_modem_config *mc = &data->modem_cfg;

	/* FE calibration with the fixed Wio-LR2021 bins (working firmware
	 * does this once before RX and waits 50 ms). */
	lr_calibrate_fe(cfg);
	k_msleep(50);
	lr_set_rf_frequency(cfg, mc->frequency);

	lr_set_rx_path(cfg, LR20XX_RX_PATH_LF,
		       data->rx_boost_enabled ? LR20XX_RX_BOOST_LF
					      : LR20XX_RX_BOOST_NONE);
	data->rx_boost_applied = data->rx_boost_enabled;

	lr_set_lora_mod_params(cfg, (uint8_t)mc->datarate,
			       lr_bw_to_code(mc->bandwidth),
			       (uint8_t)mc->coding_rate,
			       lr_ldro_for((uint8_t)mc->datarate, mc->bandwidth));

	lr_set_lora_pkt_params(cfg, mc->preamble_len, 255,
			       LR20XX_PKT_EXPLICIT,
			       mc->packet_crc_disable ? LR20XX_CRC_DISABLED
						      : LR20XX_CRC_ENABLED,
			       mc->iq_inverted ? LR20XX_IQ_INVERTED
					       : LR20XX_IQ_STANDARD);

	lr_set_lora_syncword(cfg, mc->public_network ? 0x34 : 0x12);

	if (tx_mode) {
		uint8_t duty = 0x04, slices = 0x01;
		lr_pa_cfg_for_power(mc->tx_power, &duty, &slices);
		lr_set_pa_cfg(cfg, LR20XX_PA_SEL_LF, LR20XX_PA_LF_MODE_FSM,
			      duty, slices, LR20XX_PA_HF_DUTY_UNUSED);
		/* Half-dBm units (RadioLib: power * 2) */
		lr_set_tx_params(cfg, (int8_t)(mc->tx_power * 2),
				 LR20XX_RAMP_48_US);
	}

	lr_set_dio_irq_cfg(cfg, cfg->irq_dio_num, LR20XX_DIO_IRQ_MASK);
}

/* ── Start / restart RX ─────────────────────────────────────────────── */

static void lr20xx_start_rx(struct lr20xx_data *data)
{
	const struct lr20xx_config *cfg = data->dev->config;

	lr_set_standby(cfg, LR20XX_STDBY_RC);
	lr_clear_irq(cfg, LR20XX_IRQ_ALL_MASK);
	lr_cmd(cfg, LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);

	lr20xx_apply_modem_config(data, false);

	if (data->rx_duty_cycle_enabled) {
		lr_set_rx_duty_cycle(cfg,
				     data->dc_rx_ms * LR20XX_RTC_FREQ_HZ / 1000u,
				     data->dc_sleep_ms * LR20XX_RTC_FREQ_HZ / 1000u);
	} else {
		lr_set_rx(cfg, LR20XX_RX_TIMEOUT_INF);
	}

	/* Clear IRQ flags set during modem configuration, then verify state.
	 * NOTE: GET_STATUS returns the status of the PREVIOUS command, so the
	 * clear_irq below must run AFTER the settle delay for the dump to
	 * show the settled chip mode (RX = 4), not the STBY->RX transition. */
	data->in_rx_mode = true;
	data->tx_active = false;

	k_msleep(3);
	lr_clear_irq(cfg, LR20XX_IRQ_ALL_MASK);
	lr_dump_state(data, "post-SET_RX");
}

static void lr20xx_restart_rx(struct lr20xx_data *data)
{
	const struct lr20xx_config *cfg = data->dev->config;

	lr_clear_irq(cfg, LR20XX_IRQ_ALL_MASK);

	if (data->rx_duty_cycle_enabled) {
		lr_set_rx_duty_cycle(cfg,
				     data->dc_rx_ms * LR20XX_RTC_FREQ_HZ / 1000u,
				     data->dc_sleep_ms * LR20XX_RTC_FREQ_HZ / 1000u);
	} else {
		lr_set_rx(cfg, LR20XX_RX_TIMEOUT_INF);
	}

	data->in_rx_mode = true;
}

/* ── DIO1 IRQ handler (work queue, thread context) ──────────────────── */

static void lr20xx_dio1_work_handler(struct k_work *work)
{
	struct lr20xx_data *data = CONTAINER_OF(work, struct lr20xx_data,
						dio1_work);
	const struct lr20xx_config *cfg = data->dev->config;
	bool rx_restarted = false;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	uint32_t irq = 0;
	int rc = lr_get_and_clear_irq(cfg, &irq);

	LOG_INF("DIO1 irq raw: 0x%08x (rc=%d, pin=%d)", irq, rc,
		gpio_pin_get_dt(&cfg->dio1));

	if (rc != 0) {
		LOG_ERR("Failed to read IRQ status (rc=%d)", rc);
		goto safety_check;
	}

	if (irq & LR20XX_IRQ_ERROR) {
		LOG_WRN("IRQ hardware ERROR: 0x%08x", irq);
	}
	if (irq != 0) {
		data->dio1_stuck_count = 0;
	}

	/* ── RX done (gated: CRC-failed packets assert RX_DONE + CRC_ERROR) */
	if ((irq & LR20XX_IRQ_RX_DONE) &&
	    !(irq & (LR20XX_IRQ_CRC_ERROR | LR20XX_IRQ_LORA_HEADER_ERROR))) {
		uint16_t pkt_len = 0;
		lr_get_rx_packet_length(cfg, &pkt_len);

		if (pkt_len > 0 && pkt_len <= 255) {
			uint8_t st_len = 0;
			int16_t rssi = 0, rssi_signal = 0;
			int8_t snr = 0;

			lr_get_lora_pkt_status(cfg, &st_len, &rssi,
					       &rssi_signal, &snr);
			if (st_len != 0) {
				pkt_len = st_len;
			}

			lr_fifo_read(cfg, LR20XX_OP_READ_RX_FIFO,
				     data->rx_buf, (uint8_t)pkt_len);

			/* Restart RX before firing callback */
			lr20xx_restart_rx(data);
			rx_restarted = true;

			/* When SNR < 0, use signal RSSI for weak links */
			if (snr < 0 && rssi_signal > rssi) {
				rssi = rssi_signal;
			}

			k_mutex_unlock(&data->spi_mutex);

			if (data->async_rx_cb) {
				data->async_rx_cb(data->dev, data->rx_buf,
						  (uint8_t)pkt_len,
						  rssi, snr,
						  data->async_rx_user_data);
			}
			return;
		}

		LOG_WRN("RX: invalid len %d", pkt_len);
		lr20xx_restart_rx(data);
		rx_restarted = true;
	}

	/* ── CAD done ── */
	if (irq & LR20XX_IRQ_CAD_DONE) {
		bool detected = (irq & LR20XX_IRQ_CAD_DETECTED) != 0;

		LOG_DBG("CAD done: %s", detected ? "activity" : "free");
		data->cad_active = false;

		if (data->cad_cb) {
			lora_cad_cb cb = data->cad_cb;
			void *ud = data->cad_user_data;

			data->cad_cb = NULL;
			data->cad_user_data = NULL;
			k_mutex_unlock(&data->spi_mutex);
			cb(data->dev, detected, ud);
			return;
		}

		data->cad_result = detected ? 1 : 0;
		k_sem_give(&data->cad_sem);
	}

	/* ── TX done ── */
	if (irq & LR20XX_IRQ_TX_DONE) {
		LOG_DBG("TX done");
		data->tx_active = false;

		lr20xx_start_rx(data);
		rx_restarted = true;

		if (data->tx_signal) {
			k_poll_signal_raise(data->tx_signal, 0);
		}
	}

	/* ── Timeout ── */
	if (irq & LR20XX_IRQ_TIMEOUT) {
		LOG_DBG("Timeout IRQ — restarting RX");
		if (!data->tx_active) {
			lr20xx_restart_rx(data);
			rx_restarted = true;
		}
	}

	/* ── CRC / Header error: drop FIFO residue, restart, notify ── */
	if (irq & (LR20XX_IRQ_CRC_ERROR | LR20XX_IRQ_LORA_HEADER_ERROR)) {
		LOG_WRN("RX error: CRC=%d HDR=%d RXDONE=%d",
			(irq & LR20XX_IRQ_CRC_ERROR) ? 1 : 0,
			(irq & LR20XX_IRQ_LORA_HEADER_ERROR) ? 1 : 0,
			(irq & LR20XX_IRQ_RX_DONE) ? 1 : 0);

		lr_cmd(cfg, LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);

		if (!data->tx_active) {
			lr20xx_restart_rx(data);
			rx_restarted = true;
		}

		k_mutex_unlock(&data->spi_mutex);

		if (data->async_rx_cb) {
			data->async_rx_cb(data->dev, NULL, 0, 0, 0,
					  data->async_rx_user_data);
		}
		return;
	}

safety_check:
	if (!rx_restarted && data->in_rx_mode && !data->tx_active) {
		LOG_WRN("DIO1 safety: no IRQ handled (0x%08x rc=%d), restarting RX",
			irq, rc);
		lr20xx_restart_rx(data);
	}

	/* Edge-triggered DIO1: if still HIGH, re-submit for pending flags.
	 * Stuck-DIO guard: after 5 empty cycles, hardware reset. */
	if (gpio_pin_get_dt(&cfg->dio1)) {
		data->dio1_stuck_count++;
		if (data->dio1_stuck_count >= 5) {
			LOG_ERR("DIO1 stuck HIGH for %d cycles — HW reset",
				data->dio1_stuck_count);
			data->dio1_stuck_count = 0;
			lr20xx_hw_init(data, cfg);
			lr20xx_start_rx(data);
		} else {
			k_work_submit_to_queue(&data->dio1_wq, &data->dio1_work);
		}
	} else {
		data->dio1_stuck_count = 0;
	}

	k_mutex_unlock(&data->spi_mutex);
}

static void lr20xx_dio1_callback(const struct device *dev,
				 struct gpio_callback *cb, uint32_t pins)
{
	struct lr20xx_data *data =
		CONTAINER_OF(cb, struct lr20xx_data, dio1_cb);

	k_work_submit_to_queue(&data->dio1_wq, &data->dio1_work);
}

/* ── Driver API: config ─────────────────────────────────────────────── */

static int lr20xx_lora_config(const struct device *dev,
			      struct lora_modem_config *config)
{
	struct lr20xx_data *data = dev->data;

	if (!data->hw_initialized) {
		int ret = lr20xx_hw_init(data, dev->config);
		if (ret != 0) {
			LOG_ERR("Hardware init failed: %d", ret);
			return ret;
		}
	}

	memcpy(&data->modem_cfg, config, sizeof(*config));
	data->configured = true;

	LOG_DBG("config: %uHz SF%d BW%d CR%d pwr=%d tx=%d",
		config->frequency, config->datarate, config->bandwidth,
		config->coding_rate, config->tx_power, config->tx);

	return 0;
}

/* ── Driver API: airtime ────────────────────────────────────────────── */

static uint32_t lr20xx_lora_airtime(const struct device *dev,
				    uint32_t data_len)
{
	struct lr20xx_data *data = dev->data;
	struct lora_modem_config *mc = &data->modem_cfg;

	uint8_t sf = (uint8_t)mc->datarate;
	float bw = lr_bw_to_khz(mc->bandwidth) * 1000.0f;
	uint8_t cr = (uint8_t)mc->coding_rate + 4;

	float ts = (float)(1 << sf) / bw;
	int de = (sf >= 11 && bw <= 125000.0f) ? 1 : 0;
	float n_payload = 8.0f + fmaxf(
		ceilf((8.0f * data_len - 4.0f * sf + 28.0f + 16.0f) /
		      (4.0f * (sf - 2.0f * de))) * cr,
		0.0f);
	float t_preamble = (mc->preamble_len + 4.25f) * ts;
	float t_payload = n_payload * ts;

	return (uint32_t)((t_preamble + t_payload) * 1000.0f);
}

/* ── Driver API: CAD ────────────────────────────────────────────────── */

static int lr20xx_do_cad(struct lr20xx_data *data)
{
	const struct lr20xx_config *cfg = data->dev->config;
	struct lora_modem_config *mc = &data->modem_cfg;

	uint8_t sf = (uint8_t)mc->datarate;
	uint8_t symb_nb = mc->cad.symbol_num ? (uint8_t)mc->cad.symbol_num : 2;
	uint8_t peak = lr_cad_detect_peak(sf);

	if (mc->cad.detection_peak != 0) {
		peak = mc->cad.detection_peak;
	} else if (data->cad_peak_offset != 0) {
		int p = (int)peak + data->cad_peak_offset;
		peak = (uint8_t)((p < 48) ? 48 : ((p > 90) ? 90 : p));
	}
	if (data->cad_probe_peak != 0) {
		peak = data->cad_probe_peak;
	}

	lr_set_lora_cad_params(cfg, symb_nb, 0, LR20XX_CAD_EXIT_STBY_RC,
			       0, peak);
	lr_clear_irq(cfg, LR20XX_IRQ_ALL_MASK);
	data->cad_active = true;
	return lr_set_cad(cfg);
}

static int lr20xx_cad_timeout_ms(struct lr20xx_data *data)
{
	struct lora_modem_config *mc = &data->modem_cfg;
	uint8_t sf = (uint8_t)mc->datarate;
	uint8_t symb_nb = mc->cad.symbol_num ? (uint8_t)mc->cad.symbol_num : 2;
	uint32_t bw_hz = (uint32_t)(lr_bw_to_khz(mc->bandwidth) * 1000.0f);

	if (bw_hz == 0 || sf < 5 || sf > 12) {
		return 200;
	}

	uint32_t tsym_us = ((1UL << sf) * 1000000UL) / bw_hz;
	uint32_t ms = ((symb_nb + 1U) * tsym_us) / 1000U + 100U;

	return MAX(ms, 200U);
}

static int lr20xx_lora_cad(const struct device *dev, k_timeout_t timeout)
{
	struct lr20xx_data *data = dev->data;
	int ret;

	if (!data->configured) {
		return -EINVAL;
	}

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	bool was_in_rx = data->in_rx_mode;
	if (was_in_rx) {
		data->in_rx_mode = false;
		lr_set_standby(dev->config, LR20XX_STDBY_RC);
	}

	k_sem_reset(&data->cad_sem);
	data->cad_result = -ETIMEDOUT;
	data->cad_cb = NULL;

	ret = lr20xx_do_cad(data);
	k_mutex_unlock(&data->spi_mutex);

	if (ret < 0) {
		return ret;
	}

	ret = k_sem_take(&data->cad_sem, timeout);
	if (ret == -EAGAIN) {
		data->cad_active = false;
		return -ETIMEDOUT;
	}

	LOG_INF("cad result: %d", data->cad_result);
	return data->cad_result;
}

static int lr20xx_lora_cad_async(const struct device *dev,
				 lora_cad_cb cb, void *user_data)
{
	struct lr20xx_data *data = dev->data;

	if (cb == NULL) {
		data->cad_cb = NULL;
		data->cad_user_data = NULL;
		data->cad_active = false;
		return 0;
	}

	if (!data->configured) {
		return -EINVAL;
	}

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	bool was_in_rx = data->in_rx_mode;
	if (was_in_rx) {
		data->in_rx_mode = false;
		lr_set_standby(dev->config, LR20XX_STDBY_RC);
	}

	data->cad_cb = cb;
	data->cad_user_data = user_data;

	int ret = lr20xx_do_cad(data);
	k_mutex_unlock(&data->spi_mutex);

	return ret;
}

/* ── Driver API: send_async ─────────────────────────────────────────── */

static int lr20xx_lora_send_async(const struct device *dev,
				  uint8_t *buf, uint32_t data_len,
				  struct k_poll_signal *async)
{
	struct lr20xx_data *data = dev->data;
	const struct lr20xx_config *cfg = dev->config;

	if (!data->configured) {
		return -EINVAL;
	}
	if (data->tx_active) {
		return -EBUSY;
	}
	if (data_len > 255 || data_len == 0) {
		return -EINVAL;
	}

	/* LBT: blocking CAD before TX; restore RX on busy */
	if (data->modem_cfg.cad.mode == LORA_CAD_MODE_LBT) {
		bool was_in_rx = data->in_rx_mode;
		int cad_ret = lr20xx_lora_cad(dev,
					      K_MSEC(lr20xx_cad_timeout_ms(data)));
		if (cad_ret > 0) {
			LOG_DBG("LBT: channel busy");
			if (was_in_rx && data->async_rx_cb != NULL) {
				k_mutex_lock(&data->spi_mutex, K_FOREVER);
				lr20xx_start_rx(data);
				k_mutex_unlock(&data->spi_mutex);
			}
			return -EBUSY;
		}
		if (cad_ret < 0 && cad_ret != -ENOSYS) {
			LOG_WRN("LBT: CAD failed (%d), proceeding with TX", cad_ret);
		}
	}

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	data->async_rx_cb = NULL;
	data->in_rx_mode = false;

	lr_set_standby(cfg, LR20XX_STDBY_RC);
	lr_clear_irq(cfg, LR20XX_IRQ_ALL_MASK);
	lr_cmd(cfg, LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);

	lr20xx_apply_modem_config(data, true);

	/* TX-specific packet length */
	lr_set_lora_pkt_params(cfg, data->modem_cfg.preamble_len,
			       (uint8_t)data_len, LR20XX_PKT_EXPLICIT,
			       data->modem_cfg.packet_crc_disable
				       ? LR20XX_CRC_DISABLED
				       : LR20XX_CRC_ENABLED,
			       data->modem_cfg.iq_inverted
				       ? LR20XX_IQ_INVERTED
				       : LR20XX_IQ_STANDARD);

	lr_fifo_write(cfg, LR20XX_OP_WRITE_TX_FIFO, buf, (uint8_t)data_len);

	lr_clear_irq(cfg, LR20XX_IRQ_ALL_MASK);

	data->tx_signal = async;
	data->tx_active = true;

	/* 5 s TX timeout (RTC steps) */
	lr_set_tx(cfg, LR20XX_TX_TIMEOUT_MS * LR20XX_RTC_FREQ_HZ / 1000u);

	lr_dump_state(data, "post-SET_TX");

	/* Poll for TX_DONE (RadioLib-style) instead of relying on the DIO8
	 * edge — the edge fired at TX start and was consumed, so TX_DONE
	 * never woke the work handler and TX always timed out.  Holding the
	 * SPI mutex during the poll is safe (no concurrent radio access; the
	 * DIO work handler just blocks on the mutex). */
	int64_t tx_start = k_uptime_get();
	uint32_t irq = 0;
	while ((k_uptime_get() - tx_start) < 6000) {
		if (lr_get_and_clear_irq(cfg, &irq) == 0 &&
		    (irq & LR20XX_IRQ_TX_DONE)) {
			break;
		}
		k_msleep(2);
	}

	if (irq & LR20XX_IRQ_TX_DONE) {
		LOG_INF("TX done (polled, irq=0x%08x)", irq);
	} else {
		LOG_WRN("TX done poll TIMEOUT (last irq=0x%08x)", irq);
	}

	data->tx_active = false;

	if (data->tx_signal) {
		k_poll_signal_raise(data->tx_signal, 0);
	}

	/* Back to RX */
	lr20xx_start_rx(data);

	k_mutex_unlock(&data->spi_mutex);

	return 0;
}

/* ── Driver API: send (sync) ────────────────────────────────────────── */

static int lr20xx_lora_send(const struct device *dev,
			    uint8_t *buf, uint32_t data_len)
{
	struct k_poll_signal done = K_POLL_SIGNAL_INITIALIZER(done);
	struct k_poll_event evt = K_POLL_EVENT_INITIALIZER(
		K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &done);

	int ret = lr20xx_lora_send_async(dev, buf, data_len, &done);
	if (ret < 0) {
		return ret;
	}

	uint32_t air_time = lr20xx_lora_airtime(dev, data_len);
	ret = k_poll(&evt, 1, K_MSEC(2 * air_time + 1000));
	if (ret < 0) {
		LOG_ERR("TX sync timeout");
		return ret;
	}

	return 0;
}

/* ── Driver API: recv_async ─────────────────────────────────────────── */

static int lr20xx_lora_recv_async(const struct device *dev,
				  lora_recv_cb cb, void *user_data)
{
	struct lr20xx_data *data = dev->data;

	if (cb == NULL) {
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		data->async_rx_cb = NULL;
		data->async_rx_user_data = NULL;
		data->in_rx_mode = false;
		k_mutex_unlock(&data->spi_mutex);
		return 0;
	}

	if (!data->configured) {
		return -EINVAL;
	}

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	data->async_rx_cb = cb;
	data->async_rx_user_data = user_data;
	data->rx_duty_cycle_enabled = false;

	lr20xx_start_rx(data);

	k_mutex_unlock(&data->spi_mutex);

	return 0;
}

/* ── Driver API: recv (sync) — not supported (async only) ───────────── */

static int lr20xx_lora_recv(const struct device *dev, uint8_t *buf,
			    uint8_t size, k_timeout_t timeout,
			    int16_t *rssi, int8_t *snr)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	ARG_UNUSED(size);
	ARG_UNUSED(timeout);
	ARG_UNUSED(rssi);
	ARG_UNUSED(snr);
	return -ENOTSUP;
}

/* ── Driver API: recv_duty_cycle ────────────────────────────────────── */

static int lr20xx_lora_recv_duty_cycle(const struct device *dev,
				       k_timeout_t rx_period,
				       k_timeout_t sleep_period,
				       lora_recv_cb cb, void *user_data)
{
	struct lr20xx_data *data = dev->data;
	const struct lr20xx_config *cfg = dev->config;

	if (cb == NULL) {
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		data->async_rx_cb = NULL;
		data->async_rx_user_data = NULL;
		data->in_rx_mode = false;
		k_mutex_unlock(&data->spi_mutex);
		return 0;
	}

	if (!data->configured) {
		return -EINVAL;
	}

	if (K_TIMEOUT_EQ(rx_period, K_FOREVER) ||
	    K_TIMEOUT_EQ(sleep_period, K_FOREVER)) {
		LOG_ERR("recv_duty_cycle: explicit rx/sleep periods required");
		return -EINVAL;
	}

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	data->async_rx_cb = cb;
	data->async_rx_user_data = user_data;

	lr_set_standby(cfg, LR20XX_STDBY_RC);
	lr_clear_irq(cfg, LR20XX_IRQ_ALL_MASK);
	lr_cmd(cfg, LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);
	lr20xx_apply_modem_config(data, false);

	uint32_t rx_ms = k_ticks_to_ms_ceil32(rx_period.ticks);
	uint32_t slp_ms = k_ticks_to_ms_ceil32(sleep_period.ticks);
	if (rx_ms < 1) {
		rx_ms = 1;
	}
	if (slp_ms < 1) {
		slp_ms = 1;
	}

	data->dc_rx_ms = rx_ms;
	data->dc_sleep_ms = slp_ms;
	data->rx_duty_cycle_enabled = true;

	lr_set_rx_duty_cycle(cfg,
			     rx_ms * LR20XX_RTC_FREQ_HZ / 1000u,
			     slp_ms * LR20XX_RTC_FREQ_HZ / 1000u);

	lr_clear_irq(cfg, LR20XX_IRQ_ALL_MASK);
	data->in_rx_mode = true;
	data->tx_active = false;

	LOG_INF("recv_duty_cycle: rx=%ums sleep=%ums", rx_ms, slp_ms);

	k_mutex_unlock(&data->spi_mutex);
	return 0;
}

/* ── LR20xx extension API (used by LR2021Radio.cpp) ─────────────────── */

int16_t lr20xx_get_rssi_inst(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	int16_t rssi = -128;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);
	lr_get_rssi_inst(dev->config, &rssi);
	k_mutex_unlock(&data->spi_mutex);

	return rssi;
}

bool lr20xx_is_receiving(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;

	if (!data->in_rx_mode || data->tx_active) {
		return false;
	}

	if (k_mutex_lock(&data->spi_mutex, K_NO_WAIT) != 0) {
		return false;
	}

	uint32_t irq = 0;
	lr_get_status(dev->config, NULL, NULL, &irq);
	k_mutex_unlock(&data->spi_mutex);

	return (irq & (LR20XX_IRQ_PREAMBLE_DETECTED |
		       LR20XX_IRQ_SYNC_WORD_HEADER_VALID)) != 0;
}

void lr20xx_set_rx_boost(const struct device *dev, bool enable)
{
	struct lr20xx_data *data = dev->data;

	if (data->rx_boost_enabled == enable) {
		return;
	}

	data->rx_boost_enabled = enable;
	LOG_DBG("RX boost %s", enable ? "enabled" : "disabled");

	if (data->in_rx_mode && data->configured) {
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		lr_set_rx_path(dev->config, LR20XX_RX_PATH_LF,
			       enable ? LR20XX_RX_BOOST_LF : LR20XX_RX_BOOST_NONE);
		data->rx_boost_applied = enable;
		k_mutex_unlock(&data->spi_mutex);
	} else {
		data->rx_boost_applied = false;
	}
}

uint32_t lr20xx_get_random(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	uint32_t random = 0;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);
	lr_get_random(dev->config, &random);
	k_mutex_unlock(&data->spi_mutex);

	return random;
}

void lr20xx_reset_agc(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	const struct lr20xx_config *cfg = dev->config;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	/* Recalibrate analog blocks (resets AGC/gain state) */
	lr_set_standby(cfg, LR20XX_STDBY_RC);
	lr_calibrate(cfg, LR20XX_CALIBRATE_ALL);
	k_msleep(5);
	lr_wait_busy(cfg);

	if (data->configured) {
		lr_calibrate_fe(cfg);
	}
	if (data->rx_boost_enabled) {
		lr_set_rx_path(cfg, LR20XX_RX_PATH_LF, LR20XX_RX_BOOST_LF);
		data->rx_boost_applied = true;
	}

	k_mutex_unlock(&data->spi_mutex);
}

void lr20xx_cad_set_peak_offset(const struct device *dev, int8_t offset)
{
	struct lr20xx_data *data = dev->data;

	data->cad_peak_offset = offset;
}

uint8_t lr20xx_cad_base_peak(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;

	return lr_cad_detect_peak((uint8_t)data->modem_cfg.datarate);
}

int lr20xx_cad_probe(const struct device *dev, int8_t peak_offset)
{
	struct lr20xx_data *data = dev->data;
	int base = (int)lr20xx_cad_base_peak(dev);
	int peak = base + peak_offset;

	if (peak < 48) {
		peak = 48;
	} else if (peak > 90) {
		peak = 90;
	}

	data->cad_probe_peak = (uint8_t)peak;
	int ret = lr20xx_lora_cad(dev, K_MSEC(lr20xx_cad_timeout_ms(data)));
	data->cad_probe_peak = 0;

	return ret;
}

/* ── Driver init (POST_KERNEL) ──────────────────────────────────────── */

static int lr20xx_lora_init(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	const struct lr20xx_config *cfg = dev->config;
	int ret;

	data->dev = dev;
	data->hw_initialized = false;

	k_mutex_init(&data->spi_mutex);
	k_sem_init(&data->cad_sem, 0, 1);
	k_work_init(&data->dio1_work, lr20xx_dio1_work_handler);

	k_work_queue_start(&data->dio1_wq, lr20xx_dio1_wq_stack,
			   K_THREAD_STACK_SIZEOF(lr20xx_dio1_wq_stack),
			   K_PRIO_COOP(7), NULL);
	k_thread_name_set(&data->dio1_wq.thread, "lr20xx_dio1");

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->busy)) {
		LOG_ERR("BUSY GPIO not ready");
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&cfg->reset)) {
		LOG_ERR("RESET GPIO not ready");
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&cfg->dio1)) {
		LOG_ERR("DIO1 GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->busy, GPIO_INPUT);
	if (ret) {
		return ret;
	}
	ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}
	ret = gpio_pin_configure_dt(&cfg->dio1, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	gpio_init_callback(&data->dio1_cb, lr20xx_dio1_callback,
			   BIT(cfg->dio1.pin));
	ret = gpio_add_callback(cfg->dio1.port, &data->dio1_cb);
	if (ret) {
		return ret;
	}
	ret = gpio_pin_interrupt_configure_dt(&cfg->dio1,
					       GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		return ret;
	}

	LOG_INF("LR20xx driver registered (hw init deferred to first config)");
	return 0;
}

/* ── Device instantiation ───────────────────────────────────────────── */

static DEVICE_API(lora, lr20xx_lora_api) = {
	.config          = lr20xx_lora_config,
	.airtime         = lr20xx_lora_airtime,
	.send            = lr20xx_lora_send,
	.send_async      = lr20xx_lora_send_async,
	.recv            = lr20xx_lora_recv,
	.recv_async      = lr20xx_lora_recv_async,
	.cad             = lr20xx_lora_cad,
	.cad_async       = lr20xx_lora_cad_async,
	.recv_duty_cycle = lr20xx_lora_recv_duty_cycle,
};

#define LR20XX_INIT(n)                                                       \
	static const struct lr20xx_config lr20xx_config_##n = {              \
		.bus = SPI_DT_SPEC_INST_GET(n,                               \
			SPI_WORD_SET(8) | SPI_OP_MODE_MASTER |               \
			SPI_TRANSFER_MSB),                                   \
		.reset = GPIO_DT_SPEC_INST_GET(n, reset_gpios),              \
		.busy  = GPIO_DT_SPEC_INST_GET(n, busy_gpios),              \
		.dio1  = GPIO_DT_SPEC_INST_GET(n, dio1_gpios),              \
		.tcxo_voltage_mv =                                           \
			DT_INST_PROP_OR(n, tcxo_voltage_mv, 0),             \
		.tcxo_startup_delay_ms =                                     \
			DT_INST_PROP_OR(n, tcxo_startup_delay_ms, 5),       \
		.rx_boosted       = DT_INST_PROP(n, rx_boosted),            \
		.irq_dio_num      = DT_INST_PROP_OR(n, irq_dio_num, 8),    \
	};                                                                   \
	static struct lr20xx_data lr20xx_data_##n;                           \
	DEVICE_DT_INST_DEFINE(n, lr20xx_lora_init, NULL,                     \
			      &lr20xx_data_##n, &lr20xx_config_##n,          \
			      POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,        \
			      &lr20xx_lora_api);

DT_INST_FOREACH_STATUS_OKAY(LR20XX_INIT)
