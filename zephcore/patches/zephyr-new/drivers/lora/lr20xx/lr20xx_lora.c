/*
 * SPDX-License-Identifier: MIT
 * LR20xx Zephyr LoRa driver — direct-SPI implementation for the Semtech LR2021
 *
 * Wire behavior ported from RadioLib's LR2021 module (verified working on the
 * Wio-LR2021 + XIAO nRF54L15) and
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

/* ── IRQ bits (verified against lr2021_status.rs, 2026-07-31) ── */
/* Low bits (per-packet events): */
#define LR20XX_IRQ_RX_FIFO                (1u << 0)
#define LR20XX_IRQ_TX_FIFO                (1u << 1)
#define LR20XX_IRQ_RNG_REQ_VLD            (1u << 2)
#define LR20XX_IRQ_TX_TIMESTAMP           (1u << 3)
#define LR20XX_IRQ_RX_TIMESTAMP           (1u << 4)
#define LR20XX_IRQ_PREAMBLE_DETECTED      (1u << 5)
#define LR20XX_IRQ_SYNC_WORD_HEADER_VALID (1u << 6)
#define LR20XX_IRQ_CAD_DETECTED           (1u << 7)
#define LR20XX_IRQ_LORA_HDR_TIMESTAMP     (1u << 8)
#define LR20XX_IRQ_LORA_HEADER_ERROR      (1u << 9)
#define LR20XX_IRQ_EOL                    (1u << 10)
#define LR20XX_IRQ_PA                     (1u << 11)
#define LR20XX_IRQ_LORA_TX_RX_HOP         (1u << 12)
#define LR20XX_IRQ_SYNC_FAIL              (1u << 13)
#define LR20XX_IRQ_LORA_SYMBOL_END        (1u << 14)
#define LR20XX_IRQ_LORA_TIMESTAMP_STAT    (1u << 15)
/* High bits (completion events — same bit positions as SX126x): */
#define LR20XX_IRQ_ERROR                  (1u << 16)
#define LR20XX_IRQ_CMD_ERROR              (1u << 17)
#define LR20XX_IRQ_RX_DONE                (1u << 18)
#define LR20XX_IRQ_TX_DONE                (1u << 19)
#define LR20XX_IRQ_CAD_DONE               (1u << 20)
#define LR20XX_IRQ_TIMEOUT                (1u << 21)
#define LR20XX_IRQ_CRC_ERROR              (1u << 22)
#define LR20XX_IRQ_LEN_ERROR              (1u << 23)
#define LR20XX_IRQ_ADDR_ERROR             (1u << 24)
#define LR20XX_IRQ_FHSS                   (1u << 25)
#define LR20XX_IRQ_INTER_PACKET1          (1u << 26)
#define LR20XX_IRQ_INTER_PACKET2          (1u << 27)
#define LR20XX_IRQ_RNG_RESP_DONE          (1u << 28)
#define LR20XX_IRQ_RNG_REQ_DIS            (1u << 29)
#define LR20XX_IRQ_RNG_EXCH_VLD           (1u << 30)
#define LR20XX_IRQ_RNG_TIMEOUT            (1u << 31)
#define LR20XX_IRQ_ALL_MASK               0xFFFFFFFFu

/* FHSS/ranging noise IRQs that fire spuriously after TX→RX transitions.
 * Also includes ERROR (bit 16) — the LR2021 sets it alongside noise bits
 * (observed: 0x2b010000 = RNG_REQ_DIS|INTER_PACKET2|FHSS|ADDR_ERROR|ERROR).
 * These are NOT real errors — they're internal state-machine side effects.
 * Filtering them out before safety_check prevents unnecessary RX restarts
 * (~70 ms deaf window) that lose ~2/3 of inbound packets. */
#define LR20XX_IRQ_NOISE_MASK \
	(LR20XX_IRQ_FHSS | LR20XX_IRQ_RNG_RESP_DONE | \
	 LR20XX_IRQ_RNG_REQ_DIS | LR20XX_IRQ_RNG_EXCH_VLD | \
	 LR20XX_IRQ_RNG_TIMEOUT | LR20XX_IRQ_INTER_PACKET1 | \
	 LR20XX_IRQ_INTER_PACKET2 | LR20XX_IRQ_ADDR_ERROR | \
	 LR20XX_IRQ_ERROR)

/* Terminal-only IRQ mask routed to DIO (no intermediate preamble/header IRQs:
 * they would restart RX mid-packet.  Matches RadioLib behavior.)
 * FHSS and ranging interrupts are not used — excluded from the mask so
 * they never wake the DIO handler. */
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
#define LR20XX_RX_BOOST_HF        0x04   /* RadioLib RX_BOOST_HF — 2.4 GHz ISM path */
#define LR20XX_FALLBACK_STBY_RC   0x01
#define LR20XX_DIO_FUNC_IRQ       0x01
#define LR20XX_DIO_DRIVE_NONE     0x00
#define LR20XX_CALIBRATE_ALL      0x6F
#define LR20XX_PA_SEL_LF          0x00
#define LR20XX_PA_SEL_HF          0x01   /* RadioLib setPaConfig(highFreq) → sel bit */
#define LR20XX_PA_LF_MODE_FSM     0x00
#define LR20XX_PA_LF_DUTY_UNUSED  0x06   /* RadioLib PA_LF_DUTY_CYCLE_UNUSED (LF PA off) */
#define LR20XX_PA_LF_SLICES_UNUSED 0x07  /* RadioLib PA_LF_SLICES_UNUSED */
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

/* Band split — RadioLib LF_CUTOFF_FREQ = 1500 MHz.  Above → 2.4 GHz ISM / S-band. */
#define LR20XX_LF_CUTOFF_HZ       1500000000u

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

/* FIFO buffers — large enough for a full 255-byte packet + 2 status bytes */
static uint8_t lr_fifo_buf_tx[258];

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

	if (resp && resp_len > 0) {
		/* Single NSS transaction for reads.  A two-phase read (CS
		 * toggled between opcode and data) makes the LR2021 answer
		 * with its status+IRQ word instead of the real response —
		 * the same defect as the FIFO read (fixed in 7a4feb3).  Live
		 * proof: every "GetRxPktLength" result equaled the pending
		 * IRQ upper halfword (0x00040170→4, 0x00060170→6,
		 * 0x00020320→2), and the original "invalid len 0" was the
		 * cleared IRQ word (0x00000000).  In one transaction the chip
		 * streams [stat16][data...]: resp[0..1] = status (during the
		 * opcode clock), resp[2..] = data — all existing wrapper
		 * offsets stay valid.  (All reads in this driver carry no
		 * params, so tx length == rx length.) */
		if (resp_len < 2) {
			return -EINVAL;
		}
		memset(lr_cmd_buf_dummy, 0, resp_len - 2);

		struct spi_buf tx_bufs[2] = {
			{ .buf = lr_cmd_buf_tx, .len = 2 + param_len },
			{ .buf = lr_cmd_buf_dummy, .len = resp_len - 2 },
		};
		struct spi_buf rx_buf = { .buf = resp, .len = resp_len };
		struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = 2 };
		struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

		return spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
	}

	/* Write path: single opcode+params transaction, no response */
	struct spi_buf tx_buf = { .buf = lr_cmd_buf_tx, .len = 2 + param_len };
	struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	return spi_transceive_dt(&cfg->bus, &tx_set, NULL);
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

/* Direct FIFO read — single NSS transaction (Semtech SDK
 * lr20xx_hal_direct_read_fifo / RadioLib readRadioRxFifo parity).  The
 * LR2021 streams the RX FIFO immediately after the opcode; CS must stay
 * asserted for the whole transfer.  A two-phase read (CS toggle between
 * opcode and data) aborts the stream and the chip answers with its
 * status+IRQ word instead of FIFO data (observed live: the "FIFO" bytes
 * 00 02 03 20 were exactly the pending IRQ 0x00020320).  Response
 * layout: [opcode echo / status 2B][FIFO data...] — the first two
 * response bytes are discarded (NULL rx buffer), FIFO data lands
 * directly in `data`. */
static int lr_fifo_read(const struct lr20xx_config *cfg, uint16_t opcode,
			uint8_t *data, uint8_t len)
{
	int ret = lr_wait_busy(cfg);
	if (ret) {
		return ret;
	}
	if (len == 0 || data == NULL) {
		return 0;
	}

	lr_fifo_buf_tx[0] = (uint8_t)(opcode >> 8);
	lr_fifo_buf_tx[1] = (uint8_t)(opcode >> 0);

	/* Command on MOSI, FIFO data on MISO, overlapped in one transfer.
	 * NULL tx buf → nRF SPIM sends 0x00 during the data phase. */
	struct spi_buf tx_bufs[2] = {
		{ .buf = lr_fifo_buf_tx, .len = 2 },
		{ .buf = NULL, .len = len },
	};
	struct spi_buf rx_bufs[2] = {
		{ .buf = NULL, .len = 2 },
		{ .buf = data, .len = len },
	};
	struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = 2 };
	struct spi_buf_set rx_set = { .buffers = rx_bufs, .count = 2 };

	return spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
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

/* IRQ read for the DIO work handler — LR2021 has NO non-destructive IRQ read.
 * GetStatus (0x0100) returns only [stat16] with a single "IRQ pending" bit
 * at position 8.  To get the full IRQ flags, GetAndClearIrq (0x0117) MUST be
 * used; it clears ALL flags atomically and returns [stat16][irq32] (6 bytes).
 * The handler saves the irq value and dispatches; no redundant lr_clear_irq
 * calls are needed downstream.  (The old SX126x pattern — read IRQ, consume
 * data, clear last — was built for a different chip; the LR2021 FIFO and
 * packet-length registers survive IRQ-clear.) */
static int lr_get_irq_status(const struct lr20xx_config *cfg, uint32_t *irq)
{
	return lr_get_and_clear_irq(cfg, irq);
}

static int lr_calibrate(const struct lr20xx_config *cfg, uint8_t blocks)
{
	uint8_t p[1] = { blocks };
	return lr_cmd(cfg, LR20XX_OP_CALIBRATE, p, 1, NULL, 0);
}

static int lr_calibrate_fe(const struct lr20xx_config *cfg)
{
	/* CalibFe 0x0123 — 3 fixed 16-bit freq entries (6 bytes).
	 * The working LR2021 reference calibrates exactly these three
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

/* ── Dual-band helpers (L1-U1: HF 2.4 GHz path selection) ───────────── */
/* RadioLib LR2021 band split: above 1500 MHz → HF (2.4 GHz ISM / S-band). */
static bool lr_is_hf(uint32_t freq_hz)
{
	return freq_hz > LR20XX_LF_CUTOFF_HZ;
}

/* Clamp TX power to the HF chip max (+12 dBm — RadioLib checkOutputPower HF). */
static int8_t lr_clamp_hf_power(int8_t power_dbm)
{
	return (power_dbm > 12) ? 12 : power_dbm;
}

/* HF PA hf_duty field = paOptTableHf duty + LR20XX_PA_HF_DUTY_UNUSED
 * (RadioLib setOutputPower HF path; verified values for our usable range). */
static uint8_t lr_pa_hf_duty_for_power(int8_t power_dbm)
{
	uint8_t duty;
	switch (power_dbm) {
	case 8:  duty = 15; break;
	case 9:  duty = 14; break;
	case 10: duty = 14; break;
	case 11: duty = 10; break;
	case 12: duty = 0;  break;
	default: duty = 14; break;   /* < +8 dBm → nearest valid (conservative) */
	}
	return (uint8_t)(duty + LR20XX_PA_HF_DUTY_UNUSED);
}

/* RX path + boost for the configured frequency (sub-GHz vs 2.4 GHz). */
static void lr_rx_path_for_freq(uint32_t freq_hz, bool boost,
				uint8_t *path, uint8_t *boost_val)
{
	if (lr_is_hf(freq_hz)) {
		*path = LR20XX_RX_PATH_HF;
		*boost_val = boost ? LR20XX_RX_BOOST_HF : LR20XX_RX_BOOST_NONE;
	} else {
		*path = LR20XX_RX_PATH_LF;
		*boost_val = boost ? LR20XX_RX_BOOST_LF : LR20XX_RX_BOOST_NONE;
	}
}

/* Set RX path+boost from the cached modem config. */
static void lr_apply_rx_path(const struct lr20xx_data *data,
			     const struct lr20xx_config *cfg)
{
	uint8_t path, boost_val;
	lr_rx_path_for_freq(data->modem_cfg.frequency,
			    data->rx_boost_enabled, &path, &boost_val);
	lr_set_rx_path(cfg, path, boost_val);
}

/* Set PA config + TX params for the configured frequency/band.
 * LF: duty/slices from the verified LF table; HF: duty from paOptTableHf,
 * power clamped to +12 dBm.  (RadioLib parity: setOutputPower applies the
 * per-band PA table and power range.) */
static void lr_apply_pa_for_freq(uint32_t freq_hz, int8_t *power_dbm,
				 const struct lr20xx_config *cfg)
{
	if (lr_is_hf(freq_hz)) {
		*power_dbm = lr_clamp_hf_power(*power_dbm);
		lr_set_pa_cfg(cfg, LR20XX_PA_SEL_HF, LR20XX_PA_LF_MODE_FSM,
			      LR20XX_PA_LF_DUTY_UNUSED,
			      LR20XX_PA_LF_SLICES_UNUSED,
			      lr_pa_hf_duty_for_power(*power_dbm));
	} else {
		uint8_t duty = 0x04, slices = 0x01;
		lr_pa_cfg_for_power(*power_dbm, &duty, &slices);
		lr_set_pa_cfg(cfg, LR20XX_PA_SEL_LF, LR20XX_PA_LF_MODE_FSM,
			      duty, slices, LR20XX_PA_HF_DUTY_UNUSED);
	}
	/* Half-dBm units (RadioLib: power * 2) */
	lr_set_tx_params(cfg, (int8_t)(*power_dbm * 2), LR20XX_RAMP_48_US);
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

	/* RadioLib modSetup/config order.  NOTE: the reference firmware sets
	 * lora.XTAL=true — the Wio-LR2021 TCXO is always-on and
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

	lr_apply_rx_path(data, cfg);
	data->rx_boost_applied = data->rx_boost_enabled;

	lr_set_lora_mod_params(cfg, (uint8_t)mc->datarate,
			       lr_bw_to_code(mc->bandwidth),
			       (uint8_t)mc->coding_rate,
			       lr_ldro_for((uint8_t)mc->datarate, mc->bandwidth));

	lr_set_lora_syncword(cfg, mc->public_network ? 0x34 : 0x12);

	if (tx_mode) {
		int8_t power = mc->tx_power;
		lr_apply_pa_for_freq(mc->frequency, &power, cfg);
	}

	lr_set_dio_irq_cfg(cfg, cfg->irq_dio_num, LR20XX_DIO_IRQ_MASK);

	/* LoRa packet params LAST, immediately before SetRx — RadioLib
	 * startReceiveCommon parity.  The LR2021's PayloadLen acts as the
	 * max accepted RX length in explicit mode (0=any, xx=1..xx per
	 * datasheet); writing it before SetSyncword/SetDioIrqConfig left
	 * it reverted to a small default, so every incoming packet was
	 * truncated to a few bytes (observed: 6-byte REQUESTs, no payload,
	 * app showing 0.0dB) — the RadioLib issue #1804 failure mode. */
	lr_set_lora_pkt_params(cfg, mc->preamble_len, 255,
			       LR20XX_PKT_EXPLICIT,
			       mc->packet_crc_disable ? LR20XX_CRC_DISABLED
						      : LR20XX_CRC_ENABLED,
			       mc->iq_inverted ? LR20XX_IQ_INVERTED
					       : LR20XX_IQ_STANDARD);
}

/* ── Start / restart RX ─────────────────────────────────────────────── */

static void lr20xx_start_rx(struct lr20xx_data *data)
{
	const struct lr20xx_config *cfg = data->dev->config;

	lr_set_standby(cfg, LR20XX_STDBY_RC);
	lr_clear_irq(cfg, LR20XX_IRQ_ALL_MASK);
	lr_cmd(cfg, LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);

	/* RadioLib LR2021 stageMode(RX) parity — the ONLY per-rearm work:
	 * RX path + gain, DIO IRQ mapping, max-RX-length restore, then
	 * SetRx.  Frequency/modparams/syncword/FE calibration are applied
	 * once in lora_config() and on AGC reset (lr20xx_reset_agc), NOT
	 * here — the old apply_modem_config() cost a 50 ms FE-calibration
	 * sleep on every TX->RX transition (a deaf window that let the
	 * peer's back-to-back ACK+msg accumulate in the FIFO). */
	lr_apply_rx_path(data, cfg);
	data->rx_boost_applied = data->rx_boost_enabled;

	lr_set_dio_irq_cfg(cfg, cfg->irq_dio_num, LR20XX_DIO_IRQ_MASK);

	/* Explicit header mode must carry pld_len=255 unconditionally —
	 * the TX path leaves PayloadLen at the transmitted size, which
	 * the receiver treats as the max accepted payload (RadioLib issue
	 * #1804).  Written LAST so no later write reverts it. */
	lr_set_lora_pkt_params(cfg, data->modem_cfg.preamble_len, 255,
			       LR20XX_PKT_EXPLICIT,
			       data->modem_cfg.packet_crc_disable
				       ? LR20XX_CRC_DISABLED
				       : LR20XX_CRC_ENABLED,
			       data->modem_cfg.iq_inverted
				       ? LR20XX_IQ_INVERTED
				       : LR20XX_IQ_STANDARD);

	lr_clear_irq(cfg, LR20XX_IRQ_ALL_MASK);

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

	/* Edge-race window fix (re-poll DIO8 after RX re-arm): if a packet
	 * or a noise IRQ
	 * toggled DIO8 HIGH during the 3 ms msleep above while the DIO work
	 * item was already running (k_work_submit drops with -EALREADY), the
	 * rising edge is consumed by the lr_clear_irq above and the packet
	 * sits unread in the FIFO with no IRQ to service it. Re-poll the
	 * GPIO and re-submit by hand so the handler re-reads the chip IRQ
	 * register and picks up the pending frame. */
	if (gpio_pin_get_dt(&cfg->dio1)) {
		LOG_INF("start_rx: DIO8 still HIGH — re-arming RX by hand");
		k_work_submit_to_queue(&data->dio1_wq, &data->dio1_work);
	}
}

static void lr20xx_restart_rx(struct lr20xx_data *data)
{
	const struct lr20xx_config *cfg = data->dev->config;

	/* RadioLib stageMode(RX) parity: RX path + gain first. */
	lr_apply_rx_path(data, cfg);
	data->rx_boost_applied = data->rx_boost_enabled;

	/* Re-apply the RX packet params before EVERY SetRx (RadioLib
	 * startReceiveCommon parity).  LR2021 quirk: the TX path leaves
	 * PayloadLen at the transmitted size, which the receiver treats as
	 * the max accepted payload — after a short TX the chip goes deaf
	 * for larger packets (RadioLib issue #1804).  Explicit header mode
	 * must carry pld_len=255 unconditionally.  Written LAST (after the
	 * DIO IRQ mask) so no later write reverts it. */
	lr_set_dio_irq_cfg(cfg, cfg->irq_dio_num, LR20XX_DIO_IRQ_MASK);

	lr_set_lora_pkt_params(cfg, data->modem_cfg.preamble_len, 255,
			       LR20XX_PKT_EXPLICIT,
			       data->modem_cfg.packet_crc_disable
				       ? LR20XX_CRC_DISABLED
				       : LR20XX_CRC_ENABLED,
			       data->modem_cfg.iq_inverted
				       ? LR20XX_IQ_INVERTED
				       : LR20XX_IQ_STANDARD);

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

/* ── RX diagnostics (debug build): raw reads for live debugging ─────── */
/* Called from the RX failure paths while holding the SPI mutex.  Logs the
 * raw GetRxPktLength / FIFO / GetLoRaPktStatus / GetErrors responses so a
 * malformed or misconfigured peer packet can be identified from the serial
 * log without a logic analyzer. */
static void lr_dump_rx_diag(const struct lr20xx_config *cfg)
{
	uint8_t raw[70] = { 0 };
	int ret;

	ret = lr_cmd(cfg, LR20XX_OP_GET_RX_PACKET_LENGTH, NULL, 0, raw, 4);
	LOG_INF("RX diag: GetRxPktLength rc=%d raw=%02x %02x %02x %02x",
		ret, raw[0], raw[1], raw[2], raw[3]);

	memset(raw, 0, sizeof(raw));
	ret = lr_fifo_read(cfg, LR20XX_OP_READ_RX_FIFO, raw, 64);
	LOG_INF("RX diag: FIFO rc=%d: %02x %02x %02x %02x %02x %02x %02x %02x "
		"%02x %02x %02x %02x %02x %02x %02x %02x",
		ret, raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6],
		raw[7], raw[8], raw[9], raw[10], raw[11], raw[12], raw[13],
		raw[14], raw[15]);

	memset(raw, 0, sizeof(raw));
	ret = lr_cmd(cfg, LR20XX_OP_GET_LORA_PKT_STATUS, NULL, 0, raw, 8);
	LOG_INF("RX diag: PktStatus rc=%d: %02x %02x %02x %02x %02x %02x %02x %02x",
		ret, raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6],
		raw[7]);

	memset(raw, 0, sizeof(raw));
	ret = lr_cmd(cfg, LR20XX_OP_GET_RSSI_INST, NULL, 0, raw, 4);
	LOG_INF("RX diag: RssiInst rc=%d raw=%02x %02x %02x %02x (sig=%d dBm)",
		ret, raw[0], raw[1], raw[2], raw[3],
		(ret == 0) ? -(int)raw[2] : 0);

	memset(raw, 0, sizeof(raw));
	ret = lr_cmd(cfg, LR20XX_OP_GET_ERRORS, NULL, 0, raw, 4);
	LOG_INF("RX diag: GetErrors rc=%d raw=%02x %02x %02x %02x",
		ret, raw[0], raw[1], raw[2], raw[3]);
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
	/* LR2021: GetAndClearIrq atomically reads + clears ALL flags.
	 * The saved irq value drives dispatch; FIFO and packet-length
	 * registers survive the clear.  No downstream lr_clear_irq needed. */
	int rc = lr_get_irq_status(cfg, &irq);

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
		uint16_t pkt_len_raw = 0;
		uint8_t st_len = 0;
		int16_t rssi = 0, rssi_signal = 0;
		int8_t snr = 0;

		/* Order matters — observed on real hardware (LR2021_RADIO_STATUS.md
		 * §22.2/§23): the FIRST status command issued after RX_DONE returns
		 * a STALE value (IRQ-word echo: st_len == (irq>>16), 7/7 in the
		 * 2026-08-01 captures), the SECOND returns the real packet status.
		 * Pre-433499d order (GetRxPktLength first → echo, GetLoRaPacketStatus
		 * second → real st_len=22) delivered inbound packets; the 433499d
		 * reorder (status first) fed the echo into st_len=4/6 and every
		 * packet was truncated + rejected ("incomplete packet").
		 *
		 * 2026-08-02 — MeshCore/RadioLib parity (sandbox-verified,
		 * tools/lr2021_sim test_meshcore_flow_burst): GetLoRaPacketStatus
		 * reports the LAST-COMPLETED packet length (22 for an [ACK 8][msg 22]
		 * burst) while GetRxPktLength resp[3] reports the REMAINING FIFO
		 * total (30). Reading only st_len truncates the burst — the 05806cf
		 * split then recovered 14 B messages with 2 B of data that could
		 * never decrypt (boot_log_splitfix.txt: 9/9 split recoveries failed
		 * "no peer could decrypt"). A second GetRxPktLength (third read,
		 * echo already consumed) yields the real remaining-FIFO total —
		 * the read length authority; rssi/snr still come from the real
		 * GetLoRaPacketStatus (second read). */
		lr_get_rx_packet_length(cfg, &pkt_len);
		pkt_len_raw = pkt_len;

		lr_get_lora_pkt_status(cfg, &st_len, &rssi, &rssi_signal, &snr);

		lr_get_rx_packet_length(cfg, &pkt_len);
		pkt_len &= 0xFF;   /* resp[3] = remaining FIFO total (probed live) */
		if (pkt_len == 0) {
			pkt_len = st_len;   /* fallback: last-packet length */
		}

		if (pkt_len > 0 && pkt_len <= 255) {

				/* Read exactly the remaining-FIFO total reported by
				 * GetRxPktLength (third read, echo already consumed).
				 * Reading fixed 64 B was pulling FIFO garbage after the
				 * real frame, causing MeshCore parser to reject packets
				 * as "incomplete" or "unsupported version". */
				lr_fifo_read(cfg, LR20XX_OP_READ_RX_FIFO,
					     data->rx_buf, pkt_len);

				/* Upstream parity (RadioLib LR11x0::readData): clear
				 * the Rx buffer AFTER the read. Without this the NEXT
				 * packet's GetLoRaPacketStatus reports the leftover FIFO
				 * total (22/38 for a back-to-back ACK+msg burst) and the
				 * whole burst is read as ONE frame — bundled messages
				 * swallowed. RadioLib does exactly this between readData()
				 * and the next startReceive(). */
				lr_cmd(cfg, LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);

				/* Restart RX BEFORE the diagnostic log / callback: the
				 * critical read→re-arm window must stay as short as
				 * possible so the peer's next back-to-back packet (its
				 * own RX_DONE) is serviced promptly. The log pump below
				 * (and the app callback) cannot delay the next packet's
				 * FIFO read — the chip is already back in RX. */
				LOG_DBG("restart_rx: RX_DONE-success");
				lr20xx_restart_rx(data);
				rx_restarted = true;

				/* When SNR < 0, use signal RSSI for weak links */
				if (snr < 0 && rssi_signal > rssi) {
					rssi = rssi_signal;
				}

				k_mutex_unlock(&data->spi_mutex);

				/* Diag (outside the SPI critical section): raw 16-bit
				 * length vs pkt-status length and the first payload
				 * bytes (truncation check). */
				LOG_INF("RX ok: raw_len=%u st_len=%u rssi=%d "
					"sig=%d snr=%d data=%02x %02x %02x %02x "
					"%02x %02x %02x %02x %02x %02x %02x %02x "
					"%02x %02x %02x %02x %02x %02x %02x %02x "
					"%02x %02x %02x %02x %02x %02x %02x %02x "
					"%02x %02x %02x %02x",
					pkt_len_raw, st_len, rssi, rssi_signal, snr,
					data->rx_buf[0], data->rx_buf[1],
					data->rx_buf[2], data->rx_buf[3],
					data->rx_buf[4], data->rx_buf[5],
					data->rx_buf[6], data->rx_buf[7],
					data->rx_buf[8], data->rx_buf[9],
					data->rx_buf[10], data->rx_buf[11],
					data->rx_buf[12], data->rx_buf[13],
					data->rx_buf[14], data->rx_buf[15],
					data->rx_buf[16], data->rx_buf[17],
					data->rx_buf[18], data->rx_buf[19],
					data->rx_buf[20], data->rx_buf[21],
					data->rx_buf[22], data->rx_buf[23],
					data->rx_buf[24], data->rx_buf[25],
					data->rx_buf[26], data->rx_buf[27],
					data->rx_buf[28], data->rx_buf[29],
					data->rx_buf[30], data->rx_buf[31]);

				if (data->async_rx_cb) {
					data->async_rx_cb(data->dev, data->rx_buf,
							  (uint8_t)pkt_len,
							  rssi, snr,
							  data->async_rx_user_data);
				}
				return;
		}

		LOG_WRN("RX: invalid len (raw=%d st=%d)", pkt_len_raw, st_len);
		lr_dump_rx_diag(cfg);
		LOG_DBG("restart_rx: invalid-len");
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

		LOG_DBG("start_rx: TX_DONE-handler");
		lr20xx_start_rx(data);
		rx_restarted = true;

		if (data->tx_signal) {
			k_poll_signal_raise(data->tx_signal, 0);
			data->tx_signal = NULL;   /* poll path won't double-raise */
		}
	}

	/* ── Timeout ── */
	if ((irq & LR20XX_IRQ_TIMEOUT) && !data->tx_active) {
		LOG_DBG("Timeout IRQ — restarting RX");
		LOG_DBG("restart_rx: TIMEOUT");
		lr20xx_restart_rx(data);
		rx_restarted = true;
	}

	/* ── CRC / Header error: drop FIFO residue, restart, notify ── */
	if (irq & (LR20XX_IRQ_CRC_ERROR | LR20XX_IRQ_LORA_HEADER_ERROR)) {
		LOG_WRN("RX error: CRC=%d HDR=%d RXDONE=%d",
			(irq & LR20XX_IRQ_CRC_ERROR) ? 1 : 0,
			(irq & LR20XX_IRQ_LORA_HEADER_ERROR) ? 1 : 0,
			(irq & LR20XX_IRQ_RX_DONE) ? 1 : 0);

		lr_dump_rx_diag(cfg);

		lr_cmd(cfg, LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);

		if (!data->tx_active) {
			LOG_DBG("restart_rx: CRC-HDR-error");
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
		uint32_t actionable = irq & ~LR20XX_IRQ_NOISE_MASK;
		if (actionable == 0) {
			LOG_DBG("DIO1 noise IRQ 0x%08x — ignoring", irq);
		} else {
			LOG_WRN("DIO1 safety: no IRQ handled (0x%08x rc=%d), restarting RX",
				irq, rc);
			LOG_DBG("restart_rx: safety-check");
			lr20xx_restart_rx(data);
		}
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
			/* HW reset wiped freq/modparams/syncword — the
			 * lean start_rx() no longer re-applies them, so
			 * restore the cached config explicitly. */
			lr20xx_apply_modem_config(data, false);
			LOG_WRN("start_rx: DIO-stuck-HW-reset");
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

	/* Apply the full modem config to the chip NOW (RadioLib LR2021
	 * begin()/setParams parity).  Frequency, modem params, syncword,
	 * RX path, DIO mapping and packet params are applied once per
	 * config change — lr20xx_start_rx()/restart_rx() no longer
	 * re-apply them on every re-arm (upstream stageMode(RX) never
	 * touches frequency/modparams).  The old per-rearm apply cost a
	 * 50 ms FE-calibration sleep on every TX->RX transition — a deaf
	 * window during which the peer's back-to-back ACK+msg piled up in
	 * the FIFO (the bundle problem).  FE calibration lives here and in
	 * lr20xx_reset_agc (upstream doResetAGC parity). */
	lr20xx_apply_modem_config(data, config->tx);

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
	const struct lr20xx_config *cfg = dev->config;
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
	if (ret < 0) {
		data->cad_active = false;
		k_mutex_unlock(&data->spi_mutex);
		return ret;
	}

	/* Poll for CAD_DONE (RadioLib-style) instead of relying on the DIO8
	 * edge — a stray edge or the CAD-start edge can consume the CAD_DONE
	 * edge, so the work handler sometimes never runs and the semaphore
	 * wait times out (-116).  Holding the SPI mutex during the poll is
	 * safe (no concurrent radio access; same as the TX poll). */
	int64_t cad_start = k_uptime_get();
	int64_t cad_timeout_ms = (timeout.ticks == K_TICKS_FOREVER)
		? 5000 : (int64_t)k_ticks_to_ms_ceil64(timeout.ticks);
	uint32_t cad_irq = 0;
	bool cad_done = false;
	while ((k_uptime_get() - cad_start) < cad_timeout_ms) {
		if (lr_get_and_clear_irq(cfg, &cad_irq) == 0 &&
		    (cad_irq & LR20XX_IRQ_CAD_DONE)) {
			cad_done = true;
			break;
		}
		k_msleep(2);
	}

	data->cad_active = false;

	if (cad_done) {
		data->cad_result = (cad_irq & LR20XX_IRQ_CAD_DETECTED) ? 1 : 0;
	} else {
		data->cad_result = -ETIMEDOUT;
		LOG_WRN("CAD poll TIMEOUT (last irq=0x%08x)", cad_irq);
	}

	LOG_INF("cad result: %d", data->cad_result);

	k_mutex_unlock(&data->spi_mutex);
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
				LOG_DBG("start_rx: LBT-busy-restore");
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

	/* TX-specific config only (RadioLib LR11x0::startTransmit parity:
	 * PA table, TX params, packet length, FIFO write).  Frequency,
	 * modem params and syncword were applied by lora_config() — the
	 * old apply_modem_config(tx=true) re-applied them with a 50 ms
	 * FE-calibration sleep and held the SPI mutex past TX end,
	 * delaying RX re-arm after every transmission. */
	{
		int8_t power = data->modem_cfg.tx_power;
		lr_apply_pa_for_freq(data->modem_cfg.frequency, &power, cfg);
	}

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

	/* Release the SPI mutex for the TX duration (MeshCore/RadioLib
	 * parity: startTransmit() returns immediately).  The DIO work
	 * handler — priority 0 — services TX_DONE via its own branch
	 * (start_rx + signal raise); the poll below is the edge-loss
	 * fallback that historically made TX reliable.  RX servicing is
	 * never blocked while we wait. */
	k_mutex_unlock(&data->spi_mutex);

	/* Poll for TX_DONE (fallback for a lost DIO edge — the edge fired
	 * at TX start and was consumed, so TX_DONE never woke the work
	 * handler and TX always timed out).  Exit early when the DIO
	 * handler already consumed TX_DONE (tx_active cleared by it). */
	int64_t tx_start = k_uptime_get();
	uint32_t irq = 0;
	bool handled_by_poll = false;
	while ((k_uptime_get() - tx_start) < 6000) {
		if (!data->tx_active) {
			break;   /* DIO handler already did TX_DONE + re-arm */
		}
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		if (data->tx_active &&
		    lr_get_and_clear_irq(cfg, &irq) == 0 &&
		    (irq & LR20XX_IRQ_TX_DONE)) {
			data->tx_active = false;
			handled_by_poll = true;
		}
		k_mutex_unlock(&data->spi_mutex);
		if (handled_by_poll || !data->tx_active) {
			break;
		}
		k_msleep(2);
	}

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	if (data->tx_active) {
		LOG_WRN("TX done poll TIMEOUT (last irq=0x%08x)", irq);
		data->tx_active = false;
		handled_by_poll = true;   /* wake async waiters */
	} else if (handled_by_poll) {
		LOG_INF("TX done (polled, irq=0x%08x)", irq);
	} else {
		LOG_INF("TX done (DIO handler)");
	}

	/* Back to RX — unless the DIO handler's TX_DONE branch already
	 * re-armed (then in_rx_mode is true; a second start_rx would
	 * only re-run the same re-arm). */
	if (!data->in_rx_mode) {
		LOG_DBG("start_rx: TX-poll-path");
		lr20xx_start_rx(data);
	}

	if (handled_by_poll && data->tx_signal) {
		k_poll_signal_raise(data->tx_signal, 0);
		data->tx_signal = NULL;
	}

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

	LOG_DBG("start_rx: recv-entry");
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
		uint8_t path, boost_val;
		lr_rx_path_for_freq(data->modem_cfg.frequency,
				    data->rx_boost_enabled, &path, &boost_val);
		lr_set_rx_path(dev->config, path, boost_val);
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
		uint8_t path, boost_val;
		lr_rx_path_for_freq(data->modem_cfg.frequency, true,
				    &path, &boost_val);
		lr_set_rx_path(cfg, path, boost_val);
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
			   K_PRIO_PREEMPT(0), NULL);
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
