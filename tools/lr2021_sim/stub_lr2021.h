/*
 * SPDX-License-Identifier: MIT
 *
 * stub_lr2021.h — Sandbox stub for Semtech LR2021 + Zephyr SPI loopback.
 *
 * Purpose:
 *   Test the lr20xx driver command-level logic (lr_cmd viengunge NSS,
 *   lr_get_irq_status, lr_get_rx_packet_length, etc.) WITHOUT real hardware,
 *   flash, or CI. Runs as plain C unit tests on the development host
 *   (Linux / Windows / macOS).
 *
 * What this stub models:
 *   - Single SPI bus with two buffers: MOSI (MCU -> radio) and MISO
 *     (radio -> MCU). Each `stub_spi_transceive_dt` call consumes one SPI
 *     transaction's worth of bytes.
 *   - LR2021's "single-NSS" SPI behaviour: status word [stat16] arrives
 *     during the opcode phase; the response data follows immediately
 *     without CS toggling. (This is the rule lr_cmd()'s single-NSS branch
 *     relies on; see lr20xx_lora.c.)
 *   - LR2021 IRQ word accumulation: stub records each command's effect
 *     and fires DIO interrupt when appropriate (RX_DONE, TX_DONE, etc.).
 *   - Fake LoRa packet buffer: caller can `stub_inject_packet()` to
 *     simulate a received frame, then trigger RX_DONE IRQ.
 *
 * What this stub does NOT model:
 *   - Real radio timing, RF path, signal-RSSI accuracy.
 *   - LR2021 FIFO level wraparound beyond 255 bytes.
 *   - CAD, ranging, FHSS noise IRQs.
 *
 * The driver under test (DUT) is a near-verbatim, but Zephyr-free, copy
 * of the lr20xx_lora.c command layer (`driver_under_test.c`). This lets
 * the harness compile with a plain C compiler and link against the stub.
 */
#ifndef STUB_LR2021_H
#define STUB_LR2021_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ── Constants from datasheet §5 / RadioLib LR2021_commands.h ─────────── */

#define LR20XX_OP_GET_STATUS          0x0100
#define LR20XX_OP_GET_VERSION         0x0101
#define LR20XX_OP_GET_ERRORS          0x0110
#define LR20XX_OP_CLEAR_IRQ           0x0116
#define LR20XX_OP_GET_AND_CLEAR_IRQ   0x0117
#define LR20XX_OP_SET_DIO_IRQ_CFG     0x0115
#define LR20XX_OP_SET_DIO_FUNC        0x0112
#define LR20XX_OP_SET_STANDBY         0x0104
#define LR20XX_OP_SET_RX              0x020C
#define LR20XX_OP_SET_TX              0x020D
#define LR20XX_OP_GET_RSSI_INST       0x020B
#define LR20XX_OP_GET_RX_PACKET_LENGTH 0x0212
#define LR20XX_OP_GET_LORA_PKT_STATUS 0x022A
#define LR20XX_OP_GET_OOK_PKT_STATUS  0x0287
#define LR20XX_OP_GET_OOK_RX_STATS    0x0286
#define LR20XX_OP_GET_RX_FIFO_LEVEL   0x011C
#define LR20XX_OP_CLEAR_RX_FIFO       0x011E
#define LR20XX_OP_READ_RX_FIFO        0x0001
/* Sniffer/band-switch opcodes (lockstep with lr20xx_lora.c) */
#define LR20XX_OP_SET_RF_FREQUENCY    0x0200
#define LR20XX_OP_SET_PKT_TYPE        0x0207
#define LR20XX_OP_SET_RX_PATH         0x0201
#define LR20XX_OP_CALIBRATE_FRONT_END 0x0123
#define LR20XX_OP_SET_LORA_MOD_PARAMS 0x0220
#define LR20XX_OP_SET_LORA_SYNCWORD   0x0223
#define LR20XX_OP_SET_LORA_PKT_PARAMS 0x0221
/* WM-BUS opcodes (F1, 2026-09-19 — lockstep with lr20xx_lora.c) */
#define LR20XX_OP_SET_WMBUS_PARAMS    0x026A
#define LR20XX_OP_GET_WMBUS_RX_STATS  0x026C
#define LR20XX_OP_GET_WMBUS_PKT_STATUS 0x026D

#define LR20XX_PKT_TYPE_LORA          0x00   /* spec SetPacketType enum */
#define LR20XX_PKT_TYPE_WMBUS         0x08
#define LR20XX_PKT_TYPE_OOK           0x0A

#define LR20XX_STDBY_RC               0x00
#define LR20XX_STDBY_XOSC             0x01

/* IRQ flags (per datasheet §6 + RadioLib) */
#define LR20XX_IRQ_TX_DONE            (1u << 0)
#define LR20XX_IRQ_RX_DONE            (1u << 1)
#define LR20XX_IRQ_TIMEOUT            (1u << 2)
#define LR20XX_IRQ_CRC_ERROR          (1u << 4)
#define LR20XX_IRQ_LORA_HEADER_VALID  (1u << 6)
#define LR20XX_IRQ_LORA_HEADER_ERROR  (1u << 9)
#define LR20XX_IRQ_CAD_DONE           (1u << 14)
#define LR20XX_IRQ_CAD_DETECTED       (1u << 15)
#define LR20XX_IRQ_ERROR              (1u << 16)
#define LR20XX_IRQ_FHSS               (1u << 25)
#define LR20XX_IRQ_ALL_MASK           0xFFFFFFFFu

/* DIO config */
#define LR20XX_DIO_FUNC_IRQ           0x01
#define LR20XX_DIO_DRIVE_NONE         0x00
#define LR20XX_DIO_8                  8

/* Status word upper byte = "interrupt pending" flag at bit 8. */
#define LR20XX_STATUS_IRQ_PENDING     (1u << 8)

/* ── Stub state ──────────────────────────────────────────────────────── */

#define STUB_SPI_MAX_BYTES   260u  /* opcode (2) + payload + response <= 255+overhead */
#define STUB_FIFO_SIZE       260u  /* full LoRa packet max + 2 status */

typedef struct {
    /* SPI transaction accounting */
    uint32_t transceive_calls;        /* number of stub_spi_transceive_dt invocations */
    bool     cs_asserted;             /* simulated CS line state after last call */

    /* Latest transaction bytes */
    uint8_t  mosi[STUB_SPI_MAX_BYTES];
    uint8_t  miso[STUB_SPI_MAX_BYTES];
    size_t   mosi_len;
    size_t   miso_len;

    /* LR2021 logical state */
    uint32_t irq_pending;             /* current IRQ register value (read-only via GetAndClear) */
    uint32_t dio_irq_mask;            /* mask from last SetDioIrqCfg */
    uint8_t  dio_function[16];        /* per-DIO pin function setting */
    uint8_t  mode;                    /* 0=STBY_RC, 4=RX, 5=TX (datasheet) */
    uint8_t  pkt_type;                /* last SetPacketType argument (LR20XX_PKT_TYPE_*) */
    bool     dio_pin_high;            /* simulated DIO8 line state (gpio_pin_get_dt) */
    uint16_t rx_buffer_length;        /* last received packet length, set by stub_inject_packet */
    uint16_t rx_status_len;           /* GetLoRaPacketStatus-reported length (0 = use
                                         rx_buffer_length). Models the live chip: for an
                                         [ACK 8][msg 22] burst it reports the LAST packet
                                         (22), while GetRxPktLength reports the FIFO total
                                         (30). */
    uint16_t rx_next_pkt_len;         /* GetRxPktLength/GetRxBufferStatus: FIRST unread packet length */
    uint16_t rx_consumed;             /* bytes already read from the FIFO (offset reporting) */

    /* GetOokRxStats response model (6 payload bytes, RadioLib parity) */
    uint16_t ook_stats_rx;            /* pkt_rx counter */
    uint16_t ook_stats_crc;           /* crc_error counter */
    uint16_t ook_stats_len;           /* len_error counter */

    /* GetRssiInst response model: 9-bit raw (RadioLib parity), power = -raw/2 dBm */
    uint16_t rssi_inst_raw;

    /* GetWmbusRxStats response model (DS Table 12-4: three u16 BE) */
    uint16_t wmbus_stats_rx;          /* pkt_rx counter */
    uint16_t wmbus_stats_crc;         /* pkt_crc_error counter */
    uint16_t wmbus_stats_len;         /* LenError counter */

    /* GetWmbusPacketStatus response model (DS Table 12-6, 11 bytes) */
    uint8_t  wmbus_l_field;           /* L-field (demodulated length) */
    uint16_t wmbus_pkt_len;           /* 0 = follow rx_status_len/rx_buffer_length */
    uint16_t wmbus_rssi_avg_raw9;     /* 9-bit raw, power = -raw/2 dBm */
    uint16_t wmbus_rssi_sync_raw9;
    uint32_t wmbus_crc_mask;          /* 17-bit per-CRC failure bitmap */
    uint8_t  wmbus_sw_idx;            /* 0 = format A received, 1 = format B */
    uint8_t  wmbus_lqi;               /* 0.25 dB steps */

    /* Fake RX FIFO contents (the bytes the chip returns on ReadRxFifo) */
    uint8_t  rx_fifo[STUB_FIFO_SIZE];
    size_t   rx_fifo_len;

    /* Failing-injection hooks (test helpers) */
    bool     force_cs_toggle_between_transactions;  /* if true, the read AFTER
                                                       GetAndClearIrq returns the
                                                       IRQ-word echo (one-transaction-
                                                       behind quirk, observed live) */
    uint16_t force_getRxPktLength_return;           /* override GetRxPktLength response */
    bool     echo_next_cmd;                         /* one-shot echo armed by GetAndClearIrq */
    uint32_t last_irq_word;                         /* IRQ word snapshot for the echo */

    /* Counters for diagnostics */
    uint32_t cmd_seen[0x0300];        /* opcode dispatch count (first 0x300 opcodes) */

    /* k_work_submit_to_queue stand-in counter: how many times the DIO
     * work item was (re-)submitted. Models the workqueue side effect of
     * the edge-race re-poll in lr20xx_start_rx. */
    uint32_t work_resubmits;
} stub_lr2021_t;

extern stub_lr2021_t g_stub;

/* ── API for the driver-under-test to use (replaces spi_transceive_dt) ── */

/*
 * Simulate a single SPI transceive. The MOSI bytes (MCU->radio) are
 * copied into g_stub.mosi. The MISO bytes (radio->MCU) are produced
 * by the stub according to the LR2021 protocol rules (single NSS).
 *
 * Pass Zephyr-style struct spi_buf_set pointers:
 *   - tx_set/rx_set point to Zephyr-compatibles with `buffers`/`count`
 *   - either may be NULL to indicate "no buffers"
 * The stub produces [stat16][data...] on the MISO side of any READ
 * command, mimicking what the real chip does.
 *
 * Returns 0 on success, -EINVAL on any structural error.
 */
int stub_spi_transceive_dt(const void *tx_set_, size_t dummy_a,
                           const void *rx_set_, size_t dummy_b);

/* ── Helpers for tests ──────────────────────────────────────────────── */

/* Reset the entire stub to power-on defaults. Call between test cases. */
void stub_reset(void);

/* Inject a fake received packet into the RX FIFO. After this call,
 * a subsequent ReadRxFifo (0x0001) returns these bytes. Sets the
 * internal length so GetRxPktLength (0x0212) returns it. */
void stub_inject_packet(const uint8_t *data, size_t len);

/* Inject a back-to-back bundle: total FIFO bytes, first packet first_len
 * bytes (what GetRxPktLength reports before any read). */

/* Force the stub to behave as if the chip's SPI response is ONE
 * transaction behind (the quirk observed live on the LR2021): the
 * read command issued immediately after GetAndClearIrq returns
 * [stat16][IRQ word...] instead of its proper response. All later
 * reads return real data. */
void stub_set_force_cs_toggle(bool enable);

/* Model the live chip's length split (2026-08-01 captures):
 * GetLoRaPacketStatus reports the LAST-COMPLETED packet length
 * (22 for an [ACK 8][msg 22] burst), GetRxPktLength reports the
 * REMAINING FIFO total (30 for the same burst). Pass 0 to reset
 * (both commands then report rx_buffer_length). */
void stub_set_rx_status_len(uint16_t len);

/* Manually fire RX_DONE IRQ (test convenience). Sets the IRQ pending
 * register and asserts the simulated DIO8 line HIGH. */
void stub_fake_irq_fire_rx_done(void);

/* Manually fire TX_DONE IRQ. */
void stub_fake_irq_fire_tx_done(void);

/* Read the simulated DIO8 line state. */
bool stub_dio_pin_read(void);

/* Force the simulated DIO8 line state (models the chip keeping the line
 * asserted while an unread packet sits in the RX FIFO, even after the
 * IRQ register was cleared by CLEAR_IRQ / GetAndClearIrq). */
void stub_force_dio_pin(bool high);

/* Inspect: how many times has `opcode` (e.g. 0x0100) been issued? */
uint32_t stub_cmd_count(uint16_t opcode);

/* Set the GetOokRxStats response counters (6-byte payload, RadioLib
 * parity: pkt_rx, crc_error, len_error). */
void stub_set_ook_stats(uint16_t pkt_rx, uint16_t crc_error,
                        uint16_t len_error);

/* Set the GetWmbusRxStats response counters (DS Table 12-4:
 * pkt_rx, pkt_crc_error, LenError). */
void stub_set_wmbus_stats(uint16_t pkt_rx, uint16_t crc_error,
                          uint16_t len_error);

/* Set the GetWmbusPacketStatus response fields (DS Table 12-6).
 * pkt_len 0 = follow the rx_status_len/rx_buffer_length length model.
 * RSSIs are 9-bit raw (power = -raw/2 dBm); crc_mask is the 17-bit
 * per-CRC failure bitmap (bit0 = header CRC, format A); sw_idx is
 * 0 = format A / 1 = format B; lqi raw. */
void stub_set_wmbus_status(uint16_t pkt_len, uint16_t rssi_avg_raw9,
                           uint16_t rssi_sync_raw9, uint32_t crc_mask,
                           uint8_t sw_idx, uint8_t lqi, uint8_t l_field);

/* Set the GetRssiInst 9-bit raw response value (power = -raw/2 dBm). */
void stub_set_rssi_inst_raw(uint16_t raw9);

/* Read the chip's current packet type (last SetPacketType argument). */
uint8_t stub_get_pkt_type(void);

#endif /* STUB_LR2021_H */
