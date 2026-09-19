/*
 * SPDX-License-Identifier: MIT
 *
 * stub_lr2021.c — see stub_lr2021.h for design notes.
 */
#include "stub_lr2021.h"
#include <string.h>
#include <stdio.h>
#include <assert.h>

stub_lr2021_t g_stub;

/* ── Tiny helpers ───────────────────────────────────────────────────── */

static uint16_t rd_be16(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }
static void     wr_be16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }

/* ── Public helpers ─────────────────────────────────────────────────── */

void stub_reset(void)
{
    memset(&g_stub, 0, sizeof(g_stub));
    g_stub.mode = LR20XX_STDBY_RC;   /* power-on default = standby-RC */
    g_stub.cs_asserted = true;       /* CS idle (deasserted) */
    /* Default DIO IRQ mask = empty (driver will set) */
    g_stub.force_getRxPktLength_return = 0;  /* sentinel: use rx_buffer_length */
    g_stub.rx_status_len = 0;
}

void stub_inject_packet(const uint8_t *data, size_t len)
{
    assert(len <= sizeof(g_stub.rx_fifo));
    if (len > sizeof(g_stub.rx_fifo)) len = sizeof(g_stub.rx_fifo);
    memcpy(g_stub.rx_fifo, data, len);
    g_stub.rx_fifo_len = len;
    g_stub.rx_buffer_length = (uint16_t)len;
    g_stub.rx_next_pkt_len = (uint16_t)len;
    g_stub.rx_consumed = 0;
}

void stub_set_force_cs_toggle(bool enable)
{
    g_stub.force_cs_toggle_between_transactions = enable;
}

void stub_set_rx_status_len(uint16_t len)
{
    g_stub.rx_status_len = len;
}

void stub_fake_irq_fire_rx_done(void)
{
    g_stub.irq_pending |= LR20XX_IRQ_RX_DONE;
    g_stub.dio_pin_high = true;       /* DIO8 line goes HIGH */
}

void stub_fake_irq_fire_tx_done(void)
{
    g_stub.irq_pending |= LR20XX_IRQ_TX_DONE;
    g_stub.dio_pin_high = true;
}

bool stub_dio_pin_read(void) { return g_stub.dio_pin_high; }

void stub_force_dio_pin(bool high) { g_stub.dio_pin_high = high; }

uint32_t stub_cmd_count(uint16_t opcode) { return g_stub.cmd_seen[opcode]; }

/* ── Response builders ──────────────────────────────────────────────── */

/*
 * Produce the radio-side response for one read command.
 * Writes the bytes that should appear in MISO of a SINGLE-NSS transceive:
 *   [stat16][response_data...]
 *
 * In CS-toggle-injection mode (legacy bug), response_data is replaced by
 * the IRQ word echo, exactly as a real broken chip would.
 */
static void build_response_for_opcode(uint16_t opcode, uint8_t *out, size_t *out_len)
{
    size_t i = 0;

    /* Stat word: bit8 = IRQ pending, lower byte = current mode. */
    uint16_t stat = (uint16_t)((g_stub.irq_pending ? LR20XX_STATUS_IRQ_PENDING : 0) | g_stub.mode);
    wr_be16(&out[i], stat);  i += 2;

    /* One-transaction-behind quirk (observed live): the read issued
     * immediately after GetAndClearIrq gets the IRQ-word echo instead
     * of its real response. Exactly one command is affected. */
    if (g_stub.force_cs_toggle_between_transactions && g_stub.echo_next_cmd) {
        wr_be16(&out[i], (uint16_t)((g_stub.last_irq_word >> 16) & 0xFFFF)); i += 2;
        wr_be16(&out[i], (uint16_t)( g_stub.last_irq_word        & 0xFFFF)); i += 2;
        g_stub.echo_next_cmd = false;
        *out_len = i;
        return;
    }

    switch (opcode) {
    case LR20XX_OP_GET_VERSION: {
        /* 0x0124 in big-endian, plus reserved zero byte. */
        out[i++] = 0x01;
        out[i++] = 0x24;
        out[i++] = 0x00;
        break;
    }
    case LR20XX_OP_GET_RX_PACKET_LENGTH: {
        /* REAL LR2021 layout (pitfall #34, verified live 2026-08-01):
         *   [stat16][stat_byte][len_byte]
         * resp[2] = STATUS byte (0x14 observed), NOT a length — RadioLib's
         * SX126x [len][offset] layout does NOT apply to the LR2021.
         * resp[3] = remaining FIFO total (22 for an 8+14 bundle, 38 for
         * a 22+16 bundle, 8/14 for a single packet).
         * The driver parses (resp[2]<<8)|resp[3] as diagnostic raw_len only;
         * the length authority is GetLoRaPacketStatus st_len (resp[3]). */
        uint16_t v = g_stub.force_getRxPktLength_return
                        ? g_stub.force_getRxPktLength_return
                        : g_stub.rx_buffer_length;
        out[i++] = 0x14;              /* status byte (as observed on HW) */
        out[i++] = (uint8_t)(v & 0xFF); /* length = FIFO total */
        break;
    }
    case LR20XX_OP_GET_LORA_PKT_STATUS: {
        /* Real LR2021 layout (RadioLib LR2021_cmds_lora.cpp):
         *   flags + packetLen + snr + rssi_pkt + rssi_signal + lowbits
         * Driver (lr20xx_lora.c:684-709) parses resp[3] = length,
         * resp[4] = SNR (0.25 dB), resp[5]/[6] = RSSI bytes.
         * Chip returns ZEROS for snr/rssi_pkt (pitfall #35 — app shows
         * "0.0dB"); rssi_signal is real (0x58 -> -44 dBm side-by-side).
         * Live 2026-08-01: for an [ACK 8][msg 22] burst this reports the
         * LAST-COMPLETED packet length (22), while GetRxPktLength
         * reports the remaining FIFO total (30) — rx_status_len models
         * that split (0 = fall back to rx_buffer_length). */
        uint16_t stlen = g_stub.rx_status_len ? g_stub.rx_status_len
                                               : g_stub.rx_buffer_length;
        out[i++] = 0x10;          /* flags: bit4 = CRC ok */
        out[i++] = (uint8_t)(stlen & 0xFF);
        out[i++] = 0x00;          /* snr_pkt  = 0 (chip zeros it) */
        out[i++] = 0x00;          /* rssi_pkt = 0 (chip zeros it) */
        out[i++] = 0x58;          /* rssi_signal raw = 0x58 -> -44 dBm (real) */
        out[i++] = 0x00;          /* low bits + detector */
        break;
    }
    case LR20XX_OP_GET_OOK_PKT_STATUS: {
        /* GetOokPacketStatus 0x0287 (DS.LR2021 §20 / TheClams spec):
         * [stat16][pkt_len u16 BE][rssi_avg low][rssi_high][bits]
         * rssi_avg 9-bit = resp[4] + bit2 of resp[6]; power = -rssi/2 dBm.
         * Length source mirrors the LoRa status read split:
         * rx_status_len (last-completed) override else rx_buffer_length. */
        uint16_t stlen = g_stub.rx_status_len ? g_stub.rx_status_len
                                               : g_stub.rx_buffer_length;
        wr_be16(&out[i], stlen); i += 2;
        out[i++] = 0x5A;   /* rssi_avg low: 0x5A = 90 half-dB → -45 dBm */
        out[i++] = 0x00;   /* rssi_high */
        out[i++] = 0x00;   /* lqi + rssi bit2/bit0 */
        break;
    }
    case LR20XX_OP_GET_AND_CLEAR_IRQ: {
        uint32_t word = g_stub.irq_pending;
        wr_be16(&out[i], (uint16_t)((word >> 16) & 0xFFFF)); i += 2;
        wr_be16(&out[i], (uint16_t)( word        & 0xFFFF)); i += 2;
        /* Side effect: clear the IRQ register. */
        g_stub.irq_pending = 0;
        /* Side effect: drop DIO8 line (ack'd interrupt). */
        g_stub.dio_pin_high = false;
        /* Quirk: when enabled, the NEXT read returns this IRQ word. */
        if (g_stub.force_cs_toggle_between_transactions && word) {
            g_stub.last_irq_word = word;
            g_stub.echo_next_cmd = true;
        }
        break;
    }
    case LR20XX_OP_GET_STATUS: {
        /* Already produced in stat word; no extra response bytes per datasheet. */
        break;
    }
    case LR20XX_OP_GET_RX_FIFO_LEVEL: {
        wr_be16(&out[i], (uint16_t)g_stub.rx_fifo_len); i += 2;
        break;
    }
    case LR20XX_OP_GET_RSSI_INST: {
        /* 2 bytes, signed raw (signal RSSI in 0.5 dB steps) */
        out[i++] = 0xC0;
        out[i++] = 0x00;
        break;
    }
    case LR20XX_OP_GET_ERRORS: {
        out[i++] = 0x00;
        out[i++] = 0x00;
        out[i++] = 0x00;
        out[i++] = 0x00;
        break;
    }
    default:
        /* Unknown opcode — no extra payload, just stat. */
        break;
    }

    *out_len = i;
}

/* ── SPI transceive (the only API for the driver-under-test) ───────── */

int stub_spi_transceive_dt(const void *tx_set_, size_t tx_count_unused,
                           const void *rx_set_, size_t rx_count_unused)
{
    /* Zephyr struct spi_buf_set layout:
     *   struct spi_buf_set { const struct spi_buf *buffers; size_t count; };
     *   struct spi_buf     { void *buf; size_t len; };
     * We receive `tx_set` as `const void *` and re-cast. */
    typedef struct { void *buf; size_t len; } buf_t;
    typedef struct { const buf_t *buffers; size_t count; } buf_set_t;
    const buf_set_t *tx = (const buf_set_t *)tx_set_;
    const buf_set_t *rx = (const buf_set_t *)rx_set_;

    g_stub.transceive_calls++;
    g_stub.mosi_len = 0;
    g_stub.miso_len = 0;

    /* Flatten MOSI */
    for (size_t k = 0; k < tx->count && tx; k++) {
        if (g_stub.mosi_len + tx->buffers[k].len > STUB_SPI_MAX_BYTES) return -EINVAL;
        if (tx->buffers[k].buf && tx->buffers[k].len) {
            memcpy(g_stub.mosi + g_stub.mosi_len, tx->buffers[k].buf, tx->buffers[k].len);
        }
        g_stub.mosi_len += tx->buffers[k].len;
    }

    /* Decode opcode from first 2 MOSI bytes */
    if (g_stub.mosi_len < 2) {
        /* No MOSI supplied — feed rx FIFO bytes back if rx_set present. */
        if (rx && rx->count > 0) {
            size_t want = 0;
            for (size_t k = 0; k < rx->count; k++) want += rx->buffers[k].len;
            if (want > STUB_SPI_MAX_BYTES) want = STUB_SPI_MAX_BYTES;
            size_t available = g_stub.rx_fifo_len;
            size_t give = (want < available) ? want : available;
            for (size_t b = 0; b < give && g_stub.miso_len < STUB_SPI_MAX_BYTES; b++)
                g_stub.miso[g_stub.miso_len++] = g_stub.rx_fifo[b];
            g_stub.rx_fifo_len = (available > give) ? available - give : 0;
        }
        goto fill_rx_bufs;
    }
    uint16_t opcode = rd_be16(g_stub.mosi);
    if (opcode < 0x0300) g_stub.cmd_seen[opcode]++;

    /* Side effects for write commands */
    if (opcode == LR20XX_OP_SET_STANDBY && g_stub.mosi_len >= 3) {
        g_stub.mode = g_stub.mosi[2];
    } else if (opcode == LR20XX_OP_SET_RX) {
        g_stub.mode = 4;  /* RX */
    } else if (opcode == LR20XX_OP_SET_TX) {
        g_stub.mode = 5;  /* TX */
    } else if (opcode == LR20XX_OP_CLEAR_IRQ && g_stub.mosi_len >= 6) {
        /* CLEAR_IRQ: write mask; clear only the flags in the mask. */
        uint32_t mask = ((uint32_t)g_stub.mosi[2] << 24) |
                        ((uint32_t)g_stub.mosi[3] << 16) |
                        ((uint32_t)g_stub.mosi[4] <<  8) |
                        ((uint32_t)g_stub.mosi[5] <<  0);
        g_stub.irq_pending &= ~mask;
        if (g_stub.irq_pending == 0) g_stub.dio_pin_high = false;
    } else if (opcode == LR20XX_OP_CLEAR_RX_FIFO) {
        g_stub.rx_fifo_len = 0;
        g_stub.rx_buffer_length = 0;
    } else if (opcode == LR20XX_OP_READ_RX_FIFO) {
        /* For the FIFO path we put the entire response directly in MISO:
         * the stub "returns" the FIFO contents on the same CS. */
        size_t available = g_stub.rx_fifo_len;
        size_t want = 0;
        if (rx) {
            for (size_t k = 0; k < rx->count; k++) want += rx->buffers[k].len;
        }
        if (want > STUB_SPI_MAX_BYTES) want = STUB_SPI_MAX_BYTES;
        /* Fill MISO with FIFO bytes; truncate if needed. */
        size_t give = (want < available) ? want : available;
        if (g_stub.miso_len + 2 + give > STUB_SPI_MAX_BYTES) give = 0;
        /* FIFO read in LR2021 has NO status prefix; data starts at the
         * current FIFO pointer (advanced by previous reads). */
        for (size_t b = 0; b < give; b++) {
            g_stub.miso[g_stub.miso_len++] = g_stub.rx_fifo[g_stub.rx_consumed + b];
        }
        g_stub.rx_fifo_len = (available > give) ? available - give : 0;
        g_stub.rx_consumed += (uint16_t)give;
        /* After the first packet is consumed, the remainder is delivered
         * as further packets (drain-loop model). */
        g_stub.rx_next_pkt_len = g_stub.rx_fifo_len;
        goto fill_rx_bufs;
    } else if (opcode == LR20XX_OP_SET_DIO_IRQ_CFG && g_stub.mosi_len >= 7) {
        /* byte 2 = DIO pin, bytes 3..6 = mask BE */
        uint8_t pin = g_stub.mosi[2];
        uint32_t mask = ((uint32_t)g_stub.mosi[3] << 24) |
                        ((uint32_t)g_stub.mosi[4] << 16) |
                        ((uint32_t)g_stub.mosi[5] <<  8) |
                        ((uint32_t)g_stub.mosi[6] <<  0);
        if (pin < 16) g_stub.dio_function[pin] = 0x01;  /* IRQ */
        g_stub.dio_irq_mask = mask;
    }

    /* For Read commands: build response in MISO */
    if (rx && rx->count > 0) {
        size_t want_total = 0;
        for (size_t k = 0; k < rx->count; k++) want_total += rx->buffers[k].len;
        if (want_total > STUB_SPI_MAX_BYTES) want_total = STUB_SPI_MAX_BYTES;

        uint8_t tmp[STUB_SPI_MAX_BYTES];
        size_t  tmp_len = 0;
        build_response_for_opcode(opcode, tmp, &tmp_len);
        /* If the driver requested less than tmp_len, we pad with zeros. */
        if (want_total > tmp_len) want_total = tmp_len;
        for (size_t b = 0; b < want_total; b++) {
            if (g_stub.miso_len < STUB_SPI_MAX_BYTES) g_stub.miso[g_stub.miso_len++] = tmp[b];
        }
    }

fill_rx_bufs:
    /* Copy MISO into user's rx bufs */
    {
        size_t pos = 0;
        if (rx) {
            for (size_t k = 0; k < rx->count; k++) {
                if (!rx->buffers[k].buf) continue;
                size_t copy = rx->buffers[k].len;
                if (copy > STUB_SPI_MAX_BYTES - pos) copy = STUB_SPI_MAX_BYTES - pos;
                if (copy) memcpy((void *)rx->buffers[k].buf, g_stub.miso + pos, copy);
                pos += copy;
            }
        }
        g_stub.miso_len = pos;
    }

    /* CS idles high after a transaction. */
    g_stub.cs_asserted = true;
    return 0;
}
