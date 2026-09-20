/*
 * SPDX-License-Identifier: MIT
 *
 * driver_under_test.c — Zephyr-free re-implementation of the LR2021
 * command-level functions from lr20xx_lora.c, exactly as they exist
 * on master HEAD (433499d + local RX-order fix), for testing in stub_lr2021.
 *
 * The bodies are kept IDENTICAL to the real driver where they apply
 * to SPI behaviour. Zephyr-specific glue (mutex, k_work, k_msleep,
 * gpio_pin_get_dt, logging) is replaced with stand-ins or no-ops.
 *
 * ONLY the parts exercised by the test harness are included; nothing
 * else. If you change the real lr20xx_lora.c, update this file in
 * lockstep and re-run the tests.
 */
#include "driver_under_test.h"
#include "stub_lr2021.h"
#include <string.h>
#include <errno.h>

/* ── Module-private scratch buffers (mirrors static buffers in driver) ── */

static uint8_t lr_cmd_buf_tx[70];
static uint8_t lr_cmd_buf_dummy[70];

/* ── The CORE function under test ───────────────────────────────────── */

/*
 * REPRODUCED FROM lr20xx_lora.c (HEAD 433499d).
 * Single-NSS read branch: tx_bufs[0] = opcode+params, tx_bufs[1] = zeros,
 * rx_buf = resp. NO CS toggle between opcode and response phase.
 */
int lr_cmd(uint16_t opcode, const uint8_t *params, size_t param_len,
           uint8_t *resp, size_t resp_len)
{
    /* Guard against sub-stream reads: chip returns at least stat16. */
    if (resp && resp_len > 0 && resp_len < 2) return -EINVAL;

    lr_cmd_buf_tx[0] = (uint8_t)(opcode >> 8);
    lr_cmd_buf_tx[1] = (uint8_t)(opcode >> 0);
    if (params && param_len > 0) {
        memcpy(lr_cmd_buf_tx + 2, params, param_len);
    }

    if (resp && resp_len > 0) {
        memset(lr_cmd_buf_dummy, 0, resp_len - 2);

        struct spi_buf tx_bufs[2] = {
            { .buf = lr_cmd_buf_tx,   .len = 2 + param_len },
            { .buf = lr_cmd_buf_dummy, .len = resp_len - 2 },
        };
        struct spi_buf rx_buf = { .buf = resp, .len = resp_len };
        struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = 2 };
        struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

        return stub_spi_transceive_dt(&tx_set, 1, &rx_set, 1);
    }

    /* Write path */
    struct spi_buf tx_buf = { .buf = lr_cmd_buf_tx, .len = 2 + param_len };
    struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
    return stub_spi_transceive_dt(&tx_set, 1, NULL, 0);
}

/* ── The wrappers under test (current lr20xx_lora.c implementations) ─ */

int lr_get_version(uint8_t *major, uint8_t *minor)
{
    /* Real driver requests 4 bytes and parses from resp[2] (skip the
     * 2-byte status prefix). Matches lr20xx_lora.c:828-842. */
    uint8_t resp[4] = { 0 };
    int rc = lr_cmd(LR20XX_OP_GET_VERSION, NULL, 0, resp, sizeof(resp));
    if (rc) return rc;
    *major = resp[2];
    *minor = resp[3];
    return 0;
}

int lr_clear_irq(uint32_t mask)
{
    uint8_t p[4] = {
        (uint8_t)(mask >> 24), (uint8_t)(mask >> 16),
        (uint8_t)(mask >>  8), (uint8_t)(mask >>  0),
    };
    return lr_cmd(LR20XX_OP_CLEAR_IRQ, p, sizeof(p), NULL, 0);
}

int lr_get_and_clear_irq(uint32_t *irq)
{
    uint8_t resp[6] = { 0 };
    int rc = lr_cmd(LR20XX_OP_GET_AND_CLEAR_IRQ, NULL, 0, resp, sizeof(resp));
    if (rc) return rc;
    if (irq) {
        *irq = ((uint32_t)resp[2] << 24) | ((uint32_t)resp[3] << 16) |
               ((uint32_t)resp[4] <<  8) |  (uint32_t)resp[5];
    }
    return 0;
}

/*
 * Alias used by the dio_work_handler in the real driver. In real life
 * it would call lr_get_and_clear_irq. Kept here for parity.
 */
int lr_get_irq_status(uint32_t *irq) { return lr_get_and_clear_irq(irq); }

int lr_set_standby(uint8_t mode)
{
    uint8_t p[1] = { mode };
    return lr_cmd(LR20XX_OP_SET_STANDBY, p, sizeof(p), NULL, 0);
}

int lr_set_rx(uint32_t timeout_rtc)
{
    uint8_t p[3] = {
        (uint8_t)(timeout_rtc >> 16),
        (uint8_t)(timeout_rtc >>  8),
        (uint8_t)(timeout_rtc >>  0),
    };
    return lr_cmd(LR20XX_OP_SET_RX, p, sizeof(p), NULL, 0);
}

int lr_set_tx(uint32_t timeout_rtc)
{
    uint8_t p[3] = {
        (uint8_t)(timeout_rtc >> 16),
        (uint8_t)(timeout_rtc >>  8),
        (uint8_t)(timeout_rtc >>  0),
    };
    return lr_cmd(LR20XX_OP_SET_TX, p, sizeof(p), NULL, 0);
}

int lr_set_dio_function(uint8_t dio, uint8_t func, uint8_t drive)
{
    uint8_t p[2] = { dio, (uint8_t)((func << 1) | (drive & 0x01)) };
    return lr_cmd(LR20XX_OP_SET_DIO_FUNC, p, sizeof(p), NULL, 0);
}

int lr_set_dio_irq_cfg(uint8_t dio, uint32_t mask)
{
    uint8_t p[5] = {
        (uint8_t)dio,
        (uint8_t)(mask >> 24), (uint8_t)(mask >> 16),
        (uint8_t)(mask >>  8), (uint8_t)(mask >>  0),
    };
    return lr_cmd(LR20XX_OP_SET_DIO_IRQ_CFG, p, sizeof(p), NULL, 0);
}

int lr_get_rx_packet_length(uint16_t *pkt_len)
{
    /* Real driver (lr20xx_lora.c): 0x0212 = [stat16][stat_byte][len_byte],
     * parsed as (resp[2]<<8)|resp[3] — diagnostic raw_len ONLY (the length
     * authority is GetLoRaPacketStatus st_len). The FIRST read after
     * GetAndClearIrq returns the IRQ-word echo instead (one-transaction-
     * behind quirk), which is why this is diagnostic-only. */
    uint8_t resp[4] = { 0 };
    int rc = lr_cmd(LR20XX_OP_GET_RX_PACKET_LENGTH, NULL, 0, resp, sizeof(resp));
    if (rc) return rc;
    if (pkt_len) *pkt_len = (uint16_t)((resp[2] << 8) | resp[3]);
    return 0;
}

int lr_get_lora_packet_status(uint8_t *st_len, int16_t *rssi,
                              int16_t *rssi_signal, int8_t *snr)
{
    /*
     * Mirrors lr20xx_lora.c:684-709: reads 8 bytes, parses
     *   resp[2] = flags (bit4: CRC ok, low nibble: CR)
     *   resp[3] = packet length
     *   resp[4] = SNR (0.25 dB steps, signed)
     *   resp[5] = RSSI packet byte, resp[6] = RSSI signal byte
     *   resp[7] = bit0: signal RSSI LSB, bit1: packet RSSI LSB
     */
    uint8_t resp[8] = { 0 };
    int rc = lr_cmd(LR20XX_OP_GET_LORA_PKT_STATUS, NULL, 0, resp, sizeof(resp));
    if (rc) return rc;
    if (st_len)      *st_len      = resp[3];
    if (rssi)        *rssi        = -((int)(resp[5] << 1) | ((resp[7] >> 1) & 1)) / 2;
    if (rssi_signal) *rssi_signal = -((int)(resp[6] << 1) | (resp[7] & 1)) / 2;
    if (snr)         *snr         = ((int8_t)resp[4]) / 4;
    return 0;
}

int lr_fifo_read(uint8_t *data, size_t len)
{
    /*
     * Real driver: lr_fifo_read takes an opcode, builds [opcode, zeros],
     * calls a single SPI transceive, and accumulates the response into
     * data[] (skipping the 2 opcode/status bytes).
     */
    uint8_t tx_buf[2 + 64] = { 0 };
    tx_buf[0] = 0x00;
    tx_buf[1] = 0x01;  /* LR20XX_OP_READ_RX_FIFO */
    /* tx_len is 2 + len — but stub only needs the 2 opcode bytes. */

    memset(data, 0, len);
    struct spi_buf tx_spi  = { .buf = tx_buf, .len = 2 };       /* opcode only */
    struct spi_buf rx_spi  = { .buf = data,    .len = len };
    struct spi_buf_set tx_set = { .buffers = &tx_spi, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_spi, .count = 1 };
    return stub_spi_transceive_dt(&tx_set, 1, &rx_set, 1);
}

int lr_clear_rx_fifo(void)
{
    return lr_cmd(LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);
}

/* ── start_rx edge-race re-poll (2026-08-02, mirror of the DIO8
 * ── re-check at the end of lr20xx_start_rx) ───────────────────────── */

/*
 * Mirrors the tail of lr20xx_start_rx (lr20xx_lora.c, after the second
 * lr_clear_irq + lr_dump_state): re-poll DIO8 after the IRQ register was
 * cleared. If a packet (e.g. an ACK) or a noise IRQ toggled DIO8 HIGH
 * during the 3 ms re-arm msleep while the DIO work item was already
 * running (k_work_submit drops with -EALREADY), the rising edge is
 * consumed by the clear and the frame sits unread in the FIFO with no
 * IRQ to service it. Re-submit by hand so the handler re-reads the chip
 * IRQ register and picks up the pending frame.
 *
 * Stand-ins (Zephyr glue): gpio_pin_get_dt -> stub_dio_pin_read(),
 * k_work_submit_to_queue -> g_stub.work_resubmits++.
 * Returns 1 when the work item was re-submitted, 0 otherwise.
 */
int lr20xx_start_rx_edge_recheck(void)
{
    if (stub_dio_pin_read()) {
        g_stub.work_resubmits++;   /* k_work_submit_to_queue(&data->dio1_wq,
                                    * &data->dio1_work) */
        return 1;
    }
    return 0;
}

/* ── RX read flows (the two candidates under comparison) ───────────── */

/*
 * CURRENT driver flow (mirrors lr20xx_lora.c dio1 handler, HEAD 05806cf):
 *   GetAndClearIrq → GetRxPktLength (diag; FIRST read = IRQ echo) →
 *   GetLoRaPacketStatus (authoritative st_len) → fifo_read(st_len).
 * For an [ACK 8][msg 22] burst the chip reports st_len = 22 (LAST packet)
 * while the FIFO holds 30 B → reads ACK + first 14 B of the msg → the
 * Dispatcher split recovers a truncated msg that can never decrypt.
 */
int lr_rx_flow_current(uint8_t *data, size_t maxlen, size_t *out_len)
{
    uint32_t irq = 0;
    int rc = lr_get_and_clear_irq(&irq);
    if (rc) return rc;

    uint16_t pkt_len_raw = 0;
    lr_get_rx_packet_length(&pkt_len_raw);   /* first read → IRQ echo (diag) */

    uint8_t st_len = 0;
    int16_t rssi = 0, rssi_signal = 0;
    int8_t snr = 0;
    rc = lr_get_lora_packet_status(&st_len, &rssi, &rssi_signal, &snr);
    if (rc) return rc;

    if (st_len == 0 || st_len > maxlen) return -EINVAL;

    rc = lr_fifo_read(data, st_len);
    if (rc) return rc;
    lr_clear_rx_fifo();
    *out_len = st_len;
    return 0;
}

/*
 * MESH CORE / RadioLib-style flow (candidate fix — RadioLibWrapper
 * recvRaw + LR2021::readData parity):
 *   GetAndClearIrq → GetRxPktLength (FIRST read = IRQ echo, diag) →
 *   GetLoRaPacketStatus (SECOND read = real rssi/snr/st_len) →
 *   GetRxPktLength (THIRD read = REAL: remaining FIFO total) →
 *   fifo_read(remaining) → clear FIFO → clear IRQ.
 * For the burst this reads the WHOLE 30 B (ACK 8 + msg 22) → the
 * Dispatcher split recovers the FULL msg (10 B data → decryptable).
 * This mirrors the real driver exactly (rssi/snr stay real — the echo
 * only ever hits the FIRST read).
 */
int lr_rx_flow_meshcore(uint8_t *data, size_t maxlen, size_t *out_len)
{
    uint32_t irq = 0;
    int rc = lr_get_and_clear_irq(&irq);
    if (rc) return rc;

    uint16_t len16 = 0;
    lr_get_rx_packet_length(&len16);   /* 1st read → IRQ echo (diag) */

    uint8_t st_len = 0;
    int16_t rssi = 0, rssi_signal = 0;
    int8_t snr = 0;
    rc = lr_get_lora_packet_status(&st_len, &rssi, &rssi_signal, &snr);
    if (rc) return rc;

    rc = lr_get_rx_packet_length(&len16);  /* 3rd read → REAL value */
    if (rc) return rc;

    /* Real LR2021 layout: [stat16][0x14 status][len_byte] — resp[3] is
     * the remaining FIFO total (probed live 2026-08-01). For lengths
     * < 256 the low byte is the length under BOTH the [0x14][len] and
     * the [len_hi][len_lo] interpretations. */
    uint8_t len = (uint8_t)(len16 & 0xFF);
    if (len == 0) {
        len = st_len;                  /* fallback: last-packet length */
    }
    if (len == 0 || len > maxlen) return -EINVAL;

    rc = lr_fifo_read(data, len);
    if (rc) return rc;
    lr_clear_rx_fifo();
    lr_clear_irq(LR20XX_IRQ_ALL_MASK);
    *out_len = len;
    return 0;
}

/* ── Dual-band helpers (L1-U1, 2026-08-02) ───────────────────────────
 * Identical bodies to lr20xx_lora.c (constants live in driver_under_test.h).
 * Only the pure logic is mirrored here — the SPI wrappers (lr_set_pa_cfg,
 * lr_set_rx_path) are not exercised by the harness in these tests. */

bool lr_is_hf(uint32_t freq_hz)
{
    return freq_hz > LR20XX_LF_CUTOFF_HZ;
}

int8_t lr_clamp_hf_power(int8_t power_dbm)
{
    return (power_dbm > 12) ? 12 : power_dbm;
}

/* RSSI source selection — identical body to lr20xx_lora.c
 * lr_rssi_effective() (§9.9.9). Keep in lockstep. */
int16_t lr_rssi_effective(int16_t rssi, int16_t rssi_signal)
{
    return (rssi == 0 && rssi_signal < 0) ? rssi_signal : rssi;
}

uint8_t lr_pa_hf_duty_for_power(int8_t power_dbm)
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

void lr_rx_path_for_freq(uint32_t freq_hz, bool boost,
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

/* ── TDM band-switch pure helpers (L3-U4, 2026-08-02) ───────────────
 * Mirrored byte-for-byte from lr20xx_lora.c (pitfall #21).  Deterministic
 * (no chip state), so the sandbox can lock their behaviour before the full
 * lr20xx_switch_band() driver work is built/flashed. */

uint16_t lr_cal_fe_single_bin_hz(uint32_t freq_hz)
{
    uint16_t bin = (uint16_t)((freq_hz / 1000000u + 2u) / 4u);
    return (uint16_t)(bin | (lr_is_hf(freq_hz) ? 0x8000u : 0u));
}

bool lr_band_switch_needs_cal(uint32_t old_hz, uint32_t new_hz)
{
    uint32_t delta = (old_hz > new_hz) ? (old_hz - new_hz)
                                       : (new_hz - old_hz);
    return delta >= 20000000u;
}

/* ── Sniffer OOK poll-mode extension (2026-09-19) ────────────────────
 * Byte-for-byte sequence mirrors of lr20xx_sniffer_ook_arm/_poll in
 * lr20xx_lora.c (mutex/led/log glue replaced with no-ops, device
 * pointers dropped — single-instance DUT).  Keep in lockstep.
 * Opcodes: SetOokModulationParams 0x0281, SetOokPacketParams 0x0282,
 * SetOokSyncWord 0x0284, GetOokPacketStatus 0x0287, SetOokDetector
 * 0x0288, SetRfFrequency 0x0200, SetPacketType 0x0207, SetRxPath
 * 0x0201, CalibrateFrontEnd 0x0123 (spec commands.yaml). */

#define LR20XX_OP_SET_RF_FREQUENCY_DUT   0x0200
#define LR20XX_OP_SET_RX_PATH_DUT        0x0201
#define LR20XX_OP_SET_PKT_TYPE_DUT       0x0207
#define LR20XX_OP_CAL_FE_DUT             0x0123
#define LR20XX_OP_OOK_MOD_PARAMS_DUT     0x0281
#define LR20XX_OP_OOK_PKT_PARAMS_DUT     0x0282
#define LR20XX_OP_OOK_SYNC_WORD_DUT      0x0284
#define LR20XX_OP_OOK_PKT_STATUS_DUT     0x0287
#define LR20XX_OP_OOK_DETECTOR_DUT       0x0288
#define LR20XX_OP_OOK_RX_STATS_DUT       0x0286
#define LR20XX_OP_LORA_MOD_PARAMS_DUT    0x0220
#define LR20XX_OP_LORA_SYNCWORD_DUT      0x0223
#define LR20XX_OP_LORA_PKT_PARAMS_DUT    0x0221
#define LR20XX_OP_SET_LORA_CAD_PARAMS_DUT 0x0227
#define LR20XX_OP_SET_LORA_CAD_DUT        0x0228

/* Frequency cache (mirror of data->modem_cfg.frequency tracking);
 * boot band = 869.618 MHz (NodePrefs default). */
static uint32_t lr_sniffer_cur_freq = 869618000u;

/* Test hook: deterministic starting frequency for cycle tests (the
 * tracker persists across tests otherwise — a leftover 2.4 GHz from a
 * BLE test would fire the >=20 MHz cal on the first 868 hop). */
void lr_sniffer_reset_freq_tracker(void)
{
    lr_sniffer_cur_freq = 869618000u;
}

int lr_set_rf_frequency(uint32_t freq_hz)
{
    uint8_t p[4] = {
        (uint8_t)(freq_hz >> 24), (uint8_t)(freq_hz >> 16),
        (uint8_t)(freq_hz >>  8), (uint8_t)(freq_hz >>  0),
    };
    return lr_cmd(LR20XX_OP_SET_RF_FREQUENCY_DUT, p, sizeof(p), NULL, 0);
}

int lr_set_pkt_type(uint8_t pkt_type)
{
    uint8_t p[1] = { pkt_type };
    return lr_cmd(LR20XX_OP_SET_PKT_TYPE_DUT, p, sizeof(p), NULL, 0);
}

int lr_sniffer_ook_arm(uint32_t freq_hz, uint32_t br_bps,
                       uint8_t rx_bw_code, uint16_t pld_len)
{
    int ret;

    /* Lean housekeeping (parity with lr20xx_switch_band). */
    ret = lr_set_standby(LR20XX_STDBY_RC);
    if (ret) return ret;
    ret = lr_clear_irq(LR20XX_IRQ_ALL_MASK);
    if (ret) return ret;
    ret = lr_cmd(LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);
    if (ret) return ret;

    if (lr_band_switch_needs_cal(lr_sniffer_cur_freq, freq_hz)) {
        uint16_t bin = lr_cal_fe_single_bin_hz(freq_hz);
        uint8_t p[2] = { (uint8_t)(bin >> 8), (uint8_t)(bin & 0xFF) };
        ret = lr_cmd(LR20XX_OP_CAL_FE_DUT, p, 2, NULL, 0);
        if (ret) return ret;
    }

    ret = lr_set_rf_frequency(freq_hz);
    if (ret) return ret;
    ret = lr_set_pkt_type(LR20XX_PKT_TYPE_OOK);
    if (ret) return ret;

    uint8_t mod_p[7] = {
        (uint8_t)(br_bps >> 24), (uint8_t)(br_bps >> 16),
        (uint8_t)(br_bps >>  8), (uint8_t)(br_bps >>  0),
        0x00,                    /* pulse shape NONE */
        rx_bw_code,              /* RX BW code */
        0x00,                    /* ook_depth FULL */
    };
    ret = lr_cmd(LR20XX_OP_OOK_MOD_PARAMS_DUT, mod_p, sizeof(mod_p),
                 NULL, 0);
    if (ret) return ret;

    uint8_t pkt_p[6] = {
        0x00, 0x08,              /* pre_len_tx 8 bits */
        0x00,                    /* addr OFF | FIXED length */
        (uint8_t)(pld_len >> 8), (uint8_t)(pld_len >> 0),
        0x00,                    /* CRC OFF | encoding NONE */
    };
    ret = lr_cmd(LR20XX_OP_OOK_PKT_PARAMS_DUT, pkt_p, sizeof(pkt_p),
                 NULL, 0);
    if (ret) return ret;

    uint8_t sw_p[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    ret = lr_cmd(LR20XX_OP_OOK_SYNC_WORD_DUT, sw_p, sizeof(sw_p),
                 NULL, 0);
    if (ret) return ret;

    uint8_t det_p[5] = { 0x00, 0x02, 0x01, 0x01, 0x00 };
    ret = lr_cmd(LR20XX_OP_OOK_DETECTOR_DUT, det_p, sizeof(det_p),
                 NULL, 0);
    if (ret) return ret;

    lr_sniffer_cur_freq = freq_hz;

    /* RX path + boost (mirror of lr_apply_rx_path; DUT always boosted). */
    {
        uint8_t path, boost;
        lr_rx_path_for_freq(freq_hz, true, &path, &boost);
        uint8_t p[2] = { path, boost };
        ret = lr_cmd(LR20XX_OP_SET_RX_PATH_DUT, p, sizeof(p), NULL, 0);
        if (ret) return ret;
    }

    /* Poll mode: silence DIO IRQ routing. */
    ret = lr_set_dio_irq_cfg(LR20XX_DIO_8, 0);
    if (ret) return ret;

    return lr_set_rx(LR20XX_RX_TIMEOUT_INF);
}

int lr_sniffer_ook_poll(uint8_t *buf, uint16_t cap, uint16_t *out_len,
                        int16_t *rssi_avg_dbm)
{
    *out_len = 0;
    if (rssi_avg_dbm) *rssi_avg_dbm = 0;

    uint32_t irq = 0;
    int ret = lr_get_and_clear_irq(&irq);
    if (ret) return ret;
    if (!(irq & LR20XX_IRQ_RX_DONE)) return 0;

    /* 3-read dance, mirror of lr_rx_flow_meshcore: 1st GetRxPacketLength
     * = IRQ echo (diag; eats the echo), 2nd GetOokPacketStatus = real
     * (st_len + RSSI), 3rd GetRxPacketLength = REAL remaining-FIFO total;
     * packet-status length = zero-fallback.  A single-read poll returns
     * the echo here and silently drops the packet. */
    uint16_t pkt_len_raw = 0;
    lr_get_rx_packet_length(&pkt_len_raw);   /* 1st read → IRQ echo (diag) */
    (void)pkt_len_raw;

    uint16_t st_len = 0;
    {
        uint8_t resp[8] = { 0 };
        if (lr_cmd(LR20XX_OP_OOK_PKT_STATUS_DUT, NULL, 0, resp,
                   sizeof(resp)) == 0) {
            st_len = (uint16_t)((resp[2] << 8) | resp[3]);
            if (rssi_avg_dbm) {
                /* RadioLib getOokPacketStatus parity: 9-bit raw =
                 * (resp[4]<<1) | bit2 of resp[6]; power = -raw/2.
                 * The pre-fix parse folded the 9th bit into bit 8
                 * (half weight) → reported 2× low. */
                uint16_t raw = ((uint16_t)resp[4] << 1) |
                               ((resp[6] >> 2) & 0x01u);
                *rssi_avg_dbm = -(int16_t)((raw + 1) / 2);
            }
        }
    }

    uint16_t pkt_len = 0;
    lr_get_rx_packet_length(&pkt_len);       /* 3rd read → REAL value */
    pkt_len &= 0xFF;   /* resp[3] = remaining FIFO total */
    if (pkt_len == 0) pkt_len = st_len;      /* fallback */
    if (pkt_len > cap) pkt_len = cap;
    if (pkt_len == 0) {
        lr_cmd(LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);
        lr_set_dio_irq_cfg(LR20XX_DIO_8, 0);
        lr_set_rx(LR20XX_RX_TIMEOUT_INF);
        return 0;
    }

    lr_fifo_read(buf, (uint8_t)pkt_len);
    lr_cmd(LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);

    lr_set_dio_irq_cfg(LR20XX_DIO_8, 0);
    lr_set_rx(LR20XX_RX_TIMEOUT_INF);
    *out_len = pkt_len;
    return 0;
}

/* ── OOK stats + RSSI-instant reads (lockstep with lr20xx_lora.c) ──── */

int lr_sniffer_ook_stats(uint16_t *pkt_rx, uint16_t *crc_error,
                         uint16_t *len_error, uint8_t *raw_out)
{
    /* GetOokRxStats (0x0286) — RadioLib parity: 6 payload bytes,
     * [stat16][pkt_rx u16][crc_error u16][len_error u16].  The pre-fix
     * parse read 18 bytes and reported pbl_det/sync_ok/sync_fail from
     * offsets that do not exist in this response (always 0). */
    uint8_t resp[16] = { 0 };
    int ret = lr_cmd(LR20XX_OP_OOK_RX_STATS_DUT, NULL, 0, resp, sizeof(resp));
    if (ret) return ret;
    if (raw_out)   memcpy(raw_out, resp, sizeof(resp));
    if (pkt_rx)    *pkt_rx    = ((uint16_t)resp[2] << 8) | resp[3];
    if (crc_error) *crc_error = ((uint16_t)resp[4] << 8) | resp[5];
    if (len_error) *len_error = ((uint16_t)resp[6] << 8) | resp[7];
    return 0;
}

int lr_get_rssi_inst(int16_t *rssi)
{
    /* GetRssiInst (0x020B) — RadioLib parity: 9-bit raw =
     * (buff[0]<<1) | (buff[1]>>7); power = -raw/2 dBm.  The pre-fix
     * parse -(resp[2]) was 1:1 on the payload MSB (half the value,
     * blind to the 9th bit). */
    uint8_t resp[4] = { 0 };
    int ret = lr_cmd(LR20XX_OP_GET_RSSI_INST, NULL, 0, resp, sizeof(resp));
    if (ret) return ret;
    if (rssi) {
        uint16_t raw = ((uint16_t)resp[2] << 1) | ((resp[3] >> 7) & 0x01u);
        *rssi = -(int16_t)((raw + 1) / 2);
    }
    return 0;
}

/* ── LoRa band switch (mirror of lr20xx_switch_band, lr20xx_lora.c) ──
 * General form: dynamic SF / BW / CR (packing mirrors lr_bw_to_code +
 * lr_ldro_for — LDRO ON when symbol time >= 16 ms).  The sniffer 868
 * leg wrapper fixes 869.618 MHz / SF8 / BW62.5 / CR4/8.  The command
 * ORDER and the packet-type restore are the lockstep surface. */

uint8_t lr_bw_code_dut(uint16_t bw_khz)
{
    /* lr_bw_to_code() parity (BW enum → chip code; DS §9 +
     * commands.yaml: BW_62 = 3, BW_125 = 4, BW_250 = 5, BW_500 = 6).
     * PITFALL: the original fixed-byte mirror wrote 0x05 for BW62.5 —
     * that is the BW_250 code; the driver writes 3. */
    switch (bw_khz) {
    case 7:    return 0;
    case 10:   return 8;
    case 15:   return 1;
    case 20:   return 9;
    case 31:   return 2;
    case 41:   return 10;
    case 62:   return 3;
    case 125:  return 4;
    case 200:  return 13;
    case 250:  return 5;
    case 400:  return 14;
    case 500:  return 6;
    case 800:  return 15;
    case 1000: return 7;
    default:   return 4;
    }
}

uint8_t lr_ldro_dut(uint8_t sf, uint16_t bw_khz)
{
    /* lr_ldro_for() parity: LDRO ON when symbol time >= 16 ms. */
    uint32_t sym_us = ((1u << sf) * 1000000u) / ((uint32_t)bw_khz * 1000u);

    return (sym_us >= 16000u) ? 1u : 0u;
}

int lr_sniffer_switch_band_cfg(uint32_t freq_hz, uint8_t sf,
                               uint16_t bw_khz, uint8_t cr_code)
{
    int ret;

    ret = lr_set_standby(LR20XX_STDBY_RC);
    if (ret) return ret;
    ret = lr_clear_irq(LR20XX_IRQ_ALL_MASK);
    if (ret) return ret;
    ret = lr_cmd(LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);
    if (ret) return ret;

#ifndef LR2021_SIM_OLD_SWITCH_BAND
    /* THE FIX (2026-09-19): restore the LoRa packet type — the OOK arm
     * left the chip in the OOK modem, and without this the post-hop
     * "LoRa" leg delivers OOK-sliced noise as LORA frames. */
    ret = lr_set_pkt_type(LR20XX_PKT_TYPE_LORA);
    if (ret) return ret;
#endif

    if (lr_band_switch_needs_cal(lr_sniffer_cur_freq, freq_hz)) {
        uint16_t bin = lr_cal_fe_single_bin_hz(freq_hz);
        uint8_t p[2] = { (uint8_t)(bin >> 8), (uint8_t)(bin & 0xFF) };
        ret = lr_cmd(LR20XX_OP_CAL_FE_DUT, p, 2, NULL, 0);
        if (ret) return ret;
    }

    ret = lr_set_rf_frequency(freq_hz);
    if (ret) return ret;

    /* SetLoraModParams: p[0]=(sf<<4)|bw_code, p[1]=(cr<<4)|ldro. */
    {
        uint8_t p[2] = { (uint8_t)((sf << 4) | lr_bw_code_dut(bw_khz)),
                         (uint8_t)((cr_code << 4) |
                                   lr_ldro_dut(sf, bw_khz)) };
        ret = lr_cmd(LR20XX_OP_LORA_MOD_PARAMS_DUT, p, sizeof(p), NULL, 0);
        if (ret) return ret;
    }

    /* SetLoraSyncword: 0x12 = private network. */
    {
        uint8_t p[1] = { 0x12 };
        ret = lr_cmd(LR20XX_OP_LORA_SYNCWORD_DUT, p, sizeof(p), NULL, 0);
        if (ret) return ret;
    }

    /* RX path + boost for the band (always boosted in the DUT). */
    {
        uint8_t path, boost;
        lr_rx_path_for_freq(freq_hz, true, &path, &boost);
        uint8_t p[2] = { path, boost };
        ret = lr_cmd(LR20XX_OP_SET_RX_PATH_DUT, p, sizeof(p), NULL, 0);
        if (ret) return ret;
    }

    /* Re-enable DIO IRQ routing (OOK / BLE / WM-BUS arms silenced it). */
    ret = lr_set_dio_irq_cfg(LR20XX_DIO_8, LR20XX_IRQ_ALL_MASK);
    if (ret) return ret;

    /* SetLoraPacketParams: preamble 8, pld_len 255 (max RX), explicit
     * header, CRC ON, IQ standard.  p[2]=255, p[3]=(0<<2)|(1<<1)|0=2. */
    {
        uint8_t p[4] = { 0x00, 0x08, 255, 0x02 };
        ret = lr_cmd(LR20XX_OP_LORA_PKT_PARAMS_DUT, p, sizeof(p), NULL, 0);
        if (ret) return ret;
    }

    lr_sniffer_cur_freq = freq_hz;

    return lr_set_rx(LR20XX_RX_TIMEOUT_INF);
}

/* Sniffer 868 leg wrapper: 869.618 MHz / SF8 / BW62.5 / CR4/8. */
int lr_sniffer_switch_band_lora(uint32_t freq_hz)
{
    return lr_sniffer_switch_band_cfg(freq_hz, 8, 62, 4);
}

/* lr_cad_detect_peak() parity (driver table; LR20xx scale 48-90). */
static uint8_t lr_cad_detect_peak_dut(uint8_t sf)
{
    static const uint8_t table[8] = { 48, 48, 50, 55, 55, 59, 61, 65 };

    if (sf < 5 || sf > 12) {
        return 55;
    }
    return table[sf - 5];
}

/* ── CAD probe (mirror of lr20xx_cad_probe + lr20xx_lora_cad) ────────
 * Blocks until CAD_DONE (polling GetAndClearIrq like the driver) and
 * returns 1 = activity, 0 = silent, < 0 = error/-ETIMEDOUT.  The chip
 * is left in STANDBY (CAD exit mode STBY_RC) — the caller re-arms via
 * the next switch/arm, exactly like the driver documents. */

int lr_sniffer_cad_probe(uint32_t freq_hz, uint8_t sf, uint16_t bw_khz,
                         int8_t peak_offset)
{
    int ret = lr_sniffer_switch_band_cfg(freq_hz, sf, bw_khz, 4);

    if (ret) return ret;

    /* lr20xx_lora_cad parity: RX → standby before the CAD. */
    ret = lr_set_standby(LR20XX_STDBY_RC);
    if (ret) return ret;

    int peak = (int)lr_cad_detect_peak_dut(sf) + (int)peak_offset;

    if (peak < 48) peak = 48;
    else if (peak > 90) peak = 90;

    /* lr20xx_do_cad parity: symb_nb default 2, pnr_delta 0,
     * exit_mode CAD_ONLY (STBY_RC), timeout 0, det_peak. */
    {
        uint8_t p[7] = { 2, 0, 0, 0, 0, 0, (uint8_t)peak };

        ret = lr_cmd(LR20XX_OP_SET_LORA_CAD_PARAMS_DUT, p, sizeof(p),
                     NULL, 0);
        if (ret) return ret;
    }

    ret = lr_clear_irq(LR20XX_IRQ_ALL_MASK);
    if (ret) return ret;

    ret = lr_cmd(LR20XX_OP_SET_LORA_CAD_DUT, NULL, 0, NULL, 0);
    if (ret) return ret;

    /* CAD_DONE poll window (the real driver polls on a ms clock; the
     * sandbox polls a bounded iteration count — stub raises CAD_DONE
     * synchronously unless stub_set_cad_stuck(true)). */
    uint32_t irq = 0;

    for (int i = 0; i < 250; i++) {
        if (lr_get_and_clear_irq(&irq) == 0 &&
            (irq & LR20XX_IRQ_CAD_DONE)) {
            return (irq & LR20XX_IRQ_CAD_DETECTED) ? 1 : 0;
        }
    }

    return -ETIMEDOUT;
}

/* ── Sniffer WM-BUS poll-mode extension (F1, 2026-09-19) ─────────────
 * Byte-for-byte sequence mirrors of lr20xx_sniffer_wmbus_arm/_poll/
 * _stats in lr20xx_lora.c (mutex/led/log glue replaced with no-ops,
 * device pointer dropped — single-instance DUT).  Keep in lockstep.
 * Opcodes: SetWmbusParams 0x026A, GetWmbusRxStats 0x026C,
 * GetWmbusPacketStatus 0x026D; packet type WM-BUS = 8 (spec). */

#define LR20XX_OP_SET_WMBUS_PARAMS_DUT     0x026A
#define LR20XX_OP_GET_WMBUS_RX_STATS_DUT   0x026C
#define LR20XX_OP_GET_WMBUS_PKT_STATUS_DUT 0x026D

uint16_t lr_wmbus_pbl_len_tx(uint8_t mode)
{
    switch (mode) {
    case 0x00: return 30;  /* S */
    case 0x04: return 78;  /* R2 */
    case 0x0C: return 78;  /* F2 */
    case 0x01:
    case 0x02:
    case 0x03: return 38;  /* T1 / T2 meterRx / T2 meterTx */
    case 0x05:
    case 0x06:
    case 0x07: return 32;  /* C1 / C2 meterRx / C2 meterTx */
    default:   return 16;  /* N modes */
    }
}

void lr_sniffer_wmbus_build_params(uint8_t mode, uint8_t pkt_format,
                                   uint8_t pld_len, uint8_t out[8])
{
    uint16_t pbl = lr_wmbus_pbl_len_tx(mode);

    out[0] = (uint8_t)(mode & 0x0F);
    out[1] = 0xFF;                              /* rx_bw auto */
    out[2] = (uint8_t)(pkt_format & 0x01);
    out[3] = 0x00;                              /* addr filtering OFF */
    out[4] = pld_len;
    out[5] = (uint8_t)(pbl >> 8);
    out[6] = (uint8_t)(pbl & 0xFF);
    out[7] = 0xFF;                              /* pbl_len_detect auto */
}

int lr_sniffer_wmbus_arm(uint32_t freq_hz, uint8_t mode, uint8_t pkt_format,
                         uint8_t pld_len)
{
    int ret;

    /* Lean housekeeping (parity with lr20xx_switch_band). */
    ret = lr_set_standby(LR20XX_STDBY_RC);
    if (ret) return ret;
    ret = lr_clear_irq(LR20XX_IRQ_ALL_MASK);
    if (ret) return ret;
    ret = lr_cmd(LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);
    if (ret) return ret;

    if (lr_band_switch_needs_cal(lr_sniffer_cur_freq, freq_hz)) {
        uint16_t bin = lr_cal_fe_single_bin_hz(freq_hz);
        uint8_t p[2] = { (uint8_t)(bin >> 8), (uint8_t)(bin & 0xFF) };
        ret = lr_cmd(LR20XX_OP_CAL_FE_DUT, p, 2, NULL, 0);
        if (ret) return ret;
    }

    ret = lr_set_rf_frequency(freq_hz);
    if (ret) return ret;
    ret = lr_set_pkt_type(LR20XX_PKT_TYPE_WMBUS);
    if (ret) return ret;

    {
        uint8_t wp[8];
        lr_sniffer_wmbus_build_params(mode, pkt_format, pld_len, wp);
        ret = lr_cmd(LR20XX_OP_SET_WMBUS_PARAMS_DUT, wp, sizeof(wp),
                     NULL, 0);
        if (ret) return ret;
    }

    lr_sniffer_cur_freq = freq_hz;

    /* RX path + boost (mirror of lr_apply_rx_path; DUT always boosted). */
    {
        uint8_t path, boost;
        lr_rx_path_for_freq(freq_hz, true, &path, &boost);
        uint8_t p[2] = { path, boost };
        ret = lr_cmd(LR20XX_OP_SET_RX_PATH_DUT, p, sizeof(p), NULL, 0);
        if (ret) return ret;
    }

    /* Poll mode: silence DIO IRQ routing. */
    ret = lr_set_dio_irq_cfg(LR20XX_DIO_8, 0);
    if (ret) return ret;

    return lr_set_rx(LR20XX_RX_TIMEOUT_INF);
}

int lr_sniffer_wmbus_poll(uint8_t *buf, uint16_t cap, uint16_t *out_len,
                          struct lr_sniffer_wmbus_status *st)
{
    *out_len = 0;
    if (st) memset(st, 0, sizeof(*st));

    uint32_t irq = 0;
    int ret = lr_get_and_clear_irq(&irq);
    if (ret) return ret;
    if (!(irq & LR20XX_IRQ_RX_DONE)) return 0;

    uint16_t pkt_len = 0;

#ifndef LR2021_SIM_OLD_WMBUS_SINGLE_READ
    /* 3-read dance (mirror of lr_rx_flow_meshcore + the OOK poll): 1st
     * GetRxPacketLength = IRQ echo victim; GetWmbusPacketStatus = real
     * (L-field / pkt_len / RSSIs / CRC mask; consumes the echo); 2nd
     * GetRxPacketLength = REAL remaining-FIFO total; status pkt_len =
     * zero-fallback.  A single-read poll returns the echo and silently
     * DROPS the frame (negative control variant). */
    uint16_t pkt_len_raw = 0;
    lr_get_rx_packet_length(&pkt_len_raw);   /* 1st read → IRQ echo */
    (void)pkt_len_raw;

    uint16_t st_len = 0;
    {
        uint8_t resp[11] = { 0 };
        if (lr_cmd(LR20XX_OP_GET_WMBUS_PKT_STATUS_DUT, NULL, 0, resp,
                   sizeof(resp)) == 0) {
            uint16_t rssi_raw, sync_raw;
            st_len = (uint16_t)((resp[3] << 8) | resp[4]);
            if (st) {
                st->l_field = resp[2];
                st->pkt_len = st_len;
                rssi_raw = ((uint16_t)resp[5] << 1) |
                           ((resp[9] >> 4) & 0x01u);
                st->rssi_avg_dbm = -(int16_t)((rssi_raw + 1) / 2);
                sync_raw = ((uint16_t)resp[6] << 1) |
                           (resp[9] & 0x01u);
                st->rssi_sync_dbm = -(int16_t)((sync_raw + 1) / 2);
                st->crc_err_mask =
                    (uint32_t)((resp[9] >> 6) & 0x01u) |
                    ((uint32_t)resp[8] << 1) |
                    ((uint32_t)resp[7] << 9);
                st->syncword_idx = (resp[9] >> 7) & 0x01u;
                st->lqi = resp[10];
            }
        }
    }

    lr_get_rx_packet_length(&pkt_len);       /* 3rd read → REAL value */
    pkt_len &= 0xFF;
    if (pkt_len == 0) pkt_len = st_len;      /* fallback */
#else
    /* PRE-FIX variant (negative control): single-read poll — the FIRST
     * read after GetAndClearIrq returns the IRQ-word echo, so the frame
     * length is garbage and the packet is dropped.  The poll test must
     * FAIL on this build. */
    lr_get_rx_packet_length(&pkt_len);
    pkt_len &= 0xFF;
#endif

    if (pkt_len > cap) pkt_len = cap;
    if (pkt_len == 0) {
        lr_cmd(LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);
        lr_set_dio_irq_cfg(LR20XX_DIO_8, 0);
        lr_set_rx(LR20XX_RX_TIMEOUT_INF);
        return 0;
    }

    lr_fifo_read(buf, (uint8_t)pkt_len);
    lr_cmd(LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);

    lr_set_dio_irq_cfg(LR20XX_DIO_8, 0);
    lr_set_rx(LR20XX_RX_TIMEOUT_INF);
    *out_len = pkt_len;
    return 0;
}

int lr_sniffer_wmbus_stats(uint16_t *pkt_rx, uint16_t *crc_error,
                           uint16_t *len_error, uint8_t *raw_out)
{
    /* GetWmbusRxStats (0x026C) — DS Table 12-4 parity: three u16 BE
     * counters after the 2-byte status word. */
    uint8_t resp[16] = { 0 };
    int ret = lr_cmd(LR20XX_OP_GET_WMBUS_RX_STATS_DUT, NULL, 0, resp,
                     sizeof(resp));
    if (ret) return ret;
    if (raw_out)   memcpy(raw_out, resp, sizeof(resp));
    if (pkt_rx)    *pkt_rx    = ((uint16_t)resp[2] << 8) | resp[3];
    if (crc_error) *crc_error = ((uint16_t)resp[4] << 8) | resp[5];
    if (len_error) *len_error = ((uint16_t)resp[6] << 8) | resp[7];
    return 0;
}

/* ── Sniffer BLE poll-mode extension (F2, 2026-09-19) ────────────────
 * Byte-for-byte sequence mirrors of lr20xx_sniffer_ble_arm/_poll/_stats
 * in lr20xx_lora.c (mutex/led/log glue replaced with no-ops, device
 * pointer dropped — single-instance DUT).  Keep in lockstep.
 * Opcodes: SetBleModulationParams 0x0260, SetBleChannelParams 0x0261,
 * GetBleRxStats 0x0264, GetBlePacketStatus 0x0265; packet type BLE = 3. */

#define LR20XX_OP_SET_BLE_MOD_PARAMS_DUT     0x0260
#define LR20XX_OP_SET_BLE_CHANNEL_PARAMS_DUT 0x0261
#define LR20XX_OP_GET_BLE_RX_STATS_DUT       0x0264
#define LR20XX_OP_GET_BLE_PKT_STATUS_DUT     0x0265

void lr_sniffer_ble_build_channel_params(uint8_t whit_init,
                                         uint8_t channel_type,
                                         uint8_t out[9])
{
    out[0] = (uint8_t)(channel_type & 0x0F);   /* crc_in_fifo = 0 */
    out[1] = whit_init;
    out[2] = (uint8_t)((LR20XX_BLE_ADV_CRC_INIT >> 16) & 0xFFu);
    out[3] = (uint8_t)((LR20XX_BLE_ADV_CRC_INIT >> 8) & 0xFFu);
    out[4] = (uint8_t)(LR20XX_BLE_ADV_CRC_INIT & 0xFFu);
    out[5] = (uint8_t)((LR20XX_BLE_ADV_SYNCWORD >> 24) & 0xFFu);
    out[6] = (uint8_t)((LR20XX_BLE_ADV_SYNCWORD >> 16) & 0xFFu);
    out[7] = (uint8_t)((LR20XX_BLE_ADV_SYNCWORD >> 8) & 0xFFu);
    out[8] = (uint8_t)(LR20XX_BLE_ADV_SYNCWORD & 0xFFu);
}

int lr_sniffer_ble_arm(uint32_t freq_hz, uint8_t whit_init,
                       uint8_t channel_type)
{
    int ret;

    /* Lean housekeeping (parity with lr20xx_switch_band). */
    ret = lr_set_standby(LR20XX_STDBY_RC);
    if (ret) return ret;
    ret = lr_clear_irq(LR20XX_IRQ_ALL_MASK);
    if (ret) return ret;
    ret = lr_cmd(LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);
    if (ret) return ret;

    if (lr_band_switch_needs_cal(lr_sniffer_cur_freq, freq_hz)) {
        uint16_t bin = lr_cal_fe_single_bin_hz(freq_hz);
        uint8_t p[2] = { (uint8_t)(bin >> 8), (uint8_t)(bin & 0xFF) };
        ret = lr_cmd(LR20XX_OP_CAL_FE_DUT, p, 2, NULL, 0);
        if (ret) return ret;
    }

    ret = lr_set_rf_frequency(freq_hz);
    if (ret) return ret;
    ret = lr_set_pkt_type(LR20XX_PKT_TYPE_BLE);
    if (ret) return ret;

    {
        /* SetBleModulationParams (0x0260): [mode]; LE 1M = 0. */
        uint8_t mod_p[1] = { 0x00 };
        ret = lr_cmd(LR20XX_OP_SET_BLE_MOD_PARAMS_DUT, mod_p,
                     sizeof(mod_p), NULL, 0);
        if (ret) return ret;
    }

    {
        /* SetBleChannelParams (0x0261): advertising defaults —
         * crc_in_fifo 0, crc_init 0x555555, access 0x8E89BED6. */
        uint8_t chan_p[9];
        lr_sniffer_ble_build_channel_params(whit_init, channel_type,
                                            chan_p);
        ret = lr_cmd(LR20XX_OP_SET_BLE_CHANNEL_PARAMS_DUT, chan_p,
                     sizeof(chan_p), NULL, 0);
        if (ret) return ret;
    }

    lr_sniffer_cur_freq = freq_hz;

    /* RX path + boost (mirror of lr_apply_rx_path; DUT always boosted;
     * 2.4 GHz → HF path). */
    {
        uint8_t path, boost;
        lr_rx_path_for_freq(freq_hz, true, &path, &boost);
        uint8_t p[2] = { path, boost };
        ret = lr_cmd(LR20XX_OP_SET_RX_PATH_DUT, p, sizeof(p), NULL, 0);
        if (ret) return ret;
    }

    /* Poll mode: silence DIO IRQ routing. */
    ret = lr_set_dio_irq_cfg(LR20XX_DIO_8, 0);
    if (ret) return ret;

    return lr_set_rx(LR20XX_RX_TIMEOUT_INF);
}

int lr_sniffer_ble_poll(uint8_t *buf, uint16_t cap, uint16_t *out_len,
                        struct lr_sniffer_ble_status *st)
{
    *out_len = 0;
    if (st) memset(st, 0, sizeof(*st));

    uint32_t irq = 0;
    int ret = lr_get_and_clear_irq(&irq);
    if (ret) return ret;
    if (!(irq & LR20XX_IRQ_RX_DONE)) return 0;

    uint16_t pkt_len = 0;

#ifndef LR2021_SIM_OLD_BLE_SINGLE_READ
    /* 3-read dance (mirror of the OOK/WM-BUS poll): 1st
     * GetRxPacketLength = IRQ echo victim; GetBlePacketStatus = real
     * (pkt_len / RSSIs / LQI; consumes the echo); 2nd
     * GetRxPacketLength = REAL remaining-FIFO total; status pkt_len =
     * zero-fallback.  A single-read poll returns the echo and silently
     * DROPS the PDU (negative control variant). */
    uint16_t pkt_len_raw = 0;
    lr_get_rx_packet_length(&pkt_len_raw);   /* 1st read → IRQ echo */
    (void)pkt_len_raw;

    uint16_t st_len = 0;
    {
        uint8_t resp[8] = { 0 };
        if (lr_cmd(LR20XX_OP_GET_BLE_PKT_STATUS_DUT, NULL, 0, resp,
                   sizeof(resp)) == 0) {
            uint16_t rssi_raw, sync_raw;
            st_len = (uint16_t)((resp[2] << 8) | resp[3]);
            if (st) {
                st->pkt_len = st_len;
                rssi_raw = ((uint16_t)resp[4] << 1) |
                           ((resp[6] >> 2) & 0x01u);
                st->rssi_avg_dbm = -(int16_t)((rssi_raw + 1) / 2);
                sync_raw = ((uint16_t)resp[5] << 1) |
                           (resp[6] & 0x01u);
                st->rssi_sync_dbm = -(int16_t)((sync_raw + 1) / 2);
                st->lqi = resp[7];
            }
        }
    }

    lr_get_rx_packet_length(&pkt_len);       /* 3rd read → REAL value */
    pkt_len &= 0xFF;
    if (pkt_len == 0) pkt_len = st_len;      /* fallback */
#else
    /* PRE-FIX variant (negative control): single-read poll — the FIRST
     * read after GetAndClearIrq returns the IRQ-word echo, so the PDU
     * length is garbage and the packet is dropped.  The poll test must
     * FAIL on this build. */
    lr_get_rx_packet_length(&pkt_len);
    pkt_len &= 0xFF;
#endif

    if (pkt_len > cap) pkt_len = cap;
    if (pkt_len == 0) {
        lr_cmd(LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);
        lr_set_dio_irq_cfg(LR20XX_DIO_8, 0);
        lr_set_rx(LR20XX_RX_TIMEOUT_INF);
        return 0;
    }

    lr_fifo_read(buf, (uint8_t)pkt_len);
    lr_cmd(LR20XX_OP_CLEAR_RX_FIFO, NULL, 0, NULL, 0);

    lr_set_dio_irq_cfg(LR20XX_DIO_8, 0);
    lr_set_rx(LR20XX_RX_TIMEOUT_INF);
    *out_len = pkt_len;
    return 0;
}

int lr_sniffer_ble_stats(uint16_t *pkt_rx, uint16_t *crc_error,
                         uint16_t *len_error, uint8_t *raw_out)
{
    /* GetBleRxStats (0x0264) — DS Table 14-6 parity: three u16 BE
     * counters after the 2-byte status word. */
    uint8_t resp[16] = { 0 };
    int ret = lr_cmd(LR20XX_OP_GET_BLE_RX_STATS_DUT, NULL, 0, resp,
                     sizeof(resp));
    if (ret) return ret;
    if (raw_out)   memcpy(raw_out, resp, sizeof(resp));
    if (pkt_rx)    *pkt_rx    = ((uint16_t)resp[2] << 8) | resp[3];
    if (crc_error) *crc_error = ((uint16_t)resp[4] << 8) | resp[5];
    if (len_error) *len_error = ((uint16_t)resp[6] << 8) | resp[7];
    return 0;
}
