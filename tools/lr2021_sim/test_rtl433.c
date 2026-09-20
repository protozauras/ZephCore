/*
 * SPDX-License-Identifier: MIT
 *
 * test_rtl433.c — sandbox tests for the on-chip rtl_433 OOK decode path
 * (zephcore/src/rtl433/sniffer_rtl433.c, phase 3 of the RF-sniffer plan).
 *
 * Covers, with a negative control:
 *
 *   1. FineOffset WH2  (OOK_PULSE_PWM): full frame -> exactly 1 JSON line
 *      with the expected model / id / temperature / humidity.
 *   2. Acurite 986     (OOK_PULSE_PPM): full frame -> 1 JSON line.
 *   3. NEGATIVE: WH2 with one bit flipped in the CRC byte -> 0 lines
 *      (proves CRC gating, not a pass-through).
 *   4. Multi-chunk: WH2 split across two deliveries (240 bits + rest)
 *      with the long zero tail -> still exactly 1 line.
 *   5. Noise rejection: 60 bytes of alternating bits, no long zero run
 *      -> 0 JSON lines.
 *
 * The synthetic frames are built from the protocol docs in the vendored
 * decoders (fineoffset.c / acurite.c / tpms_*.c) with the 50 µs-per-bit
 * OOK run-length representation at the 20 kbps capture clock
 * (1-run => pulse us, 0-run => gap us).
 *
 * Counters are owned by main() in test_lr2021_driver.c; run_rtl433_tests
 * writes through the pointers so failures fail the whole suite.
 */

#include "sniffer_rtl433.h"
#include "bit_util.h" /* crc8 / crc8le — the SAME functions the vendored
                       * decoders use for their message checksums */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── Counter plumbing (owned by test_lr2021_driver.c main) ──────────── */

static int *g_run_ptr;
static int *g_fail_ptr;

#define LOG(fmt, ...)  do { printf(fmt "\n", ##__VA_ARGS__); } while (0)
#define PASS(name)     do { LOG("  [PASS] %s", name); (*g_run_ptr)++; } while (0)
#define FAIL(name, fmt, ...) do {                                              \
        LOG("  [FAIL] %s: " fmt, name, ##__VA_ARGS__);                         \
        (*g_fail_ptr)++; (*g_run_ptr)++;                                       \
    } while (0)
#define CHECK(cond, name, fmt, ...) do {                                        \
        if (!(cond)) { FAIL(name, fmt, ##__VA_ARGS__); return; }               \
    } while (0)

/* ── Bit/run builders ────────────────────────────────────────────────── */

#define BB_MAX 2048

typedef struct {
    uint8_t b[BB_MAX / 8];
    uint32_t n;
} bitvec_t;

static void bb_push_bit(bitvec_t *v, int bit)
{
    if (v->n >= BB_MAX) {
        return;
    }
    if (bit) {
        v->b[v->n >> 3] |= (uint8_t)(0x80u >> (v->n & 7u));
    }
    v->n++;
}

static void bb_push_run(bitvec_t *v, int level, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        bb_push_bit(v, level);
    }
}

static void feed_zero_bytes(uint32_t nbytes)
{
    static uint8_t zbuf[64];
    uint32_t left = nbytes;

    memset(zbuf, 0, sizeof(zbuf));
    while (left > 0) {
        uint32_t k = left > sizeof(zbuf) ? (uint32_t)sizeof(zbuf) : left;
        snf_rtl433_feed(zbuf, (uint16_t)k);
        left -= k;
    }
}

/* ── Frame builders ──────────────────────────────────────────────────── */

/* FineOffset WH2: 48 bits = 0xFF + 5 bytes [type|id][id|temp][temp][humi]
 * [crc].  PWM: bit 1 = 500 us pulse, bit 0 = 1500 us pulse, 1000 us gap
 * between pulses (fineoffset.c doc: 544/1524/1036 us on air, decoder
 * windows short=500 long=1500 tol=160).
 * `corrupt_crc` builds a perfectly-formed frame whose CRC byte is
 * corrupted BEFORE modulation — it decodes cleanly on air and must then
 * be rejected by the decoder's crc8 gate (DECODE_FAIL_MIC). */
static void wh2_frame_ex(bitvec_t *v, int id, int temp_x10, int humidity,
                         int corrupt_crc)
{
    uint8_t b[5];
    uint8_t msg[6];

    b[0] = (uint8_t)(0x40 | ((id >> 4) & 0x0F));     /* type 4, id hi */
    b[1] = (uint8_t)(((id & 0x0F) << 4) | ((temp_x10 >> 8) & 0x0F));
    b[2] = (uint8_t)(temp_x10 & 0xFF);
    b[3] = (uint8_t)humidity;
    b[4] = crc8(b, 4, 0x31, 0);
    if (corrupt_crc) {
        b[4] ^= 0x01u; /* corrupt one CRC bit before modulation */
    }

    msg[0] = 0xFF;
    memcpy(&msg[1], b, 5);

    for (int i = 0; i < 6; i++) {
        for (int bit = 7; bit >= 0; bit--) {
            int one = (msg[i] >> bit) & 1;
            bb_push_run(v, 1, one ? 10u : 30u); /* pulse: 500/1500 us */
            bb_push_run(v, 0, 20u);            /* gap:   1000 us */
        }
    }
}

static void wh2_frame(bitvec_t *v, int id, int temp_x10, int humidity)
{
    wh2_frame_ex(v, id, temp_x10, humidity, 0);
}

/* Acurite 986: PPM.  Preamble 2x(216us pulse + 276us gap) + 4x(1600us
 * pulse + 1560us gap), then 5 bytes LSB-first per byte (acurite.c doc):
 * tempf | sensor_id hi | sensor_id lo | status | crc8le(...,0x07,0).
 * Data: 200us pulse + 500us gap (bit 0) / 900us gap (bit 1). */
static void acurite_986_frame(bitvec_t *v, int temp_f, int sensor_id,
                              int status)
{
    uint8_t br[5];

    br[0] = (uint8_t)temp_f;
    br[1] = (uint8_t)(sensor_id >> 8);
    br[2] = (uint8_t)(sensor_id & 0xFF);
    br[3] = (uint8_t)status;
    br[4] = crc8le(br, 4, 0x07, 0);

    for (int i = 0; i < 2; i++) {
        bb_push_run(v, 1, 4u);  /* 216 us pulse  -> 200 us */
        bb_push_run(v, 0, 6u);  /* 276 us gap    -> 300 us */
    }
    for (int i = 0; i < 4; i++) {
        bb_push_run(v, 1, 32u); /* 1600 us pulse */
        bb_push_run(v, 0, 32u); /* 1560 us gap  */
    }
    for (int b = 0; b < 5; b++) {
        for (int bit = 0; bit < 8; bit++) {
            int one = (br[b] >> bit) & 1;
            bb_push_run(v, 1, 4u);              /* 220 us pulse -> 200 us */
            bb_push_run(v, 0, one ? 18u : 10u); /* 880/520 us gap */
        }
    }
}

/* ── Capture helpers ─────────────────────────────────────────────────── */

static int expect_json_lines(int expected, char const *name)
{
    int n = snf_test_json_count();

    if (n != expected) {
        snf_test_dbg_dump();
        LOG("    (finalize count: %u, total lines: %d)",
            (unsigned)snf_test_finalize_count(), snf_test_line_count());
        for (int i = 0; i < snf_test_line_count(); i++) {
            LOG("    line[%d]: %s", i, snf_test_line(i));
        }
        FAIL(name, "expected %d JSON line(s), got %d", expected, n);
        return 0;
    }
    return 1;
}

/* ── Tests ───────────────────────────────────────────────────────────── */

/* 1) WH2 full frame in 3 deliveries -> exactly one JSON line with the
 * expected model / id / temperature / humidity. */
static void test_wh2_full_frame(void)
{
    bitvec_t v = {{0}, 0};

    wh2_frame(&v, 0x5A, 213, 55); /* id 90, +21.3 C, 55 %RH */
    snf_test_capture_reset();

    /* 3 deliveries: 240 bits, 240 bits, rest + 40 zero bytes (>= 30 ms
     * gap tail => end of transmission). */
    snf_rtl433_feed(v.b, 30);
    snf_rtl433_feed(v.b + 30, 30);
    snf_rtl433_feed(v.b + 60, (uint16_t)((v.n + 7u) / 8u - 60u));
    feed_zero_bytes(80);

    CHECK(expect_json_lines(1, "wh2_json_line_count"),
          "wh2_json_line_count", "exactly one JSON line expected");
    CHECK(snf_test_json_match("Fineoffset-WH2", 3, (char const *[]){"\"id\":90", "temperature_C\":21.3", "humidity\":55"}),
          "wh2_json_content", "WH2 JSON content mismatch");
    PASS("rtl433_wh2_full_frame");
}

/* 2) Acurite 986 (PPM) -> one JSON line with the expected content. */
static void test_acurite_986_full_frame(void)
{
    bitvec_t v = {{0}, 0};

    acurite_986_frame(&v, 72, 300, 0); /* 72 F, id 300, sensor 1 */
    snf_test_capture_reset();

    snf_rtl433_feed(v.b, 30);
    snf_rtl433_feed(v.b + 30, (uint16_t)((v.n + 7u) / 8u - 30u));
    feed_zero_bytes(80);

    CHECK(expect_json_lines(1, "acurite986_json_line_count"),
          "acurite986_json_line_count", "exactly one JSON line expected");
    CHECK(snf_test_json_match("Acurite-986", 4, (char const *[]){"\"id\":300", "\"channel\":\"1R\"", "temperature_F\":72.0", "battery_ok\":1"}),
          "acurite986_json_content", "Acurite-986 JSON content mismatch");
    PASS("rtl433_acurite_986_frame");
}

/* 3) NEGATIVE CONTROL: WH2 frame with one bit flipped in the CRC byte
 * (corrupted BEFORE modulation — the frame is perfectly formed on air,
 * decodes cleanly, and must then be rejected by the decoder's crc8
 * gate): zero JSON lines. */
static void test_wh2_bad_crc_rejected(void)
{
    bitvec_t v = {{0}, 0};
    uint32_t nbytes;

    wh2_frame_ex(&v, 0x5A, 213, 55, 1 /* corrupt crc */);
    nbytes = (v.n + 7u) / 8u;

    snf_test_capture_reset();
    snf_rtl433_feed(v.b, 30);
    snf_rtl433_feed(v.b + 30, (uint16_t)(nbytes - 30u));
    feed_zero_bytes(80);

    CHECK(expect_json_lines(0, "wh2_badcrc_zero_lines"),
          "wh2_badcrc_zero_lines", "corrupted frame must not decode");
    PASS("rtl433_wh2_bad_crc_rejected");
}

/* 4) Multi-chunk split: WH2 split across two deliveries — 240 bits,
 * then the remainder — with the long zero gap tail.  Still exactly one
 * JSON line (bit accumulation must survive chunk boundaries). */
static void test_wh2_split_across_chunks(void)
{
    bitvec_t v = {{0}, 0};
    uint32_t nbytes;

    wh2_frame(&v, 0x5A, 213, 55);
    nbytes = (v.n + 7u) / 8u;

    snf_test_capture_reset();
    snf_rtl433_feed(v.b, 30);                              /* 240 bits */
    snf_rtl433_feed(v.b + 30, (uint16_t)(nbytes - 30u));   /* rest */
    feed_zero_bytes(80);                                   /* 30 ms tail */

    CHECK(expect_json_lines(1, "wh2_split_json_line_count"),
          "wh2_split_json_line_count", "exactly one JSON line expected");
    CHECK(snf_test_json_match("Fineoffset-WH2", 1, (char const *[]){"\"id\":90"}),
          "wh2_split_json_content", "WH2 split JSON content mismatch");
    PASS("rtl433_wh2_split_across_chunks");
}

/* 5) Noise rejection: 60 bytes of alternating bits (no zero run longer
 * than 1, no long gap) -> zero JSON lines. */
static void test_noise_rejected(void)
{
    uint8_t noise[60];

    for (int i = 0; i < 60; i++) {
        noise[i] = (i & 1) ? 0x55u : 0xAAu;
    }

    snf_test_capture_reset();
    snf_rtl433_feed(noise, 60);

    CHECK(expect_json_lines(0, "noise_zero_lines"),
          "noise_zero_lines", "noise must not decode");
    PASS("rtl433_noise_rejected");
}

/* ── Manchester (OOK_MC_ZEROBIT) frame builder + TPMS tests ─────────── */

/* Set one bit of an MSB-first bit array. */
static void bv_set_bit(uint8_t *b, uint32_t idx, int val)
{
    uint8_t mask = (uint8_t)(0x80u >> (idx & 7u));

    if (val) {
        b[idx >> 3] |= mask;
    } else {
        b[idx >> 3] &= (uint8_t)~mask;
    }
}

/* MC_ZEROBIT on air: data bit 0 = rising mid-cell edge ([low, high]
 * halves), data bit 1 = falling mid-cell edge ([high, low] halves).
 * `half_bits` is the half-cell length on the 50 µs capture grid —
 * 2 bits = 100 µs is what the 122 µs Schrader/GM half-bit becomes when
 * the 20 kbps capture clock's phase lands on the short side.  The glue
 * strips LEADING zeros, so cell 0's low half merges into the pre-frame
 * gap; the slicer recovers bit 0 via its hardcoded leading zero (both
 * TPMS preambles start with 0, so nothing is lost).  At the OLD 10 kbps
 * grid (-DSNF_OOK_BIT_US=100) these frames stop decoding — that build
 * is the negative control proving the tests gate the capture clock. */
static void mc_frame(bitvec_t *v, uint8_t const *msg, uint32_t n_bits)
{
    for (uint32_t i = 0; i < n_bits; i++) {
        int bit    = (msg[i >> 3] >> (7 - (i & 7))) & 1;
        int first  = bit;          /* data 1: [H, L]; data 0: [L, H] */
        int second = bit ? 0 : 1;

        /* Push BOTH halves unconditionally: consecutive same-level
         * halves merge naturally in the accumulator (one long run);
         * skipping a push here would DELETE 100 µs from the timeline
         * and corrupt every following cell boundary. */
        bb_push_run(v, first, 2u);  /* 100 µs half-cell */
        bb_push_run(v, second, 2u);
    }
}

/* 6) Schrader Motorcycle TPMS (OOK MC, 122 µs half-bit): preamble
 * {13}0x7ff8 + 56-bit payload; real captured frame fd420b3ddc5352
 * (upstream tpms_schrader_motorcycle.c doc). */
static void test_tpms_schrader_mc(void)
{
    static const uint8_t msg[7] = { 0xfd, 0x42, 0x0b, 0x3d, 0xdc, 0x53, 0x52 };
    uint8_t frame[9] = { 0 };
    bitvec_t v = { { 0 }, 0 };

    bv_set_bit(frame, 0, 0);                       /* 0 + 12 ones */
    for (int i = 1; i < 13; i++) {
        bv_set_bit(frame, (uint32_t)i, 1);
    }
    for (uint32_t i = 0; i < 56u; i++) {
        bv_set_bit(frame, 13u + i, (msg[i >> 3] >> (7 - (i & 7))) & 1);
    }

    mc_frame(&v, frame, 69u);
    snf_test_capture_reset();

    /* The MC frames are SHORT (214 bits = 27 bytes at the 50 µs grid) —
     * smaller than the 30-byte first chunk, so the remainder feed is
     * conditional (an underflow here reads past the bit vector). */
    uint32_t nbytes = (v.n + 7u) / 8u;
    snf_rtl433_feed(v.b, 30);
    if (nbytes > 30u) {
        snf_rtl433_feed(v.b + 30, (uint16_t)(nbytes - 30u));
    }
    feed_zero_bytes(80);

    CHECK(expect_json_lines(1, "tpms_schrader_line_count"),
          "tpms_schrader_line_count", "exactly one JSON line expected");
    CHECK(snf_test_json_match("Schrader-Motorcycle", 3,
                              (char const *[]){ "\"id\":5276367",
                                                "pressure_kPa\":238.0",
                                                "temperature_C\":33.0" }),
          "tpms_schrader_json", "Schrader MC JSON content mismatch");
    PASS("rtl433_tpms_schrader_mc");
}

/* 7) GM-Aftermarket TPMS (OOK MC, 120 µs half-bit): 48 zero preamble
 * bits + flags/id/pressure/temp/mod-256 checksum, 130 bits total. */
static void test_tpms_gm(void)
{
    uint8_t frame[17] = { 0 };
    uint8_t sum = 0;
    uint32_t nbytes;
    bitvec_t v = { { 0 }, 0 };

    frame[6]  = 0x00;
    frame[7]  = 0x09;                              /* flags */
    frame[11] = 0xAB;
    frame[12] = 0xCD;                              /* id 0xABCD */
    frame[13] = 0x5A;                              /* 90 → 247.5 kPa */
    frame[14] = 0x50;                              /* 80 → 20 C */
    for (int i = 6; i <= 14; i++) {
        sum = (uint8_t)(sum + frame[i]);
    }
    frame[15] = sum;

    /* frame[0..5] = 0x00 = the 48 zero preamble bits; 16 bytes + 2 pad. */
    mc_frame(&v, frame, 130u);
    snf_test_capture_reset();

    nbytes = (v.n + 7u) / 8u;
    snf_rtl433_feed(v.b, 40);
    if (nbytes > 40u) {
        snf_rtl433_feed(v.b + 40, (uint16_t)(nbytes - 40u));
    }
    feed_zero_bytes(80);   /* GM reset_limit = 15600 µs: tail must beat it */

    CHECK(expect_json_lines(1, "tpms_gm_line_count"),
          "tpms_gm_line_count", "exactly one JSON line expected");
    CHECK(snf_test_json_match("GM-Aftermarket", 3,
                              (char const *[]){ "\"id\":43981",
                                                "pressure_kPa\":247.5",
                                                "temperature_C\":20.0" }),
          "tpms_gm_json", "GM aftermarket JSON content mismatch");
    PASS("rtl433_tpms_gm");
}

/* 8) Freq context: the glue echoes the capture frequency as "freq" in
 * every OOK JSON line — the ETL derives its band tag from it (the multi
 * leg rotates 433.92 / 868.35). */
static void test_ook_freq_field(void)
{
    bitvec_t v = { { 0 }, 0 };

    wh2_frame(&v, 0x5A, 213, 55);
    snf_test_capture_reset();
    snf_rtl433_set_ook_freq(868350000u);

    snf_rtl433_feed(v.b, 30);
    snf_rtl433_feed(v.b + 30, (uint16_t)((v.n + 7u) / 8u - 30u));
    feed_zero_bytes(80);

    CHECK(snf_test_json_match("Fineoffset-WH2", 1,
                              (char const *[]){ "\"freq\":868350000" }),
          "ook_freq_field", "freq context missing from OOK JSON");
    snf_rtl433_set_ook_freq(433920000u);  /* restore for later tests */
    PASS("rtl433_ook_freq_field");
}

/* ── Entry point (called from test_lr2021_driver.c main) ─────────────── */

void run_rtl433_tests(int *tests_run, int *tests_failed)
{
    g_run_ptr  = tests_run;
    g_fail_ptr = tests_failed;

    LOG("---- rtl_433 on-chip decode (phase 3, 2026-09-19) ----");
    setvbuf(stdout, NULL, _IONBF, 0);   /* crash-tracing: no lost buffers */
    snf_rtl433_init();
    test_wh2_full_frame();
    test_acurite_986_full_frame();
    test_wh2_bad_crc_rejected();
    test_wh2_split_across_chunks();
    test_noise_rejected();
    test_tpms_schrader_mc();
    test_tpms_gm();
    test_ook_freq_field();
}
