/*
 * SPDX-License-Identifier: MIT
 *
 * test_lr2021_driver.c — unit tests for the LR2021 command layer.
 *
 * These tests are the sandbox that lets us validate driver logic
 * BEFORE flashing. The hardest-to-test, easiest-to-bug pieces are
 * concentrated here:
 *
 *   1. lr_cmd() single-NSS read correctness for every command the
 *      RX path uses (get_and_clear_irq, get_rx_pkt_length,
 *      get_lora_packet_status, fifo_read).
 *   2. Bug injection that simulates the CS-toggle pathology
 *      (legacy pre-215f6d8 state): we verify the fixed driver
 *      still reads real data, NOT the IRQ echo.
 *   3. End-to-end RX flow mock (set_rx + IRQ + read sequence).
 *   4. Wrapper correctness: lr_get_and_clear_irq DOES clear
 *      the IRQ register (subsequent reads return 0).
 *
 * All tests print PASS/FAIL with a leading label and a counter.
 * Exit code 0 iff every assertion held.
 */
#include "driver_under_test.h"
#include "stub_lr2021.h"
#include "../../zephcore/src/sniffer_wmbus_parse.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

/* ── Tiny assert helper ─────────────────────────────────────────────── */

static int g_failures = 0;
static int g_tests_run = 0;

#define LOG(fmt, ...)  do { printf(fmt "\n", ##__VA_ARGS__); } while (0)
#define PASS(name)     do { LOG("  [PASS] %s", name); g_tests_run++; } while (0)
#define FAIL(name, fmt, ...) do {                                              \
        LOG("  [FAIL] %s: " fmt, name, ##__VA_ARGS__);                         \
        g_failures++; g_tests_run++;                                           \
    } while (0)
#define CHECK(cond, name, fmt, ...) do {                                        \
        if (!(cond)) { FAIL(name, fmt, ##__VA_ARGS__); return; }               \
    } while (0)
#define CHECK_EQ(actual, expected, name) do {                                  \
        long long a_ = (long long)(actual), e_ = (long long)(expected);        \
        if (a_ != e_) {                                                        \
            FAIL(name, "expected 0x%08X, got 0x%08X",                          \
                 (unsigned)e_, (unsigned)a_);                                  \
            return;                                                            \
        }                                                                      \
    } while (0)

/* Helper: build a known fake LoRa packet (deterministic bytes). */
static void make_test_packet(uint8_t *buf, size_t *len)
{
    static const uint8_t pkt[] = {
        0x0D, 0xA1, 0xB2, 0xC3, 0xD4,                    /* header */
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,  /* payload */
    };
    memcpy(buf, pkt, sizeof(pkt));
    *len = sizeof(pkt);
}

/* ── Test 1: read commands return real data (single NSS is enough) ─── */

static void test_read_returns_real_data(void)
{
    const char *name = "test_read_returns_real_data";
    stub_reset();

    /* GetVersion should return 0x01 0x24. */
    uint8_t maj = 0, min = 0;
    int rc = lr_get_version(&maj, &min);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(maj, 0x01, name);
    CHECK_EQ(min, 0x24, name);

    /* No false CS toggle: command should have produced ONE transceive. */
    CHECK_EQ(g_stub.transceive_calls, 1u, name);

    PASS(name);
}

/* ── Test 2: lr_get_and_clear_irq clears IRQ register ──────────────── */

static void test_get_and_clear_irq_clears_irq(void)
{
    const char *name = "test_get_and_clear_irq_clears_irq";
    stub_reset();
    stub_fake_irq_fire_rx_done();
    CHECK(g_stub.irq_pending != 0, name, "stub should have IRQ pending");
    CHECK(g_stub.dio_pin_high, name, "DIO8 should be HIGH after IRQ fire");

    uint32_t irq = 0;
    int rc = lr_get_irq_status(&irq);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(irq, LR20XX_IRQ_RX_DONE, name);

    /* The read ALSO clears the IRQ register and the DIO line. */
    CHECK_EQ(g_stub.irq_pending, 0u, name);
    CHECK_EQ(g_stub.dio_pin_high, false, name);

    PASS(name);
}

/* ── Test 3: GetRxBufferStatus returns real length even after a (simulated) ─
   ──── legacy CS-toggle bug is forced ON in the stub. The bug would ──
   ──── cause a TWO-phase read to return the IRQ word; our driver now ──
   ──── uses single-NSS so it must still see the real length. ─────────── */

static void test_get_rx_pkt_length_layout_status_byte(void)
{
    const char *name = "test_get_rx_pkt_length_layout_status_byte";
    stub_reset();

    /* 0x0212 on the LR2021 = [stat16][stat_byte 0x14][len_byte] — the
     * status byte in resp[2] is NOT a length (pitfall #34). A drain that
     * read resp[2]=0x14=20 B shifted every packet by 2 B (09b5571). The
     * driver parses (resp[2]<<8)|resp[3] as diagnostic raw_len only. */

    /* Seed the stub as if a packet has been received (length 22). */
    uint8_t fake_pkt[22];
    for (size_t i = 0; i < sizeof(fake_pkt); i++) fake_pkt[i] = (uint8_t)(i * 7 + 11);
    stub_inject_packet(fake_pkt, sizeof(fake_pkt));

    uint16_t pkt_len = 0;
    int rc = lr_get_rx_packet_length(&pkt_len);
    CHECK_EQ(rc, 0, name);
    /* raw_len = (stat<<8)|len = 0x1416 for a 22-byte FIFO. */
    CHECK(pkt_len == (uint16_t)((0x14u << 8) | 22u), name,
          "expected raw_len=0x1416, got 0x%04X (resp[2] is STATUS 0x14, not length)",
          (unsigned)pkt_len);
    /* Guard: the drain bug read resp[2] as length -> 20, NOT 22. */
    CHECK((pkt_len & 0xFFu) == 22u, name,
          "len byte must be 22, got %u", (unsigned)(pkt_len & 0xFFu));

    PASS(name);
}

/* ── Test 4: GetLoraPacketStatus returns real st_len ────────────────── */

static void test_get_lora_packet_status_returns_st_len(void)
{
    const char *name = "test_get_lora_packet_status_returns_st_len";
    stub_reset();
    stub_inject_packet((const uint8_t*)"xyz", 3);  /* sets rx_buffer_length = 3 */

    uint8_t st_len = 0;
    int16_t rssi = 0, rssi_signal = 0;
    int8_t snr = 0;
    int rc = lr_get_lora_packet_status(&st_len, &rssi, &rssi_signal, &snr);
    CHECK_EQ(rc, 0, name);
    /* Driver's `lr_get_lora_packet_status` reads at resp[3]; stub writes
     * the packetLen byte there (real LR2021 layout). */
    CHECK_EQ(st_len, 3u, name);

    PASS(name);
}

/* ── RSSI source selection (L5, 2026-08-06): on this hardware the
   ──── packet RSSI field reads 0 even for strong links, so the RX
   ──── handler falls back to the despread signal RSSI (§9.9.9). */
static void test_rssi_effective_fallback(void)
{
    const char *name = "test_rssi_effective_fallback";
    /* Packet field empty (0) + real (negative) signal → signal wins. */
    CHECK_EQ(lr_rssi_effective(0, -17), -17, name);
    CHECK_EQ(lr_rssi_effective(0, -47), -47, name);
    /* Valid packet RSSI → never overwritten by the signal RSSI. */
    CHECK_EQ(lr_rssi_effective(-44, -50), -44, name);
    CHECK_EQ(lr_rssi_effective(-5, -30), -5, name);
    /* Both empty → stays 0; non-negative "signal" → no swap. */
    CHECK_EQ(lr_rssi_effective(0, 0), 0, name);
    CHECK_EQ(lr_rssi_effective(0, 5), 0, name);
    PASS(name);
}

/* ── Test 5: full RX_DONE handler flow — read IRQ + length + FIFO ──── */

static void test_rx_done_handler_flow(void)
{
    const char *name = "test_rx_done_handler_flow";
    stub_reset();

    /* Place a known packet in the FIFO before the IRQ fires. */
    uint8_t pkt[16];   /* big enough for the literal used by make_test_packet */
    size_t  pkt_len = 0;
    make_test_packet(pkt, &pkt_len);
    stub_inject_packet(pkt, pkt_len);

    /* Set mode to RX (driver does this earlier in start_rx). */
    lr_set_rx(0xFFFFFF);

    /* Simulate the chip asserting RX_DONE IRQ after a real packet. */
    stub_fake_irq_fire_rx_done();
    CHECK(stub_dio_pin_read(), name, "DIO8 should be HIGH after RX_DONE IRQ");

    /* === DIO work handler replicas in order: === */

    /* 1. read IRQ (atomic read+clear) */
    uint32_t irq = 0;
    int rc = lr_get_irq_status(&irq);
    CHECK_EQ(rc, 0, name);
    CHECK((irq & LR20XX_IRQ_RX_DONE), name,
          "handler didn't observe RX_DONE in irq word 0x%08X", (unsigned)irq);
    CHECK_EQ(g_stub.irq_pending, 0u, name);

    /* 2. driver primary length: GetRxPktLength FIRST. On real hardware
     * this is the command that may return the IRQ-word echo (one
     * transaction behind), so it is diagnostic only. */
    uint16_t rx_pkt_len = 0;
    rc = lr_get_rx_packet_length(&rx_pkt_len);
    CHECK_EQ(rc, 0, name);

    /* 3. GetLoRaPacketStatus SECOND — authoritative. On real hardware
     * the SECOND status command returns the real packet status; its
     * st_len overrides pkt_len (fix for the 433499d reorder that read
     * the echo into st_len=4/6 and truncated every inbound packet). */
    uint8_t st_len = 0;
    int16_t rssi = 0, rssi_signal = 0;
    int8_t snr = 0;
    rc = lr_get_lora_packet_status(&st_len, &rssi, &rssi_signal, &snr);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(st_len, pkt_len, name);

    /* 4. FIFO read: exactly st_len (the authoritative length). The
     * current driver reads st_len bytes as ONE frame — no per-packet
     * drain (0x0212 resp[2] is a status byte, not a length — pitfall #34). */
    uint8_t buf[64] = { 0 };
    rc = lr_fifo_read(buf, st_len);
    CHECK_EQ(rc, 0, name);
    /* Compare byte 0 against expected packet header byte. */
    CHECK_EQ(buf[0], pkt[0], name);
    CHECK_EQ(g_stub.rx_fifo_len, 0u, name);

    /* 5. Re-arm — mirrors lr20xx_restart_rx (RadioLib startReceive
     * parity): clear FIFO + clear IRQ + SET_RX. The real driver also
     * re-applies dio_irq_cfg + lora_pkt_params before SET_RX. */
    rc = lr_clear_rx_fifo();
    CHECK_EQ(rc, 0, name);
    rc = lr_clear_irq(LR20XX_IRQ_ALL_MASK);
    CHECK_EQ(rc, 0, name);
    rc = lr_set_rx(0xFFFFFF);
    CHECK_EQ(rc, 0, name);

    PASS(name);
}

/* ── Test 6: IRQ echo regression — verify stub returns IRQ echo when ─
   ──── CS-toggle IS simulated. This guards against future edits ──
   ──── that re-introduce the two-phase read by accident. ──────────── */

static void test_stub_echo_when_cs_toggled(void)
{
    const char *name = "test_stub_echo_when_cs_toggled";
    stub_reset();
    stub_set_force_cs_toggle(true);
    stub_fake_irq_fire_rx_done();

    /* In this mode our stub deliberately returns the IRQ word in place
     * of the real response — exactly the way the real broken driver
     * captured it on the wire. */
    uint32_t irq = 0;
    int rc = lr_get_irq_status(&irq);
    CHECK_EQ(rc, 0, name);
    /* Should read whatever the IRQ word was (RX_DONE = bit 1). */
    CHECK((irq & LR20XX_IRQ_RX_DONE),
          name, "irq word did not contain RX_DONE after force_cs_toggle");

    stub_set_force_cs_toggle(false);  /* cleanup so test 7 starts clean */
    PASS(name);
}

/* ── Test 7: TX_DONE flow ──────────────────────────────────────────── */

static void test_tx_done_flow(void)
{
    const char *name = "test_tx_done_flow";
    stub_reset();

    /* Begin TX, fire TX_DONE IRQ later. */
    lr_set_tx(0xFFFFFF);
    stub_fake_irq_fire_tx_done();

    uint32_t irq = 0;
    int rc = lr_get_irq_status(&irq);
    CHECK_EQ(rc, 0, name);
    CHECK((irq & LR20XX_IRQ_TX_DONE), name,
          "TX_DONE not observed, got irq=0x%08X", (unsigned)irq);
    CHECK_EQ(g_stub.irq_pending, 0u, name);
    CHECK_EQ(g_stub.dio_pin_high, false, name);

    PASS(name);
}

/* ── Test 8: RX-order regression (the 433499d bug) ───────────────────
 * Models the live quirk: after GetAndClearIrq the NEXT read returns
 * the IRQ-word echo (one transaction behind). The 433499d reorder
 * (GetLoRaPacketStatus FIRST) fed that echo into st_len — observed on
 * real HW as st_len == (irq>>16) = 4/6 — and truncated every inbound
 * packet ("incomplete packet"). The fixed order (GetRxPktLength first,
 * GetLoRaPacketStatus second) must still end up with the full frame. */

static void test_rx_order_regression(void)
{
    const char *name = "test_rx_order_regression";
    stub_reset();
    stub_set_force_cs_toggle(true);   /* one-transaction-behind quirk ON */

    uint8_t pkt[22];
    for (size_t i = 0; i < sizeof(pkt); i++) pkt[i] = (uint8_t)(i * 7 + 11);
    const uint16_t real_len = (uint16_t)sizeof(pkt);

    /* ── Phase A: BROKEN order (433499d) must be caught ── */
    stub_inject_packet(pkt, sizeof(pkt));
    lr_set_rx(0xFFFFFF);
    stub_fake_irq_fire_rx_done();

    uint32_t irq = 0;
    int rc = lr_get_irq_status(&irq);   /* GetAndClearIrq; arms one-shot echo */
    CHECK_EQ(rc, 0, name);
    CHECK((irq & LR20XX_IRQ_RX_DONE), name, "RX_DONE not observed");

    /* status FIRST → gets the echo → st_len must NOT be the real length */
    uint8_t st_bad = 0;
    int16_t r1 = 0, r2 = 0;
    int8_t s = 0;
    rc = lr_get_lora_packet_status(&st_bad, &r1, &r2, &s);
    CHECK_EQ(rc, 0, name);
    CHECK(st_bad != real_len, name,
          "quirk not modeled: st_len=%u == real len (echo missing)", st_bad);
    CHECK_EQ(st_bad, (uint8_t)((irq >> 16) & 0xFF), name);

    /* ── Phase B: FIXED order must read the full frame ── */
    stub_inject_packet(pkt, sizeof(pkt));   /* refill FIFO for phase B */
    stub_fake_irq_fire_rx_done();
    rc = lr_get_irq_status(&irq);           /* arms one-shot echo again */
    CHECK_EQ(rc, 0, name);

    /* RxLen FIRST → echo value (diagnostic only) */
    uint16_t pkt_len_raw = 0;
    rc = lr_get_rx_packet_length(&pkt_len_raw);
    CHECK_EQ(rc, 0, name);
    CHECK(pkt_len_raw != real_len, name,
          "expected IRQ echo in raw length, got %u", (unsigned)pkt_len_raw);

    /* status SECOND → real st_len → overrides pkt_len */
    uint8_t st_ok = 0;
    rc = lr_get_lora_packet_status(&st_ok, &r1, &r2, &s);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(st_ok, real_len, name);

    /* FIFO read exactly st_len → full frame, FIFO drained */
    uint8_t buf[64] = { 0 };
    rc = lr_fifo_read(buf, st_ok);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(buf[0], pkt[0], name);
    CHECK_EQ(g_stub.rx_fifo_len, 0u, name);

    stub_set_force_cs_toggle(false);        /* cleanup for any later tests */
    PASS(name);
}

/* ── Test 9: bundle (late read) — st_len reports FIFO TOTAL ──────────
 * The peer's dispatcher TXes ACK + queued message back-to-back; when our
 * deferred handler reads late, BOTH packets are in the FIFO and st_len
 * = 22 (FIFO total), NOT the first packet's 8. The whole FIFO is read
 * as ONE frame — this is why the mesh-level split (Dispatcher::checkRecv,
 * 43bf154) exists. Guards against re-introducing the dead drain idea:
 * 0x0212 resp[2] is a STATUS byte (0x14 = 20), not a length. */

static void test_bundle_reports_fifo_total(void)
{
    const char *name = "test_bundle_reports_fifo_total";
    stub_reset();

    /* Bundle = 8-byte V1 flood ACK + 14-byte TXT_MSG (observed live). */
    const uint8_t ack_pkt[8] = { 0x0D, 0x00, 0x4F, 0x58, 0x34, 0x98, 0x00, 0x05 };
    const uint8_t msg_pkt[14] = { 0x09, 0x00, 0xA7, 0xF4, 0xC9, 0xDF, 0x39, 0x2C,
                                  0xCB, 0x55, 0x9E, 0x8C, 0x45, 0xC6 };
    uint8_t bundle[sizeof(ack_pkt) + sizeof(msg_pkt)];
    memcpy(bundle, ack_pkt, sizeof(ack_pkt));
    memcpy(bundle + sizeof(ack_pkt), msg_pkt, sizeof(msg_pkt));
    stub_inject_packet(bundle, sizeof(bundle));

    lr_set_rx(0xFFFFFF);
    stub_fake_irq_fire_rx_done();

    uint32_t irq = 0;
    int rc = lr_get_irq_status(&irq);
    CHECK_EQ(rc, 0, name);
    CHECK((irq & LR20XX_IRQ_RX_DONE), name, "RX_DONE not observed");

    /* 1st read (diag): GetRxPktLength 16-bit parse — raw_len = 0x1416
     * (status 0x14 << 8 | len 22). resp[2]=0x14 is NOT the length. */
    uint16_t pkt_len_raw = 0;
    rc = lr_get_rx_packet_length(&pkt_len_raw);
    CHECK_EQ(rc, 0, name);
    CHECK((pkt_len_raw >> 8) == 0x14u, name,
          "0x0212 resp[2] must be STATUS 0x14, got 0x%02X",
          (unsigned)(pkt_len_raw >> 8));
    CHECK((pkt_len_raw & 0xFFu) == (uint16_t)sizeof(bundle), name,
          "0x0212 resp[3] must be FIFO total 22, got %u",
          (unsigned)(pkt_len_raw & 0xFFu));

    /* 2nd read: GetLoRaPacketStatus — st_len = REMAINING FIFO total (22),
     * NOT the first packet's 8 B. */
    uint8_t st_len = 0;
    int16_t r1 = 0, r2 = 0;
    int8_t s = 0;
    rc = lr_get_lora_packet_status(&st_len, &r1, &r2, &s);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(st_len, (uint8_t)sizeof(bundle), name);

    /* Current driver reads the WHOLE FIFO as one frame (st_len bytes). */
    uint8_t buf[64] = { 0 };
    rc = lr_fifo_read(buf, st_len);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(buf[0], ack_pkt[0], name);
    CHECK_EQ(g_stub.rx_fifo_len, 0u, name);

    PASS(name);
}

/* ── Test 10: upstream parity — prompt read delivers EACH packet ─────
 * RadioLib reads ONE packet per RX_DONE immediately (ISR/main-loop, µs);
 * the chip re-asserts DIO for packets remaining in the FIFO. When the
 * handler is prompt, the FIFO holds packet 1 alone (st_len=8), we read
 * it, clear + re-arm, and packet 2 arrives as its OWN RX_DONE (st_len=14).
 * This is the timing to copy from upstream — no bundles ever form. */

static void test_upstream_parity_sequential_rx(void)
{
    const char *name = "test_upstream_parity_sequential_rx";
    stub_reset();

    const uint8_t ack_pkt[8] = { 0x0D, 0x00, 0x4F, 0x58, 0x34, 0x98, 0x00, 0x05 };
    const uint8_t msg_pkt[14] = { 0x09, 0x00, 0xA7, 0xF4, 0xC9, 0xDF, 0x39, 0x2C,
                                  0xCB, 0x55, 0x9E, 0x8C, 0x45, 0xC6 };

    lr_set_rx(0xFFFFFF);

    /* ── Packet 1 arrives alone (handler is prompt — FIFO has only 8 B) ── */
    stub_inject_packet(ack_pkt, sizeof(ack_pkt));
    stub_fake_irq_fire_rx_done();

    uint32_t irq = 0;
    int rc = lr_get_irq_status(&irq);
    CHECK_EQ(rc, 0, name);
    CHECK((irq & LR20XX_IRQ_RX_DONE), name, "RX_DONE (pkt1) not observed");

    uint16_t pkt_len_raw = 0;
    rc = lr_get_rx_packet_length(&pkt_len_raw);   /* diag (echo in real life) */
    CHECK_EQ(rc, 0, name);

    uint8_t st_len = 0;
    int16_t r1 = 0, r2 = 0;
    int8_t s = 0;
    rc = lr_get_lora_packet_status(&st_len, &r1, &r2, &s);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(st_len, (uint8_t)sizeof(ack_pkt), name);  /* 8, not 22 */

    uint8_t buf[64] = { 0 };
    rc = lr_fifo_read(buf, st_len);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(buf[0], ack_pkt[0], name);
    CHECK_EQ(g_stub.rx_fifo_len, 0u, name);

    /* Re-arm — mirrors lr20xx_restart_rx (RadioLib startReceive parity):
     * clear FIFO + clear IRQ + SET_RX, so packet 2 arrives as its own
     * RX_DONE with a clean FIFO (st_len reports only its own length). */
    rc = lr_clear_rx_fifo();
    CHECK_EQ(rc, 0, name);
    rc = lr_clear_irq(LR20XX_IRQ_ALL_MASK);
    CHECK_EQ(rc, 0, name);
    rc = lr_set_rx(0xFFFFFF);
    CHECK_EQ(rc, 0, name);

    /* ── Packet 2 arrives later — its own RX_DONE, read alone ── */
    stub_inject_packet(msg_pkt, sizeof(msg_pkt));
    stub_fake_irq_fire_rx_done();

    rc = lr_get_irq_status(&irq);
    CHECK_EQ(rc, 0, name);
    CHECK((irq & LR20XX_IRQ_RX_DONE), name, "RX_DONE (pkt2) not observed");

    rc = lr_get_rx_packet_length(&pkt_len_raw);   /* diag */
    CHECK_EQ(rc, 0, name);
    rc = lr_get_lora_packet_status(&st_len, &r1, &r2, &s);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(st_len, (uint8_t)sizeof(msg_pkt), name);  /* 14, not 22 */

    memset(buf, 0, sizeof(buf));
    rc = lr_fifo_read(buf, st_len);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(buf[0], msg_pkt[0], name);
    CHECK_EQ(buf[13], msg_pkt[13], name);
    CHECK_EQ(g_stub.rx_fifo_len, 0u, name);

    PASS(name);
}

/* ── Test: MeshCore/RadioLib RX flow vs current flow on a REAL burst ── */

/*
 * Real frames from boot_log_splitfix.txt (2026-08-01, build 05806cf):
 * the peer sends its ACK (8 B) and its next TXT_MSG (22 B, 10 B data —
 * the same frame that decrypts fine when it arrives standalone)
 * back-to-back. Live chip reports GetLoRaPacketStatus = 22 (LAST
 * packet) while the FIFO holds 30 B. The current driver reads 22 B
 * from FIFO base → ACK + first 14 B of the msg (2 B data → can NEVER
 * decrypt). The MeshCore/RadioLib flow reads the whole 30 B → the
 * Dispatcher split recovers the FULL 22 B msg.
 */
static void test_meshcore_flow_burst(void)
{
    const char *name = "test_meshcore_flow_burst";

    static const uint8_t ack8[] = {
        0x0e, 0x00, 0xdc, 0x87, 0x03, 0x4e, 0x00, 0xa2,
    };
    static const uint8_t msg22[] = {
        0x0a, 0x00, 0xa7, 0xf4, 0x3a, 0x19, 0x41, 0x79,
        0xae, 0xed, 0xa7, 0xe0, 0x79, 0x1d, 0xe8, 0x8b,
        0xee, 0xf1, 0x34, 0x7e, 0xa5, 0xe5,
    };
    uint8_t burst[30];
    memcpy(burst, ack8, 8);
    memcpy(burst + 8, msg22, 22);

    uint8_t buf[64] = { 0 };
    size_t n = 0;
    int rc = 0;

    /* ── 1. CURRENT flow on the burst: reproduces the live truncation ── */
    stub_reset();
    stub_set_force_cs_toggle(true);      /* live one-transaction-behind */
    stub_inject_packet(burst, sizeof(burst));
    stub_set_rx_status_len(22);          /* live: status reports LAST pkt */
    stub_fake_irq_fire_rx_done();

    n = 0;
    rc = lr_rx_flow_current(buf, sizeof(buf), &n);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(n, 22u, name);              /* reads only st_len, not 30 */
    CHECK(memcmp(buf, ack8, 8) == 0, name, "ACK prefix present");
    CHECK(memcmp(buf + 8, msg22, 14) == 0, name, "msg[0..13] present");
    CHECK(buf[22] == 0, name, "msg[14..21] NOT read (truncated)");

    /* ── 2. MESH CORE flow on the same burst: full 30 B recovered ── */
    stub_reset();
    stub_set_force_cs_toggle(true);
    stub_inject_packet(burst, sizeof(burst));
    stub_set_rx_status_len(22);
    stub_fake_irq_fire_rx_done();

    n = 0;
    rc = lr_rx_flow_meshcore(buf, sizeof(buf), &n);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(n, 30u, name);              /* WHOLE FIFO */
    CHECK(memcmp(buf, ack8, 8) == 0, name, "ACK first");
    CHECK(memcmp(buf + 8, msg22, 22) == 0, name,
          "FULL msg recovered (22 B, 10 B data → decryptable)");

    /* ── 3. Regression: single 22 B msg — both flows read 22 B ── */
    stub_reset();
    stub_set_force_cs_toggle(true);
    stub_inject_packet(msg22, sizeof(msg22));
    stub_set_rx_status_len(22);
    stub_fake_irq_fire_rx_done();

    n = 0;
    rc = lr_rx_flow_meshcore(buf, sizeof(buf), &n);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(n, 22u, name);
    CHECK(memcmp(buf, msg22, 22) == 0, name, "single msg intact");

    /* ── 4. Regression: single 8 B ACK — both flows read 8 B ── */
    stub_reset();
    stub_set_force_cs_toggle(true);
    stub_inject_packet(ack8, sizeof(ack8));
    stub_set_rx_status_len(8);
    stub_fake_irq_fire_rx_done();

    n = 0;
    rc = lr_rx_flow_meshcore(buf, sizeof(buf), &n);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(n, 8u, name);
    CHECK(memcmp(buf, ack8, 8) == 0, name, "single ACK intact");

    PASS(name);
}

/* ── Test: start_rx edge-race re-poll (DIO8 re-check port,
 * ── 2026-08-02) ───────────────────────────────────────────────────── */

/*
 * Scenario (from boot_log_ackfix.txt, 7/10 ACKs lost): the peer's ACK
 * arrives during the TX->RX re-arm window of lr20xx_start_rx — the
 * 3 ms msleep after SET_RX. The chip asserts RX_DONE -> DIO8 HIGH. If
 * the DIO work item was already running (TX_DONE handler path) the ISR's
 * k_work_submit is dropped (-EALREADY); the second lr_clear_irq then
 * consumes the edge and the ACK sits unread in the FIFO. The re-poll at
 * the end of start_rx must catch the still-HIGH line and re-submit the
 * work item by hand so the handler re-reads the chip IRQ register.
 */
static void test_start_rx_edge_recheck(void)
{
    const char *name = "test_start_rx_edge_recheck";
    stub_reset();

    /* (B) SET_RX re-arm, then the ACK lands during the 3 ms sleep. */
    lr_set_rx(0xFFFFFF);
    stub_fake_irq_fire_rx_done();

    /* (D) the second lr_clear_irq clears the IRQ register — but the chip
     * keeps DIO8 asserted while an unread packet remains in the FIFO
     * (observed stuck-DIO behaviour; the premise of the port). */
    lr_clear_irq(LR20XX_IRQ_ALL_MASK);
    stub_force_dio_pin(true);

    /* The re-poll must re-submit the DIO work item. */
    CHECK_EQ(g_stub.work_resubmits, 0u, name);
    CHECK_EQ(lr20xx_start_rx_edge_recheck(), 1, name);
    CHECK_EQ(g_stub.work_resubmits, 1u, name);

    /* Line LOW (edge consumed, nothing pending) -> no re-submit. */
    stub_force_dio_pin(false);
    CHECK_EQ(lr20xx_start_rx_edge_recheck(), 0, name);
    CHECK_EQ(g_stub.work_resubmits, 1u, name);

    /* Line HIGH again (e.g. a second frame) -> re-submit again. */
    stub_force_dio_pin(true);
    CHECK_EQ(lr20xx_start_rx_edge_recheck(), 1, name);
    CHECK_EQ(g_stub.work_resubmits, 2u, name);

    PASS(name);
}

/* ── main ───────────────────────────────────────────────────────────── */

/* ── Dual-band helpers (L1-U1, 2026-08-02) ──────────────────────────── */

static void test_dualband_band_split(void)
{
    /* RadioLib LF_CUTOFF = 1500 MHz; strictly-above → HF. */
    CHECK(!lr_is_hf(869618000u), "band_split_868_lf",
          "869.618 MHz must be LF");
    CHECK(!lr_is_hf(1499000000u), "band_split_1499_lf",
          "1499 MHz must be LF");
    CHECK(!lr_is_hf(1500000000u), "band_split_1500_lf",
          "exactly 1500 MHz is LF (not above)");
    CHECK(lr_is_hf(1500000001u), "band_split_1500p_hf",
          "1500.000001 MHz must be HF");
    CHECK(lr_is_hf(2400000000u), "band_split_2400_hf",
          "2400 MHz must be HF");
    CHECK(lr_is_hf(2450000000u), "band_split_2450_hf",
          "2450 MHz must be HF");
    PASS("dualband_band_split");
}

static void test_dualband_rx_path(void)
{
    uint8_t path = 0xFF, boost = 0xFF;

    /* HF + boost → HF path, HF boost (0x04) */
    lr_rx_path_for_freq(2450000000u, true, &path, &boost);
    CHECK(path == LR20XX_RX_PATH_HF, "rxpath_hf_boost_path",
          "HF path expected, got %u", path);
    CHECK(boost == LR20XX_RX_BOOST_HF, "rxpath_hf_boost_val",
          "HF boost 0x04 expected, got 0x%02X", boost);

    /* HF no boost → HF path, NONE */
    lr_rx_path_for_freq(2450000000u, false, &path, &boost);
    CHECK(path == LR20XX_RX_PATH_HF, "rxpath_hf_noboost_path",
          "HF path expected, got %u", path);
    CHECK(boost == LR20XX_RX_BOOST_NONE, "rxpath_hf_noboost_val",
          "boost NONE expected, got 0x%02X", boost);

    /* LF + boost → LF path, LF boost (0x01) */
    lr_rx_path_for_freq(869618000u, true, &path, &boost);
    CHECK(path == LR20XX_RX_PATH_LF, "rxpath_lf_boost_path",
          "LF path expected, got %u", path);
    CHECK(boost == LR20XX_RX_BOOST_LF, "rxpath_lf_boost_val",
          "LF boost 0x01 expected, got 0x%02X", boost);

    /* LF no boost → LF path, NONE */
    lr_rx_path_for_freq(869618000u, false, &path, &boost);
    CHECK(path == LR20XX_RX_PATH_LF, "rxpath_lf_noboost_path",
          "LF path expected, got %u", path);
    CHECK(boost == LR20XX_RX_BOOST_NONE, "rxpath_lf_noboost_val",
          "boost NONE expected, got 0x%02X", boost);
    PASS("dualband_rx_path");
}

static void test_dualband_hf_pa_duty(void)
{
    /* hf_duty field = paOptTableHf duty + PA_HF_DUTY_UNUSED(16). */
    CHECK(lr_pa_hf_duty_for_power(12) == (0 + 16),
          "hf_pa_12", "+12 → 0+16=16, got %u", lr_pa_hf_duty_for_power(12));
    CHECK(lr_pa_hf_duty_for_power(11) == (10 + 16),
          "hf_pa_11", "+11 → 10+16=26, got %u", lr_pa_hf_duty_for_power(11));
    CHECK(lr_pa_hf_duty_for_power(10) == (14 + 16),
          "hf_pa_10", "+10 → 14+16=30, got %u", lr_pa_hf_duty_for_power(10));
    CHECK(lr_pa_hf_duty_for_power(9) == (14 + 16),
          "hf_pa_9", "+9 → 14+16=30, got %u", lr_pa_hf_duty_for_power(9));
    CHECK(lr_pa_hf_duty_for_power(8) == (15 + 16),
          "hf_pa_8", "+8 → 15+16=31, got %u", lr_pa_hf_duty_for_power(8));
    CHECK(lr_pa_hf_duty_for_power(7) == (14 + 16),
          "hf_pa_7", "<+8 default → 14+16=30, got %u", lr_pa_hf_duty_for_power(7));
    PASS("dualband_hf_pa_duty");
}

static void test_dualband_hf_power_clamp(void)
{
    CHECK(lr_clamp_hf_power(22) == 12, "hf_clamp_22",
          "22 dBm → clamp 12, got %d", lr_clamp_hf_power(22));
    CHECK(lr_clamp_hf_power(12) == 12, "hf_clamp_12",
          "12 → 12, got %d", lr_clamp_hf_power(12));
    CHECK(lr_clamp_hf_power(10) == 10, "hf_clamp_10",
          "10 → 10, got %d", lr_clamp_hf_power(10));
    CHECK(lr_clamp_hf_power(-1) == -1, "hf_clamp_neg",
          "-1 → -1, got %d", lr_clamp_hf_power(-1));
    PASS("dualband_hf_power_clamp");
}

/*
 * SPDX-License-Identifier: MIT
 * TDM scheduler state machine (L3-U4B, 2026-08-02)
 *
 * dualband_tdm.h is the PRODUCTION pure-logic header (DualBandRadio
 * adapter) — the sandbox #includes it directly so the tested code is the
 * exact code that runs on the target (pitfall #21: no mirror copy).
 */
#include "../../zephcore/adapters/radio/dualband_tdm.h"
#include "../../zephcore/adapters/radio/dualband_route.h"
#include "../../zephcore/adapters/radio/dualband_beacon.h"

static void test_tdm_scheduler(void)
{
    dm_config_t cfg = dm_default_config();

    /* Timing defaults per plan L3-U4B: 70 ms window every 1-2 s. */
    CHECK_EQ(cfg.hf_window_ms, 70u, "tdm_cfg_window");
    CHECK_EQ(cfg.hf_extend_step_ms, 25u, "tdm_cfg_extend_step");
    CHECK_EQ(cfg.hf_max_extend_ms, 150u, "tdm_cfg_max_extend");
    CHECK_EQ(cfg.hold_ms, 1500u, "tdm_cfg_hold");
    CHECK_EQ(DM_SKIP_RETRY_MS, 125u, "tdm_cfg_skip_retry");

    /* HF (secondary) preset — plan §1.4. */
    CHECK_EQ(DM_HF_FREQ_HZ, 2450000000UL, "tdm_hf_freq");
    CHECK_EQ(DM_HF_BW_KHZ, 500u, "tdm_hf_bw");
    CHECK_EQ(DM_HF_SF, 8u, "tdm_hf_sf");
    CHECK_EQ(DM_HF_CR, 1u, "tdm_hf_cr");      /* CR_4_5 == chip code 0x01 */
    CHECK_EQ(DM_HF_TX_PWR_DM, 12, "tdm_hf_pwr");

    /* IDLE, radio idle (in RX, not mid capture) → open the HF window. */
    CHECK(dm_step(DM_STATE_IDLE, true, false, 0, &cfg) == DM_DECISION_OPEN,
          "tdm_idle_idle_open",
          "IDLE + radio idle → OPEN, got %d", dm_step(DM_STATE_IDLE, true, false, 0, &cfg));

    /* IDLE but radio NOT in RX (mesh TX active) → skip this cycle. */
    CHECK(dm_step(DM_STATE_IDLE, false, false, 0, &cfg) == DM_DECISION_SKIP,
          "tdm_idle_tx_skip",
          "IDLE + no RX (TX active) → SKIP, got %d", dm_step(DM_STATE_IDLE, false, false, 0, &cfg));

    /* IDLE but mid sub-GHz capture → skip (don't chop a primary packet). */
    CHECK(dm_step(DM_STATE_IDLE, true, true, 0, &cfg) == DM_DECISION_SKIP,
          "tdm_idle_recv_skip",
          "IDLE + mid capture → SKIP, got %d", dm_step(DM_STATE_IDLE, true, true, 0, &cfg));

    /* HF_OPEN, HF idle → close (nominal window done). */
    CHECK(dm_step(DM_STATE_HF_OPEN, true, false, 70u, &cfg) == DM_DECISION_CLOSE,
          "tdm_open_idle_close",
          "HF_OPEN + idle → CLOSE, got %d", dm_step(DM_STATE_HF_OPEN, true, false, 70u, &cfg));

    /* HF_OPEN, packet mid-capture, within cap → extend. */
    CHECK(dm_step(DM_STATE_HF_OPEN, true, true, 70u, &cfg) == DM_DECISION_EXTEND,
          "tdm_open_busy_extend",
          "HF_OPEN + receiving @70ms <150ms cap → EXTEND, got %d",
          dm_step(DM_STATE_HF_OPEN, true, true, 70u, &cfg));

    /* HF_OPEN, still receiving but cap reached → close anyway. */
    CHECK(dm_step(DM_STATE_HF_OPEN, true, true, 150u, &cfg) == DM_DECISION_CLOSE,
          "tdm_open_busy_cap_close",
          "HF_OPEN + receiving @150ms == cap → CLOSE, got %d",
          dm_step(DM_STATE_HF_OPEN, true, true, 150u, &cfg));

    /* HF_EXTEND, capture cleared → close. */
    CHECK(dm_step(DM_STATE_HF_EXTEND, true, false, 95u, &cfg) == DM_DECISION_CLOSE,
          "tdm_extend_clear_close",
          "HF_EXTEND + RX clear → CLOSE, got %d", dm_step(DM_STATE_HF_EXTEND, true, false, 95u, &cfg));

    /* HF_EXTEND, still capturing, within cap → extend again. */
    CHECK(dm_step(DM_STATE_HF_EXTEND, true, true, 95u, &cfg) == DM_DECISION_EXTEND,
          "tdm_extend_busy_extend",
          "HF_EXTEND + still receiving @95ms → EXTEND, got %d",
          dm_step(DM_STATE_HF_EXTEND, true, true, 95u, &cfg));

    /* HF_EXTEND, still capturing, cap reached → close (hard cap wins). */
    CHECK(dm_step(DM_STATE_HF_EXTEND, true, true, 150u, &cfg) == DM_DECISION_CLOSE,
          "tdm_extend_busy_cap_close",
          "HF_EXTEND + receiving @150ms cap → CLOSE, got %d",
          dm_step(DM_STATE_HF_EXTEND, true, true, 150u, &cfg));

    /* Unknown state → close (safe default: back to primary). */
    CHECK(dm_step((dm_state_t)99, true, false, 0, &cfg) == DM_DECISION_CLOSE,
          "tdm_unknown_close",
          "unknown state → CLOSE, got %d", dm_step((dm_state_t)99, true, false, 0, &cfg));

    PASS("tdm_scheduler");
}

/* ── TDM band-switch pure helpers (L3-U4, 2026-08-02) ──────────────── */

static void test_tdm_single_cal_bin(void)
{
    /* RadioLib: (freq/4)+0.5 → nearest 4 MHz bin; |=0x8000 when HF.
     * 2450.0 → 613 (2452 MHz bin), HF marker → 0x0265|0x8000 = 0x8265. */
    CHECK(lr_cal_fe_single_bin_hz(2450000000u) == 0x8265u,
          "tdm_bin_hf_2450",
          "2450 MHz → 0x8265, got 0x%04X", lr_cal_fe_single_bin_hz(2450000000u));
    /* 869.618 → 217 (LF), no marker → 0x00D9. */
    CHECK(lr_cal_fe_single_bin_hz(869618000u) == 0x00D9u,
          "tdm_bin_lf_869618",
          "869.618 MHz → 0x00D9, got 0x%04X", lr_cal_fe_single_bin_hz(869618000u));
    /* 1500 MHz boundary: NOT HF (strictly above) → no marker. */
    CHECK(lr_cal_fe_single_bin_hz(1500000000u) == 0x0177u,
          "tdm_bin_1500_lf",
          "1500 MHz → 0x0177 (no HF marker), got 0x%04X",
          lr_cal_fe_single_bin_hz(1500000000u));
    CHECK(lr_cal_fe_single_bin_hz(1501000000u) == 0x8177u,
          "tdm_bin_1501_hf",
          "1501 MHz → 0x8177 (HF marker), got 0x%04X",
          lr_cal_fe_single_bin_hz(1501000000u));
    PASS("tdm_single_cal_bin");
}

static void test_tdm_switch_needs_cal(void)
{
    /* RadioLib trigger: |Δf| >= 20 MHz. */
    CHECK(lr_band_switch_needs_cal(869618000u, 2450000000u),
          "tdm_cal_sub_to_hf",
          "sub-GHz → HF must need cal (Δ~1.58 GHz)");
    CHECK(lr_band_switch_needs_cal(2450000000u, 869618000u),
          "tdm_cal_hf_to_sub",
          "HF → sub-GHz must need cal");
    CHECK(!lr_band_switch_needs_cal(869618000u, 869620000u),
          "tdm_cal_small_delta_false",
          "2 kHz Δ must NOT need cal");
    CHECK(lr_band_switch_needs_cal(869618000u, 889618000u),
          "tdm_cal_exact_20mhz",
          "exactly 20 MHz Δ → needs cal (>=)");
    CHECK(!lr_band_switch_needs_cal(2450000000u, 2450010000u),
          "tdm_cal_hf_tiny_false",
          "0.01 MHz Δ on HF → no cal");
    CHECK(!lr_band_switch_needs_cal(869618000u, 869618000u),
          "tdm_cal_same_freq",
          "same freq → no cal");
    PASS("tdm_switch_needs_cal");
}

/* ── L4-U5: TX band routing decision (dualband_route.h, production header) ── */

static void test_db_route_mask(void)
{
    /* Flood/discovery → BOTH regardless of next hop */
    CHECK_EQ(db_tx_band_mask(true, DB_BAND_NONE), DB_BAND_BOTH,
             "db_route_flood_unknown_both");
    CHECK_EQ(db_tx_band_mask(true, DB_BAND_HF), DB_BAND_BOTH,
             "db_route_flood_hf_both");
    CHECK_EQ(db_tx_band_mask(true, DB_BAND_SUBGHZ), DB_BAND_BOTH,
             "db_route_flood_subghz_both");

    /* Direct, known next hop → that band ONLY */
    CHECK_EQ(db_tx_band_mask(false, DB_BAND_HF), DB_BAND_HF,
             "db_route_direct_hf_only");
    CHECK_EQ(db_tx_band_mask(false, DB_BAND_SUBGHZ), DB_BAND_SUBGHZ,
             "db_route_direct_subghz_only");

    /* Direct, unknown next hop → BOTH (fallback) */
    CHECK_EQ(db_tx_band_mask(false, DB_BAND_NONE), DB_BAND_BOTH,
             "db_route_direct_unknown_both");

    /* Mask constants sane */
    CHECK_EQ(DB_BAND_BOTH, (DB_BAND_SUBGHZ | DB_BAND_HF), "db_route_both_mask");
    CHECK(DB_BAND_NONE == 0, "db_route_none_zero", "DB_BAND_NONE must be 0");

    /* db_tx_band_mask never returns NONE */
    CHECK(db_tx_band_mask(false, DB_BAND_NONE) != DB_BAND_NONE,
          "db_route_never_none", "unknown direct must degrade to BOTH");

    PASS("db_route_mask");
}

/* ── L4-U6: HF beacon payload (dualband_beacon.h, production header) ── */

static void test_hf_beacon(void)
{
    db_beacon_params_t p;
    uint8_t buf[32];

    /* Preset defaults — fixed secondary band (plan §1.4 / L3-U4B). */
    db_beacon_defaults(&p);
    CHECK_EQ(DB_BEACON_ADV_TYPE, 5u, "beacon_adv_type_not_repeater");
    CHECK_EQ(p.freq_hz, 2450000000UL, "beacon_default_freq");
    CHECK_EQ(p.bw_khz, 500u, "beacon_default_bw");
    CHECK_EQ(p.sf, 8u, "beacon_default_sf");
    CHECK_EQ(p.cr, DB_BEACON_CR, "beacon_default_cr");
    CHECK_EQ(p.sync, 0x12u, "beacon_default_sync");
    CHECK_EQ(p.tx_pwr_dbm, 12, "beacon_default_pwr");
    CHECK_EQ(DB_BEACON_PERIOD_MS, 60000u, "beacon_period_60s");

    /* Encode → decode round-trip preserves every field. */
    p.freq_hz = 2425000000UL;  /* Semtech alt 2G4 channel */
    p.bw_khz = 500u;
    p.sf = 8u;
    p.cr = 1u;
    p.sync = 0x12u;
    p.tx_pwr_dbm = 12;
    p.load = 42;
    uint8_t n = db_beacon_encode(buf, sizeof(buf), &p);
    CHECK_EQ(n, DB_BEACON_PAYLOAD_SIZE, "beacon_encode_len");

    db_beacon_params_t q;
    memset(&q, 0, sizeof(q));
    CHECK(db_beacon_decode(buf, n, &q), "beacon_decode_valid",
          "decode of encoded payload failed");
    CHECK_EQ(q.freq_hz, p.freq_hz, "beacon_rt_freq");
    CHECK_EQ(q.bw_khz, p.bw_khz, "beacon_rt_bw");
    CHECK_EQ(q.sf, p.sf, "beacon_rt_sf");
    CHECK_EQ(q.cr, p.cr, "beacon_rt_cr");
    CHECK_EQ(q.sync, p.sync, "beacon_rt_sync");
    CHECK_EQ(q.tx_pwr_dbm, p.tx_pwr_dbm, "beacon_rt_pwr");
    CHECK_EQ(q.load, p.load, "beacon_rt_load");

    /* Buffer too small → no write (return 0). */
    CHECK_EQ(db_beacon_encode(buf, DB_BEACON_PAYLOAD_SIZE - 1, &p), 0,
             "beacon_encode_small_cap");

    /* Truncated payload → rejected. */
    CHECK(!db_beacon_decode(buf, DB_BEACON_PAYLOAD_SIZE - 1, &q),
          "beacon_decode_truncated", "truncated payload must be rejected");
    /* Wrong version → rejected. */
    buf[0] = 0xFF;
    CHECK(!db_beacon_decode(buf, DB_BEACON_PAYLOAD_SIZE, &q),
          "beacon_decode_bad_version", "bad version must be rejected");
    buf[0] = DB_BEACON_VERSION;

    PASS("hf_beacon");
}

/* TDM wedge mechanism + fix validation (ROOTCAUSE 2026-08-11) — defined
 * in test_tdm_wedge.c */
void run_tdm_wedge_tests(void);

/* On-chip rtl_433 OOK decode tests (test_rtl433.c) — writes through the
 * suite counters so failures fail the whole run. */
void run_rtl433_tests(int *tests_run, int *tests_failed);

/* ── Sniffer OOK poll-mode extension (lr20xx_lora.c sniffer fns) ───── */

static void test_sniffer_ook_arm_sequence(void)
{
    const char *name = "sniffer_ook_arm_sequence";
    stub_reset();

    int rc = lr_sniffer_ook_arm(433920000u, 10000u, 0x1Cu, 240u);
    CHECK_EQ(rc, 0, name);

    /* Command sequence: FE cal (869.618 → 433.92 is ≥ 20 MHz), freq,
     * pkt type OOK, OOK mod params, OOK packet params, sync word,
     * detector, RX path, DIO silenced, RX continuous. */
    CHECK_EQ(stub_cmd_count(0x0123), 1u, name);  /* CalibrateFrontEnd */
    CHECK_EQ(stub_cmd_count(0x0200), 1u, name);  /* SetRfFrequency */
    CHECK_EQ(stub_cmd_count(0x0207), 1u, name);  /* SetPacketType */
    CHECK_EQ(stub_cmd_count(0x0281), 1u, name);  /* SetOokModulationParams */
    CHECK_EQ(stub_cmd_count(0x0282), 1u, name);  /* SetOokPacketParams */
    CHECK_EQ(stub_cmd_count(0x0284), 1u, name);  /* SetOokSyncWord */
    CHECK_EQ(stub_cmd_count(0x0288), 1u, name);  /* SetOokDetector */
    CHECK_EQ(stub_cmd_count(0x0201), 1u, name);  /* SetRxPath */
    CHECK_EQ(stub_cmd_count(0x0115), 1u, name);  /* SetDioIrqCfg */
    CHECK_EQ(stub_cmd_count(0x020C), 1u, name);  /* SetRx */

    /* DIO mask silenced (last SetDioIrqCfg carried mask 0): mosi of the
     * SetRx transaction is not inspectable post-hoc, but the DIO cfg
     * params were [dio][00 00 00 00] — re-run a bare cfg to compare. */
    CHECK_EQ(lr_set_dio_irq_cfg(LR20XX_DIO_8, 0), 0, name);
    CHECK_EQ(g_stub.mosi[2], LR20XX_DIO_8, name);
    CHECK_EQ(g_stub.mosi[3], 0x00, name);
    CHECK_EQ(g_stub.mosi[4], 0x00, name);
    CHECK_EQ(g_stub.mosi[5], 0x00, name);
    CHECK_EQ(g_stub.mosi[6], 0x00, name);

    /* Same-band re-arm must NOT re-run the FE cal. */
    stub_reset();
    rc = lr_sniffer_ook_arm(433920000u, 10000u, 0x1Cu, 240u);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(stub_cmd_count(0x0123), 0u, name);

    /* Bitrate rides SetOokModulationParams big-endian (bit31 = 0 → bps).
     * Verify via a bare call: 10000 bps = 00 00 27 10. */
    stub_reset();
    lr_set_rf_frequency(433920000u);
    CHECK_EQ(g_stub.mosi[0], 0x02, name);
    CHECK_EQ(g_stub.mosi[1], 0x00, name);
    CHECK_EQ(g_stub.mosi[2], (uint8_t)(433920000u >> 24), name);
    CHECK_EQ(g_stub.mosi[3], (uint8_t)(433920000u >> 16), name);
    CHECK_EQ(g_stub.mosi[4], (uint8_t)(433920000u >> 8), name);
    CHECK_EQ(g_stub.mosi[5], (uint8_t)(433920000u >> 0), name);

    PASS(name);
}

static void test_sniffer_ook_poll_delivers_fifo(void)
{
    const char *name = "sniffer_ook_poll_delivers_fifo";
    stub_reset();

    /* Nothing pending → clean zero. */
    uint8_t buf[64];
    uint16_t len = 0;
    int16_t rssi = 0;
    CHECK_EQ(lr_sniffer_ook_poll(buf, sizeof(buf), &len, &rssi), 0, name);
    CHECK_EQ(len, 0u, name);

    /* Injected packet + RX_DONE → bytes delivered, re-armed. */
    uint8_t pkt[5] = { 0xAA, 0x0F, 0x1E, 0x2D, 0x00 };
    stub_inject_packet(pkt, sizeof(pkt));
    stub_fake_irq_fire_rx_done();
    CHECK_EQ(lr_sniffer_ook_poll(buf, sizeof(buf), &len, &rssi), 0, name);
    CHECK_EQ(len, sizeof(pkt), name);
    CHECK(memcmp(buf, pkt, sizeof(pkt)) == 0, name,
          "fifo bytes must match the injected packet");
    /* Re-arm happened after the read. */
    CHECK_EQ(stub_cmd_count(0x011E), 1u, name);  /* ClearRxFifo */
    CHECK_EQ(stub_cmd_count(0x020C), 1u, name);  /* SetRx again */

    /* FIFO drained → next poll returns nothing. */
    CHECK_EQ(lr_sniffer_ook_poll(buf, sizeof(buf), &len, &rssi), 0, name);
    CHECK_EQ(len, 0u, name);

    PASS(name);
}

/* OOK poll MUST survive the one-transaction-behind IRQ echo quirk
 * (force_cs_toggle): the FIRST status read after GetAndClearIrq returns
 * the IRQ word instead of its real response, so a single-read length
 * poll parses the echo as the length, zeroes out and silently DROPS
 * the packet (the acceptance criterion "zero silent packet losses").
 * The proven 3-read dance must still deliver every byte. */
static void test_sniffer_ook_poll_survives_irq_echo(void)
{
    const char *name = "sniffer_ook_poll_survives_irq_echo";
    stub_reset();
    stub_set_force_cs_toggle(true);

    uint8_t pkt[6] = { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC };
    stub_inject_packet(pkt, sizeof(pkt));
    stub_fake_irq_fire_rx_done();

    uint8_t buf[64];
    uint16_t len = 0;
    int16_t rssi = 0;
    CHECK_EQ(lr_sniffer_ook_poll(buf, sizeof(buf), &len, &rssi), 0, name);
    CHECK_EQ(len, sizeof(pkt), name);
    CHECK(memcmp(buf, pkt, sizeof(pkt)) == 0, name,
          "fifo bytes must match the injected packet");
    /* RSSI comes from the REAL GetOokPacketStatus (the echo was eaten by
     * the first GetRxPacketLength); stub models 0x5A → -45 dBm. */
    CHECK_EQ(rssi, -45, name);

    /* FIFO drained → next poll returns nothing. */
    len = 0;
    CHECK_EQ(lr_sniffer_ook_poll(buf, sizeof(buf), &len, &rssi), 0, name);
    CHECK_EQ(len, 0u, name);

    stub_set_force_cs_toggle(false);   /* cleanup so later tests start clean */
    PASS(name);
}

/* GetOokRxStats must parse the 6-byte payload (RadioLib parity): pkt_rx,
 * crc_error, len_error.  The pre-fix 18-byte parse read beyond the
 * response and reported non-existent pbl_det/sync_ok/sync_fail zeros. */
static void test_sniffer_ook_stats_6byte_parity(void)
{
    const char *name = "sniffer_ook_stats_6byte_parity";
    stub_reset();
    stub_set_ook_stats(0x1234, 0x0005, 0x0006);

    uint16_t rx = 0, crc = 0, len = 0;
    CHECK_EQ(lr_sniffer_ook_stats(&rx, &crc, &len), 0, name);
    CHECK_EQ(rx, 0x1234u, name);
    CHECK_EQ(crc, 0x0005u, name);
    CHECK_EQ(len, 0x0006u, name);

    /* Boundary: max counters survive (no sign/byte-order surprises). */
    stub_set_ook_stats(0xFFFF, 0x8001, 0x0000);
    CHECK_EQ(lr_sniffer_ook_stats(&rx, &crc, &len), 0, name);
    CHECK_EQ(rx, 0xFFFFu, name);
    CHECK_EQ(crc, 0x8001u, name);
    CHECK_EQ(len, 0x0000u, name);

    PASS(name);
}

/* GetRssiInst must parse the 9-bit raw (RadioLib parity): raw =
 * (buff[0]<<1)|(buff[1]>>7), power = -raw/2.  The pre-fix -(resp[2])
 * parse returned half the value and missed the 9th bit — this test
 * fails on that parse (negative control by construction). */
static void test_rssi_inst_9bit_parity(void)
{
    const char *name = "rssi_inst_9bit_parity";
    stub_reset();
    int16_t rssi = 0;

    /* raw 191 → -95.5 dBm → rounded -96.  Pre-fix: -(0x5F) = -95. */
    stub_set_rssi_inst_raw(191);
    CHECK_EQ(lr_get_rssi_inst(&rssi), 0, name);
    CHECK_EQ(rssi, -96, name);

    /* raw 88 (default floor model) → -44 dBm. */
    stub_set_rssi_inst_raw(88);
    CHECK_EQ(lr_get_rssi_inst(&rssi), 0, name);
    CHECK_EQ(rssi, -44, name);

    /* 9th bit set: raw 256+7=263 → -131.5 → -132.  Pre-fix: -(0x83) = -131. */
    stub_set_rssi_inst_raw(263);
    CHECK_EQ(lr_get_rssi_inst(&rssi), 0, name);
    CHECK_EQ(rssi, -132, name);

    PASS(name);
}

/* OOK→LoRa hop-back MUST restore the packet type.  sniffer_ook_arm()
 * leaves the chip in the OOK modem; the pre-fix switch_band skipped
 * SetPacketType(LoRa), so the post-hop "LoRa" leg kept slicing noise
 * through the OOK packet engine (observed live 2026-09-19: constant
 * len=59 hex=ff… LORA frames right after HOP -> LoRa, none before the
 * first OOK hop).  Negative control: compile the DUT with
 * -DLR2021_SIM_OLD_SWITCH_BAND and this test must FAIL. */
static void test_switch_band_restores_lora_pkt_type(void)
{
    const char *name = "switch_band_restores_lora_pkt_type";
    stub_reset();

    /* Boot state = LoRa. */
    CHECK_EQ(stub_get_pkt_type(), LR20XX_PKT_TYPE_LORA, name);

    /* Arm OOK → chip is in the OOK modem now. */
    CHECK_EQ(lr_sniffer_ook_arm(433920000u, 10000u, 0x1C, 240u), 0, name);
    CHECK_EQ(stub_get_pkt_type(), LR20XX_PKT_TYPE_OOK, name);

    /* Hop back to LoRa. */
    CHECK_EQ(lr_sniffer_switch_band_lora(869617984u), 0, name);
    CHECK_EQ(stub_get_pkt_type(), LR20XX_PKT_TYPE_LORA, name);

    /* SetPacketType issued exactly twice: OOK arm + LoRa restore. */
    CHECK_EQ(stub_cmd_count(LR20XX_OP_SET_PKT_TYPE), 2u, name);

    /* And a second full cycle is stable (regression over time). */
    CHECK_EQ(lr_sniffer_ook_arm(433920000u, 10000u, 0x1C, 240u), 0, name);
    CHECK_EQ(stub_get_pkt_type(), LR20XX_PKT_TYPE_OOK, name);
    CHECK_EQ(lr_sniffer_switch_band_lora(869617984u), 0, name);
    CHECK_EQ(stub_get_pkt_type(), LR20XX_PKT_TYPE_LORA, name);
    CHECK_EQ(stub_cmd_count(LR20XX_OP_SET_PKT_TYPE), 4u, name);

    PASS(name);
}

/* ── Sniffer WM-BUS native-modem leg (F1, 2026-09-19) ─────────────── */

static void test_sniffer_wmbus_arm_sequence(void)
{
    const char *name = "sniffer_wmbus_arm_sequence";
    stub_reset();

    int rc = lr_sniffer_wmbus_arm(868950000u, 0x1 /* T1 */, 0x0 /* A */,
                                  255u);
    CHECK_EQ(rc, 0, name);

    /* Sequence: standby → IRQ/FIFO clear → (NO FE cal: the boot band
     * 869.618 MHz → 868.95 MHz is < 20 MHz) → freq → pkt type WM-BUS
     * → SetWmbusParams → RX path → DIO silenced → RX continuous. */
    CHECK_EQ(stub_cmd_count(0x0123), 0u, name);  /* CalibrateFrontEnd */
    CHECK_EQ(stub_cmd_count(0x0200), 1u, name);  /* SetRfFrequency */
    CHECK_EQ(stub_cmd_count(0x0207), 1u, name);  /* SetPacketType */
    CHECK_EQ(stub_cmd_count(0x026A), 1u, name);  /* SetWmbusParams */
    CHECK_EQ(stub_cmd_count(0x0201), 1u, name);  /* SetRxPath */
    CHECK_EQ(stub_cmd_count(0x0115), 1u, name);  /* SetDioIrqCfg */
    CHECK_EQ(stub_cmd_count(0x020C), 1u, name);  /* SetRx */

    CHECK_EQ(stub_get_pkt_type(), LR20XX_PKT_TYPE_WMBUS, name);

    /* A far-band arm must run the single-bin image cal (868.95 → 433.82). */
    stub_reset();
    rc = lr_sniffer_wmbus_arm(433820000u, 0x0C /* F2 */, 0x1, 255u);
    CHECK_EQ(rc, 0, name);
    CHECK_EQ(stub_cmd_count(0x0123), 1u, name);

    /* SetWmbusParams payload (DS Table 12-2): [mode][rx_bw=auto]
     * [pkt_format][addr_comp=off][pld_len][pbl hi][pbl lo]
     * [pbl_len_detect=auto]; T1 preamble = 38 bits (TheClams parity). */
    {
        uint8_t p[8];
        lr_sniffer_wmbus_build_params(0x1, 0x0, 255u, p);
        CHECK_EQ(p[0], 0x01u, name);
        CHECK_EQ(p[1], 0xFFu, name);
        CHECK_EQ(p[2], 0x00u, name);
        CHECK_EQ(p[3], 0x00u, name);
        CHECK_EQ(p[4], 255u, name);
        CHECK_EQ(p[5], 0x00u, name);
        CHECK_EQ(p[6], 38u, name);
        CHECK_EQ(p[7], 0xFFu, name);
    }

    /* Preamble table: S=30, R2=78, T*=38, C*=32, N*=16, F2=78. */
    CHECK_EQ(lr_wmbus_pbl_len_tx(0x0), 30u, name);
    CHECK_EQ(lr_wmbus_pbl_len_tx(0x4), 78u, name);
    CHECK_EQ(lr_wmbus_pbl_len_tx(0x5), 32u, name);
    CHECK_EQ(lr_wmbus_pbl_len_tx(0x8), 16u, name);
    CHECK_EQ(lr_wmbus_pbl_len_tx(0xC), 78u, name);

    PASS(name);
}

static void test_sniffer_wmbus_poll_delivers_fifo(void)
{
    const char *name = "sniffer_wmbus_poll_delivers_fifo";
    stub_reset();

    /* A-format T1 frame: L, C, M('ABC'), A-ID(12345678), version, type,
     * CI, CRC1 — 12 bytes as the FIFO would carry them. */
    static const uint8_t frame[] = {
        0x2F, 0x44,               /* L, C */
        0x43, 0x04,               /* M lo/hi: 'ABC' */
        0x78, 0x56, 0x34, 0x12,   /* A-ID: BCD serial 12345678 */
        0x01, 0x02,               /* version, device type */
        0x7A,                     /* CI */
        0xCB,                     /* CRC1 (not validated here) */
    };

    uint8_t buf[64];
    uint16_t len = 0;
    struct lr_sniffer_wmbus_status st;

    /* Nothing pending → clean zero, status zeroed. */
    CHECK_EQ(lr_sniffer_wmbus_poll(buf, sizeof(buf), &len, &st), 0, name);
    CHECK_EQ(len, 0u, name);

    stub_inject_packet(frame, sizeof(frame));
    stub_set_wmbus_status(0u /* follow FIFO len */, 90u, 88u, 0x00021u,
                          0u /* format A */, 0x0Fu, 0x2F);
    stub_fake_irq_fire_rx_done();

    CHECK_EQ(lr_sniffer_wmbus_poll(buf, sizeof(buf), &len, &st), 0, name);
    CHECK_EQ(len, (uint16_t)sizeof(frame), name);
    CHECK(memcmp(buf, frame, sizeof(frame)) == 0, name,
          "fifo bytes must match the injected frame");
    CHECK_EQ(st.l_field, 0x2Fu, name);
    CHECK_EQ(st.pkt_len, (uint16_t)sizeof(frame), name);
    CHECK_EQ(st.rssi_avg_dbm, -45, name);   /* raw 90 → -45 dBm */
    CHECK_EQ(st.rssi_sync_dbm, -44, name);  /* raw 88 → -44 dBm */
    CHECK_EQ(st.crc_err_mask, 0x00021u, name); /* 17-bit mask: bit0+bit5 */
    CHECK_EQ(st.syncword_idx, 0u, name);
    CHECK_EQ(st.lqi, 0x0Fu, name);

    /* Re-armed after the read (poll-mode housekeeping). */
    CHECK_EQ(stub_cmd_count(0x011E), 1u, name);  /* ClearRxFifo */
    CHECK_EQ(stub_cmd_count(0x020C), 1u, name);  /* SetRx again */

    /* FIFO drained → next poll returns nothing. */
    CHECK_EQ(lr_sniffer_wmbus_poll(buf, sizeof(buf), &len, &st), 0, name);
    CHECK_EQ(len, 0u, name);

    PASS(name);
}

/* WM-BUS poll MUST survive the one-transaction-behind IRQ echo quirk
 * (same live chip behaviour the LoRa/OOK legs hit): the FIRST status
 * read after GetAndClearIrq returns the IRQ word instead of its real
 * response, so a single-read poll parses the echo as a length and
 * silently DROPS the frame.  The proven 3-read dance must still deliver
 * every byte.  Negative control: compile with
 * -DLR2021_SIM_OLD_WMBUS_SINGLE_READ and this test must FAIL. */
static void test_sniffer_wmbus_poll_survives_irq_echo(void)
{
    const char *name = "sniffer_wmbus_poll_survives_irq_echo";
    stub_reset();
    stub_set_force_cs_toggle(true);

    static const uint8_t frame[] = {
        0x1B, 0x44,               /* L, C (S1-style frame, format B) */
        0x3A, 0x63,               /* M lo/hi: 'XYZ' */
        0x40, 0x30, 0x20, 0x10,   /* A-ID: BCD serial 10203040 */
        0x00, 0x07,               /* version, water meter */
        0x8C,                     /* CI */
        0x00,                     /* trailing CRC byte */
    };
    stub_inject_packet(frame, sizeof(frame));
    stub_set_wmbus_status(0u, 90u, 88u, 0u, 1u /* format B */, 0x0Au,
                          0x1Bu);
    stub_fake_irq_fire_rx_done();

    uint8_t buf[64];
    uint16_t len = 0;
    struct lr_sniffer_wmbus_status st;
    CHECK_EQ(lr_sniffer_wmbus_poll(buf, sizeof(buf), &len, &st), 0, name);
    CHECK_EQ(len, (uint16_t)sizeof(frame), name);
    CHECK(memcmp(buf, frame, sizeof(frame)) == 0, name,
          "fifo bytes must match the injected frame");
    /* RSSI/L-field come from the REAL GetWmbusPacketStatus (the echo was
     * consumed by the first GetRxPacketLength). */
    CHECK_EQ(st.rssi_avg_dbm, -45, name);
    CHECK_EQ(st.l_field, 0x1Bu, name);
    CHECK_EQ(st.syncword_idx, 1u, name);   /* format B reported */
    CHECK_EQ(st.lqi, 0x0Au, name);

    /* Drained → next poll returns nothing. */
    len = 0;
    CHECK_EQ(lr_sniffer_wmbus_poll(buf, sizeof(buf), &len, &st), 0, name);
    CHECK_EQ(len, 0u, name);

    stub_set_force_cs_toggle(false);   /* cleanup for later tests */
    PASS(name);
}

/* GetWmbusRxStats must parse the DS Table 12-4 8-byte response (three
 * u16 BE counters; status word in [0..1]). */
static void test_sniffer_wmbus_stats_parity(void)
{
    const char *name = "sniffer_wmbus_stats_parity";
    stub_reset();
    stub_set_wmbus_stats(0x1234, 0x0005, 0x0006);

    uint16_t rx = 0, crc = 0, len = 0;
    CHECK_EQ(lr_sniffer_wmbus_stats(&rx, &crc, &len), 0, name);
    CHECK_EQ(rx, 0x1234u, name);
    CHECK_EQ(crc, 0x0005u, name);
    CHECK_EQ(len, 0x0006u, name);

    /* Boundary: max counters survive (no sign/byte-order surprises). */
    stub_set_wmbus_stats(0xFFFF, 0x8001, 0x0000);
    CHECK_EQ(lr_sniffer_wmbus_stats(&rx, &crc, &len), 0, name);
    CHECK_EQ(rx, 0xFFFFu, name);
    CHECK_EQ(crc, 0x8001u, name);
    CHECK_EQ(len, 0x0000u, name);

    PASS(name);
}

/* wM-Bus block-1 header parse (sniffer_wmbus_parse.c, compiled into the
 * firmware sniffer role AND this sandbox): frame A, frame B, and the
 * too-short negative control. */

static void test_wmbus_parse_frame_a(void)
{
    const char *name = "wmbus_parse_frame_a";
    static const uint8_t frame[] = {
        0x2F, 0x44, 0x43, 0x04, 0x78, 0x56, 0x34, 0x12,
        0x01, 0x02, 0x7A,
    };
    struct snf_wmbus_meta m;

    CHECK_EQ(snf_wmbus_parse(frame, sizeof(frame), &m), 0, name);
    CHECK_EQ(m.l_field, 0x2Fu, name);
    CHECK_EQ(m.c_field, 0x44u, name);
    CHECK_EQ(m.man_code, 0x0443u, name);
    CHECK(memcmp(m.man, "ABC", 3) == 0, name, "man=%s", m.man);
    CHECK(memcmp(m.serial, "12345678", 8) == 0, name, "serial=%s",
          m.serial);
    CHECK_EQ(m.version, 1u, name);
    CHECK_EQ(m.dev_type, 2u, name);
    CHECK_EQ(m.ci, 0x7Au, name);

    PASS(name);
}

static void test_wmbus_parse_frame_b(void)
{
    const char *name = "wmbus_parse_frame_b";
    static const uint8_t frame[] = {
        0x1B, 0x44, 0x3A, 0x63, 0x40, 0x30, 0x20, 0x10,
        0x00, 0x07, 0x8C,
    };
    struct snf_wmbus_meta m;

    CHECK_EQ(snf_wmbus_parse(frame, sizeof(frame), &m), 0, name);
    CHECK_EQ(m.l_field, 0x1Bu, name);
    CHECK(memcmp(m.man, "XYZ", 3) == 0, name, "man=%s", m.man);
    CHECK(memcmp(m.serial, "10203040", 8) == 0, name, "serial=%s",
          m.serial);
    CHECK_EQ(m.dev_type, 0x07u, name);
    CHECK_EQ(m.ci, 0x8Cu, name);

    /* Bad BCD nibble renders '?' instead of wrapping.  Serial bytes are
     * printed most-significant-pair first, so [4]=0xAB lands in the LAST
     * two digits: "000000??". */
    {
        uint8_t bad[11] = { 0x10, 0x44, 0x43, 0x04, 0xAB, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00 };
        CHECK_EQ(snf_wmbus_parse(bad, sizeof(bad), &m), 0, name);
        CHECK(m.serial[0] == '0' && m.serial[1] == '0' &&
              m.serial[6] == '?' && m.serial[7] == '?', name,
              "bad nibbles must render '?', got %s", m.serial);
    }

    PASS(name);
}

static void test_wmbus_parse_negative_short(void)
{
    const char *name = "wmbus_parse_negative_short";
    static const uint8_t shortbuf[10] = { 0x2F, 0x44, 0x43, 0x04, 0x78,
                                          0x56, 0x34, 0x12, 0x01, 0x02 };
    struct snf_wmbus_meta m;

    /* < 11 bytes → rejected (no OOB read). */
    CHECK(snf_wmbus_parse(shortbuf, sizeof(shortbuf), &m) != 0, name,
          "10-byte buffer must be rejected");

    PASS(name);
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    LOG("==== LR2021 driver sandbox ====");
    LOG("Architecture: stub-only, no Zephyr, no flash");
    LOG("");

    test_read_returns_real_data();
    test_get_and_clear_irq_clears_irq();
    test_get_rx_pkt_length_layout_status_byte();
    test_get_lora_packet_status_returns_st_len();
    test_rssi_effective_fallback();
    if (getenv("LR2021_SKIP_RX") == NULL) {
        test_rx_done_handler_flow();
        fprintf(stderr, "main: returned from test_rx_done_handler_flow\n");
        fflush(stderr);
    }
    if (getenv("LR2021_SKIP_ECHO") == NULL) {
        fprintf(stderr, "main: about to call test_stub_echo_when_cs_toggled\n");
        fflush(stderr);
        test_stub_echo_when_cs_toggled();
        fprintf(stderr, "main: returned from test_stub_echo_when_cs_toggled\n");
        fflush(stderr);
    }
    if (getenv("LR2021_SKIP_TX") == NULL) {
        test_tx_done_flow();
    }
    if (getenv("LR2021_SKIP_ORDER") == NULL) {
        test_rx_order_regression();
    }
    if (getenv("LR2021_SKIP_BUNDLE") == NULL) {
        test_bundle_reports_fifo_total();
    }
    if (getenv("LR2021_SKIP_PARITY") == NULL) {
        test_upstream_parity_sequential_rx();
    }
    test_meshcore_flow_burst();
    test_start_rx_edge_recheck();

    /* Dual-band helpers (L1-U1) */
    test_dualband_band_split();
    test_dualband_rx_path();
    test_dualband_hf_pa_duty();
    test_dualband_hf_power_clamp();

    /* TDM band-switch pure helpers (L3-U4) */
    test_tdm_single_cal_bin();
    test_tdm_switch_needs_cal();

    /* TDM dual-band scheduler state machine (L3-U4B) */
    test_tdm_scheduler();

    /* L4-U5: TX band routing decision (dualband_route.h) */
    test_db_route_mask();

    /* L4-U6: HF beacon payload (dualband_beacon.h) */
    test_hf_beacon();

    /* TDM wedge mechanism + fix validation (ROOTCAUSE 2026-08-11) */
    run_tdm_wedge_tests();

    /* Sniffer OOK poll-mode extension (2026-09-19) */
    test_sniffer_ook_arm_sequence();
    test_sniffer_ook_poll_delivers_fifo();
    test_sniffer_ook_poll_survives_irq_echo();

    /* Sniffer driver fix regressions (2026-09-19, 6-asis agentas) */
    test_sniffer_ook_stats_6byte_parity();
    test_rssi_inst_9bit_parity();
    test_switch_band_restores_lora_pkt_type();

    /* Sniffer WM-BUS native-modem leg (F1, ACTION_PLAN_2, 7-asis agentas) */
    test_sniffer_wmbus_arm_sequence();
    test_sniffer_wmbus_poll_delivers_fifo();
    test_sniffer_wmbus_poll_survives_irq_echo();
    test_sniffer_wmbus_stats_parity();
    test_wmbus_parse_frame_a();
    test_wmbus_parse_frame_b();
    test_wmbus_parse_negative_short();

    /* On-chip rtl_433 OOK decode (phase 3, 2026-09-19) — feed the
     * decoder glue synthetic FineOffset WH2 / Acurite-986 frames plus
     * a CRC-flip negative control and noise. */
    run_rtl433_tests(&g_tests_run, &g_failures);

    LOG("");
    LOG("==== %d tests run, %d failures ====", g_tests_run, g_failures);
    return g_failures == 0 ? 0 : 1;
}
