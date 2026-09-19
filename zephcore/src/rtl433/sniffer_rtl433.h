/** @file
    sniffer_rtl433.h — on-chip rtl_433 OOK decode glue for the ZephCore
    RF sniffer (role ZEPHCORE_ROLE_SNIFFER, RX-only by design).

    The LR2021 has no OOK direct mode; the sniffer captures the 433.92 MHz
    OOK envelope as a packet-mode bit stream at 10 kbps (100 us per bit)
    through the RX FIFO (lr20xx_sniffer_ook_poll, chunks up to 240 bits).
    This module accumulates those bits, segments transmissions on long
    zero gaps, converts the run-lengths into an rtl_433 pulse_data_t
    (sample_rate = 1e6 => all timing constants stay in microseconds),
    runs the curated registry of pure-C decoders through the upstream
    pulse_slicer_* demods, and prints one "OOK JSON {...}" line per
    decoded sensor reading over the console.

    Vendored upstream (GPL-2.0-or-later, see src/rtl433/VENDOR.md):
    rtl_433 master @ a558034cae051651f6969b77518f57a88f6a232e.
*/

#ifndef SNF_RTL433_GLUE_H_
#define SNF_RTL433_GLUE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One-time init: copies the curated decoder templates into mutable
 * instances and wires output/log callbacks.  Call once at boot. */
void snf_rtl433_init(void);

/* Feed one OOK FIFO poll delivery (<= 240 bits / 30 bytes; larger
 * buffers are accepted but clamped to the FIFO size).  Bits are appended
 * MSB-first per byte (bit7 = earliest) unless the Kconfig option
 * CONFIG_ZEPHCORE_SNIFFER_OOK_LSB_FIRST is enabled (bit order is an
 * unverified open item until a real 433 MHz capture).  A >= 30 ms zero
 * tail or a > 3000-bit accumulator triggers frame finalization, decode
 * and JSON output. */
void snf_rtl433_feed(const uint8_t *chunk, uint16_t chunk_len);

/* Drop the pending accumulator without decoding (band hop, overflow). */
void snf_rtl433_reset(void);

#ifdef SNF_RTL433_HOST
/* ── Sandbox test hooks (tools/lr2021_sim) ──────────────────────────── */

/* Redirect all SNF_PRINTF output; pass NULL to restore the built-in
 * capture buffer. */
void snf_set_print_hook(void (*hook)(char const *line));

/* Clear the built-in capture buffer and the OOK accumulator. */
void snf_test_capture_reset(void);

/* Number of captured lines. */
int snf_test_line_count(void);

/* The n-th captured line (0-based), or NULL if out of range. */
char const *snf_test_line(int idx);

/* 1 if any captured line contains `needle`. */
int snf_test_contains(char const *needle);

/* Number of captured lines starting with "OOK JSON ". */
int snf_test_json_count(void);

/* 1 if the JSON output contains a line with model `model` and all the
 * given key:value substrings (needles may be NULL-terminated subset). */
int snf_test_json_match(char const *model, int n_needles,
                        char const *const *needles);

/* Number of finalized frames since the last snf_test_capture_reset(). */
unsigned snf_test_finalize_count(void);

/* Diagnostics: dump glue state (accumulator / arena / decoder counters)
 * into the capture buffer. */
void snf_test_dbg_dump(void);
#endif /* SNF_RTL433_HOST */

#ifdef __cplusplus
}
#endif

#endif /* SNF_RTL433_GLUE_H_ */
