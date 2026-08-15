/* DiagRing — in-firmware ring of recent WRN/ERR lines, shipped home over
 * the HF backbone (2026-08-16, WORKPLAN §8.3).  The repeater captures
 * interesting error lines here; the housekeeping tick drains them into
 * HF-only packets that the companion relays to MQTT as raw RX lines.
 *
 * C-callable on purpose: both the C++ adapters and the C driver can add
 * lines.  Not thread-safe in the strict sense — concurrent adds are
 * serialized by the SPI mutex / main-loop context in practice, and a
 * lost slot is acceptable for diagnostics. */
#ifndef MESH_DIAG_RING_H
#define MESH_DIAG_RING_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One captured line: uptime_ms stamp + text (truncated). */
#define DIAG_RING_LINE_MAX 48

/* Append a WRN/ERR line to the ring (older entries wrap out). */
void diag_ring_add(uint32_t uptime_ms, const char *text);

/* True when unsent entries exist. */
bool diag_ring_pending(void);

/* Build one HF diag packet from the oldest unsent entry:
 *   [0]      = 0xD1 magic (mesh parser rejects it: version bits = 3)
 *   [1]      = monotonic sequence
 *   [2..5]   = uptime_ms (little-endian) at capture
 *   [6..6+n] = text (n <= DIAG_RING_LINE_MAX)
 * Returns the packet length, or 0 when the ring is empty.
 * On success the entry is consumed. */
int diag_ring_build(uint8_t *out, int cap);

#ifdef __cplusplus
}
#endif

#endif /* MESH_DIAG_RING_H */
