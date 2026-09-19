/*
 * SPDX-License-Identifier: MIT
 *
 * sniffer_wmbus_parse.h — wM-Bus FIFO header metadata parser (sniffer role).
 *
 * The LR2021 native WM-BUS modem decodes the EN 13757-4 PHY itself
 * (3-of-6 / Manchester coding, whitening, per-block CRCs) and delivers
 * the decoded frame through the RX FIFO.  This parser extracts the
 * block-1 metadata from that FIFO image; the field layout and byte
 * orders follow EN 13757-4 and rtl_433 m_bus.c (a558034c, reference
 * only — not vendored):
 *
 *   [0]   L        length field
 *   [1]   C        control field
 *   [2]   M lo     manufacturer, 16-bit little-endian
 *   [3]   M hi     (letters = 5-bit fields 14:10 / 9:5 / 4:0, +0x40)
 *   [4..7] A-ID    BCD serial, least-significant digit pair first
 *   [8]   version
 *   [9]   device type
 *   [10]  CI       control information
 *
 * Format A additionally carries CRC1 at [11] (and one CRC per 16-byte
 * data chunk after that); format B carries the CI at the same offset.
 * CRC *validation* is not done here — the chip reports the per-CRC
 * results via GetWmbusPacketStatus (lr20xx_sniffer_wmbus_status
 * .crc_err_mask), which the caller logs.  This module is host testable:
 * tools/lr2021_sim compiles it and unit-tests the field decoding.
 */
#ifndef SNIFFER_WMBUS_PARSE_H
#define SNIFFER_WMBUS_PARSE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Block-1 header length that must be present before parsing. */
#define SNF_WMBUS_HDR_LEN 11

struct snf_wmbus_meta {
	uint8_t  l_field;   /* L (0 = L-field position in the frame) */
	uint8_t  c_field;   /* C */
	uint16_t man_code;  /* raw M as decoded: (b[3] << 8) | b[2] */
	char     man[4];    /* 3-letter manufacturer code + NUL */
	char     serial[9]; /* 8-digit BCD serial + NUL ('?' on bad nibble) */
	uint8_t  version;   /* A-field version byte */
	uint8_t  dev_type;  /* A-field device type byte */
	uint8_t  ci;        /* CI */
};

/*
 * Parse the wM-Bus block-1 header out of a FIFO frame image.
 *
 * @param buf  FIFO bytes as delivered by the WM-BUS modem
 * @param len  number of valid bytes in buf
 * @param out  parsed metadata (always filled when 0 is returned)
 *
 * @retval 0       parsed
 * @retval -EINVAL buf/out NULL or len < SNF_WMBUS_HDR_LEN
 */
int snf_wmbus_parse(const uint8_t *buf, uint16_t len,
		    struct snf_wmbus_meta *out);

#ifdef __cplusplus
}
#endif

#endif /* SNIFFER_WMBUS_PARSE_H */
