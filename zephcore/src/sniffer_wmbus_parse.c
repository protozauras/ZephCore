/*
 * SPDX-License-Identifier: MIT
 *
 * sniffer_wmbus_parse.c — see sniffer_wmbus_parse.h for the wire layout.
 * Host testable (tools/lr2021_sim, test_lr2021_driver.c) and compiled
 * into the sniffer role (zephcore/CMakeLists.txt).
 */
#include "sniffer_wmbus_parse.h"

#include <errno.h>

/* BCD nibble → digit; anything above 9 becomes '?' (rtl_433 renders
 * garbage nibbles via its bcd2int multiply — for a sniffer log '?' is
 * more honest than silently wrapping). */
static char bcd_digit(uint8_t nibble)
{
	return (nibble <= 9u) ? (char)('0' + nibble) : '?';
}

int snf_wmbus_parse(const uint8_t *buf, uint16_t len,
		    struct snf_wmbus_meta *out)
{
	if ((buf == 0) || (out == 0) || (len < SNF_WMBUS_HDR_LEN)) {
		return -EINVAL;
	}

	out->l_field = buf[0];
	out->c_field = buf[1];

	/* Manufacturer: wire bytes [2]=lo, [3]=hi; 3 letters of 5 bits
	 * each from bits 14:10 / 9:5 / 4:0 (+0x40) — rtl_433
	 * m_bus_manuf_decode parity. */
	out->man_code = ((uint16_t)buf[3] << 8) | (uint16_t)buf[2];
	out->man[0] = (char)(((out->man_code >> 10) & 0x1Fu) + 0x40u);
	out->man[1] = (char)(((out->man_code >> 5) & 0x1Fu) + 0x40u);
	out->man[2] = (char)((out->man_code & 0x1Fu) + 0x40u);
	out->man[3] = '\0';

	/* Serial: BCD, least-significant digit pair first ([4] = LSB),
	 * printed most significant pair first → "12345678" for
	 * [4..7] = 0x78 0x56 0x34 0x12 (rtl_433 A_ID parity). */
	out->serial[0] = bcd_digit((uint8_t)(buf[7] >> 4));
	out->serial[1] = bcd_digit((uint8_t)(buf[7] & 0x0Fu));
	out->serial[2] = bcd_digit((uint8_t)(buf[6] >> 4));
	out->serial[3] = bcd_digit((uint8_t)(buf[6] & 0x0Fu));
	out->serial[4] = bcd_digit((uint8_t)(buf[5] >> 4));
	out->serial[5] = bcd_digit((uint8_t)(buf[5] & 0x0Fu));
	out->serial[6] = bcd_digit((uint8_t)(buf[4] >> 4));
	out->serial[7] = bcd_digit((uint8_t)(buf[4] & 0x0Fu));
	out->serial[8] = '\0';

	out->version  = buf[8];
	out->dev_type = buf[9];
	out->ci       = buf[10];

	return 0;
}
