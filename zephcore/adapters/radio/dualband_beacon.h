/*
 * SPDX-License-Identifier: MIT
 * Pure dual-band HF beacon (advert) encode/decode — no Zephyr / no hardware
 * dependencies.
 *
 * Shared VERBATIM between the mesh layer (production, RepeaterMesh::onAdvertRecv
 * / createHfBeacon) and the LR2021 sandbox (tools/lr2021_sim, which #includes
 * this header directly) so the tested logic can never drift from the
 * production logic (pitfall #21).
 *
 * Model (LR2021_DUALBAND_RESEARCH.md §L4-U6, concept §3.1): the high-altitude
 * repeater periodically advertises its SECONDARY band parameters on the primary
 * (sub-GHz) channel.  A dual-band home node that hears this beacon learns that
 * the sender is reachable on HF and registers it as an HF neighbour (plan §L4-U6
 * priemimo puse — even though the beacon itself arrived on sub-GHz).
 *
 * Backward compatibility: the beacon uses its OWN advert type
 * (ADV_TYPE_HF_BEACON), NOT ADV_TYPE_REPEATER, so stock single-band nodes never
 * mistake it for a repeater contact.  Legacy parsers read the low nibble as the
 * type, see an unknown type, find no ADV_NAME_MASK, and ignore it entirely —
 * they do not add a contact ({}-{}-{}-{}-{}-{}-{}).  Because a beacon carries no
 * ADV_NAME_MASK, a stock BaseChatMesh::onAdvertRecv rejects it as "no name" and
 * the packet is only flooded through, never auto-added.
 *
 * Beacon payload (fixed layout, version byte b0 for future extension):
 *   [0]      version          = 1
 *   [1..4]   freq_hz          uint32 LE (e.g. 2450000000 = 2450.0 MHz)
 *   [5..6]   bw_khz           uint16 LE (500)
 *   [7]      sf               uint8  (8)
 *   [8]      cr               uint8  (Zephyr lora_coding_rate / chip code, 1 = CR_4/5)
 *   [9]      sync             uint8  (0x12 private)
 *   [10]     tx_pwr_dbm       int8   (12)
 *   [11..12] load             uint16 LE (0 = unknown/not published)
 * Total fixed size: 13 bytes (advert app_data limit is MAX_ADVERT_DATA_SIZE 32).
 */

#ifndef ZEPHCORE_DUALBAND_BEACON_H
#define ZEPHCORE_DUALBAND_BEACON_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Advert type for the HF beacon — must not collide with existing ADV_TYPE_*
 * values (AdvertDataHelpers.h 0..4).  Kept here (not in AdvertDataHelpers.h) so
 * the pure header stays dependency-free for the sandbox. */
#define DB_BEACON_ADV_TYPE       5u

#define DB_BEACON_VERSION        1u
#define DB_BEACON_PAYLOAD_SIZE   13u   /* version + freq4 + bw2 + sf1 + cr1 + sync1 + pwr1 + load2 */

/* Every ~60 s per the concept (concept §3.1: "periodically"; plan §L4-U6 point 1). */
#define DB_BEACON_PERIOD_MS      60000u

/* Global (compile-time) secondary-band preset published in the beacon
 * (plan §1.4 / L3-U4B: 2450.0 MHz / BW500 / SF8 / CR4/5 / sync 0x12 / +12 dBm).
 * Load is a runtime quantity (0 = unknown this version). */
#define DB_BEACON_FREQ_HZ        2450000000UL
#define DB_BEACON_BW_KHZ         500u
#define DB_BEACON_SF             8u
#define DB_BEACON_CR             1u
#define DB_BEACON_SYNC           0x12u
#define DB_BEACON_TX_PWR_DBM     12
#define DB_BEACON_LOAD           0u

typedef struct {
	uint32_t freq_hz;
	uint16_t bw_khz;
	uint8_t  sf;
	uint8_t  cr;
	uint8_t  sync;
	int8_t   tx_pwr_dbm;
	uint16_t load;
} db_beacon_params_t;

/* Fill params with the published secondary-band preset. */
static inline void db_beacon_defaults(db_beacon_params_t *p)
{
	p->freq_hz    = DB_BEACON_FREQ_HZ;
	p->bw_khz     = DB_BEACON_BW_KHZ;
	p->sf         = DB_BEACON_SF;
	p->cr         = DB_BEACON_CR;
	p->sync       = DB_BEACON_SYNC;
	p->tx_pwr_dbm = DB_BEACON_TX_PWR_DBM;
	p->load       = DB_BEACON_LOAD;
}

/*
 * Encode beacon payload (excluding the advert type byte).  Returns the number
 * of bytes written (DB_BEACON_PAYLOAD_SIZE) or 0 when cap is too small.
 */
static inline uint8_t db_beacon_encode(uint8_t *out, size_t cap,
				       const db_beacon_params_t *p)
{
	uint8_t *o = out;
	if (cap < DB_BEACON_PAYLOAD_SIZE) {
		return 0;
	}
	*o++ = DB_BEACON_VERSION;
	memcpy(o, &p->freq_hz, 4); o += 4;
	memcpy(o, &p->bw_khz, 2); o += 2;
	*o++ = p->sf;
	*o++ = p->cr;
	*o++ = p->sync;
	*o++ = (uint8_t)p->tx_pwr_dbm;
	memcpy(o, &p->load, 2); o += 2;
	return (uint8_t)(o - out);
}

/*
 * Decode beacon payload.  Returns true and fills *p on success.  A payload of
 * length < DB_BEACON_PAYLOAD_SIZE or with an unsupported version is rejected.
 * freq_hz is only meaningful (and only accepted) when it is a HF value —
 * sub-GHz garbage (e.g. the tail of an unrelated advert) is rejected.
 */
static inline bool db_beacon_decode(const uint8_t *in, size_t len,
				    db_beacon_params_t *p)
{
	const uint8_t *s = in;
	if (len < DB_BEACON_PAYLOAD_SIZE) {
		return false;
	}
	if (*s != DB_BEACON_VERSION) {
		return false;
	}
	s++;
	memcpy(&p->freq_hz, s, 4); s += 4;
	memcpy(&p->bw_khz, s, 2); s += 2;
	p->sf   = *s++;
	p->cr   = *s++;
	p->sync = *s++;
	p->tx_pwr_dbm = (int8_t)*s++;
	memcpy(&p->load, s, 2); s += 2;
	return true;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_DUALBAND_BEACON_H */
