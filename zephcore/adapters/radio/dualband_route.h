/*
 * SPDX-License-Identifier: MIT
 * Pure dual-band TX routing rules — no Zephyr / no hardware dependencies.
 *
 * Shared VERBATIM between the mesh layer (production, see Dispatcher::getTxBandMask
 * and RepeaterMesh's override) and the LR2021 sandbox (tools/lr2021_sim, which
 * #includes this header directly) so the tested logic can never drift from the
 * production logic (pitfall #21).
 *
 * Model (LR2021_DUALBAND_RESEARCH.md §1.3 / §L4-U5): a dual-band node keeps
 * backward compatibility with the live 868 MHz mesh while also reaching
 * 2.4 GHz nodes.  The Tx band for a packet is decided by its route flavour:
 *
 *   - flood / discovery  -> BOTH bands (sub-GHz now, HF copy at the next
 *                           TDM window)
 *   - direct with a KNOWN next-hop band -> that band ONLY (never re-broadcast
 *                           a sub-GHz-received packet on sub-GHz when its next
 *                           hop lives on HF)
 *   - direct with UNKNOWN next-hop band -> BOTH (fallback)
 *
 * No on-air bytes are changed — the decision is purely in-RAM and invisible
 * to single-band nodes (plan §1.3).
 */

#ifndef ZEPHCORE_DUALBAND_ROUTE_H
#define ZEPHCORE_DUALBAND_ROUTE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Band bit-mask values used as the TX routing mask (plan §1.2). */
#define DB_BAND_SUBGHZ (1u << 0)      /* primary / sub-GHz (869.618 MHz) */
#define DB_BAND_HF     (1u << 1)      /* secondary / 2.4 GHz (2450 MHz) */
#define DB_BAND_BOTH   (DB_BAND_SUBGHZ | DB_BAND_HF)
#define DB_BAND_NONE   0u

/*
 * Decide the destination band mask for a packet about to be transmitted.
 *
 * @param is_flood      true when the packet is a flood/discovery (route type
 *                      FLOOD or TRANSPORT_FLOOD)
 * @param next_hop_band the peer band for DIRECT packets: DB_BAND_SUBGHZ,
 *                      DB_BAND_HF, or DB_BAND_NONE when the next hop is
 *                      unknown / not in the neighbour table.  Ignored for
 *                      floods.
 *
 * @return a DB_BAND_* mask (never DB_BAND_NONE).
 */
static inline uint8_t db_tx_band_mask(bool is_flood, uint8_t next_hop_band)
{
	/* Flood/discovery routing reaches both bands (plan §1.3: flood -> both). */
	if (is_flood) {
		return DB_BAND_BOTH;
	}
	/* Direct: known band -> that band only; unknown -> both. */
	if (next_hop_band == DB_BAND_HF) {
		return DB_BAND_HF;
	}
	if (next_hop_band == DB_BAND_SUBGHZ) {
		return DB_BAND_SUBGHZ;
	}
	return DB_BAND_BOTH;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_DUALBAND_ROUTE_H */
