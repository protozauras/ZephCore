/*
 * SPDX-License-Identifier: MIT
 * RepeaterDataStore - Filesystem storage for repeater
 *
 * Uses /lfs/repeater/ prefix to keep data separate from companion.
 * This allows flashing back and forth between roles without corruption.
 */

#pragma once

#include <cstdint>
#include <stddef.h>
#include <mesh/Identity.h>
#include <helpers/NodePrefs.h>
#include <helpers/ClientACL.h>
#include <helpers/RegionMap.h>

/* Daily traffic/login statistics — fixed-size ring persisted to flash
 * (/lfs/repeater/stats_daily.bin).  90 days × 28 B + 40 B header ≈ 2.6 KB.
 * Ring is self-pruning: when full, the oldest day is dropped — the file
 * never grows.  Counters are RAM-only between hourly persists (flash wear
 * and energy cost are negligible: ~2 µAh/day at 1 write/hour). */
#define REPEATER_DAILY_STATS_DAYS  90
#define REPEATER_DAILY_STATS_MAGIC 0x31545344u  /* 'DST1' */

struct DailyStatEntry {
    uint16_t day;       /* epoch day (clock set) or boot-relative day */
    uint16_t reserved;
    uint32_t rx_flood;  /* received flood packets */
    uint32_t rx_direct; /* received direct/pathed packets */
    uint32_t fwd;       /* received packets queued for retransmission */
    uint32_t tx;        /* transmissions (adverts, replies, forwards) */
    uint32_t admin_login;
    uint32_t guest_login;
};

struct DailyStatsFile {
    uint32_t magic;
    uint32_t version;
    uint32_t first_day; /* epoch day of ring[0] */
    uint32_t count;     /* valid entries (0..REPEATER_DAILY_STATS_DAYS) */
    uint32_t total_rx_flood, total_rx_direct, total_fwd, total_tx;
    uint32_t total_admin_login, total_guest_login;
    DailyStatEntry ring[REPEATER_DAILY_STATS_DAYS];
};

class RepeaterDataStore {
public:
    RepeaterDataStore();

    /* Initialize filesystem and repeater directory */
    bool begin();

    /* Identity management */
    bool loadIdentity(mesh::LocalIdentity& id);
    bool saveIdentity(const mesh::LocalIdentity& id);

    /* Prefs management */
    bool loadPrefs(NodePrefs& prefs);
    bool savePrefs(const NodePrefs& prefs);

    /* ACL management - paths passed to ClientACL */
    const char* getAclPath() const;

    /* Region management - paths passed to RegionMap */
    const char* getRegionsPath() const;

    /* Daily stats - fixed-size ring buffer (self-pruning, never grows) */
    bool loadDailyStats(DailyStatsFile& stats);
    bool saveDailyStats(const DailyStatsFile& stats);

    /* Factory reset - erase all repeater data */
    bool formatFileSystem();

    /* Get base path for repeater storage */
    const char* getBasePath() const;

private:
    bool _initialized;
    static constexpr const char* BASE_PATH = "/lfs/repeater";
};
