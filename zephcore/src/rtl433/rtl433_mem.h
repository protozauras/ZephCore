/** @file
    Static-arena memory shim + build compat for vendored rtl_433 sources.

    SNF_RTL433 (ZephCore RF sniffer) — this header is force-included first
    into every vendored rtl_433 translation unit (see zephcore/CMakeLists.txt
    role-specific sources, tools/lr2021_sim/Makefile and build_msvc2.bat).
    It guarantees ZERO dependence on the libc heap inside the firmware:

      - malloc/calloc/realloc/free are rebound to a static ~32 KiB bump
        arena.  realloc allocates+copy without freeing the old block (bump
        semantics); free is a no-op; snf_mem_reset() rewinds the whole
        arena and is called by the glue once per finalized OOK frame.
      - strdup is provided on top of the arena (Zephyr minimal libc has
        no strdup).
      - RTL_433_REDUCE_STACK_USE shrinks bitbuffer_t from ~6.4 KiB to
        ~1 KiB so the per-frame slicer stack stays within Zephyr's main
        thread budget.
      - SNF_PRINTF routes diagnostics: printk() on the firmware, a
        test-capture hook (snf_host_logf) in the sandbox.

    The arena itself lives in sniffer_rtl433.c (the one TU that defines
    snf_mem_arena / snf_mem_used); every other TU references it via the
    extern declarations below.
*/

#ifndef SNF_RTL433_MEM_H_
#define SNF_RTL433_MEM_H_

/* Vendored-code patch selector (all upstream edits are wrapped in this). */
#define SNF_RTL433 1

/* Upstream knob: 40x25-byte bitbuffer instead of 128x50 (6.4K -> 1K). */
#define RTL_433_REDUCE_STACK_USE 1

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Static-arena bump allocator ─────────────────────────────────────── */

#define SNF_MEM_ARENA_BYTES (32u * 1024u)

/* Instantiated in sniffer_rtl433.c (the one TU that owns the arena). */
extern uint8_t snf_mem_arena[SNF_MEM_ARENA_BYTES];
extern size_t snf_mem_used;

/* Reset the arena to empty.  Called by the glue once per finalized frame,
 * AFTER the decoded data has been printed. */
void snf_mem_reset(void);

/* High-water mark of arena usage in bytes (diagnostics). */
size_t snf_mem_highwater(void);

static inline void *snf_malloc(size_t size)
{
    uint8_t *p;
    size_t aligned;

    aligned = (size + 7u) & ~(size_t)7u;
    if (aligned < size || snf_mem_used + aligned > SNF_MEM_ARENA_BYTES) {
        return NULL; /* arena exhausted */
    }
    p = &snf_mem_arena[snf_mem_used];
    snf_mem_used += aligned;
    return p;
}

static inline void *snf_calloc(size_t n, size_t size)
{
    size_t total = n * size;
    void *p;

    if (size != 0 && total / size != n) {
        return NULL; /* overflow */
    }
    p = snf_malloc(total);
    if (p) {
        memset(p, 0, total);
    }
    return p;
}

/* Bump semantics: allocate a new block and copy; the old block is NOT
 * freed (impossible with a bump allocator) — it is reclaimed wholesale
 * by snf_mem_reset(). */
static inline void *snf_realloc(void *old, size_t size)
{
    void *p = snf_malloc(size);

    if (p && old) {
        /* Copy at most the usable arena space; callers track true size,
         * this is the standard bump-allocator compromise. */
        memcpy(p, old, size);
    }
    return p;
}

static inline void snf_free(void *ptr)
{
    (void)ptr; /* no-op: reclaim happens at snf_mem_reset() */
}

static inline char *snf_strdup(char const *s)
{
    size_t len = strlen(s) + 1u;
    char *p   = (char *)snf_malloc(len);

    if (p) {
        memcpy(p, s, len);
    }
    return p;
}

/* Rebind the heap symbols for the vendored TUs (defined AFTER <stdlib.h>
 * so the library prototypes themselves are not macro-expanded).
 * Object-like (not function-like) macros on purpose: upstream data.c
 * takes `free` as a bare function pointer ((value_release_fn)free),
 * which a function-like macro would NOT expand — leaving a call to the
 * CRT free() on arena pointers (heap corruption). */
#define malloc snf_malloc
#define calloc snf_calloc
#define realloc snf_realloc
#define free snf_free
#define strdup snf_strdup

/* ── Diagnostic output routing ───────────────────────────────────────── */

#ifdef SNF_RTL433_HOST
/* Sandbox build: single variadic sink owned by sniffer_rtl433.c.
 * The default sink appends to a capture buffer the tests read back;
 * snf_set_print_hook() can redirect it. */
void snf_host_logf(char const *fmt, ...);
#define SNF_PRINTF(fmt, ...) snf_host_logf(fmt, ##__VA_ARGS__)
#else
/* Firmware: printk straight to the USB CDC console. */
#include <zephyr/sys/printk.h>
#define SNF_PRINTF(fmt, ...) printk(fmt, ##__VA_ARGS__)
#endif

#ifdef __cplusplus
}
#endif

#endif /* SNF_RTL433_MEM_H_ */
