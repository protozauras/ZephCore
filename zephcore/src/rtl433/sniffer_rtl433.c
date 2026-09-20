/** @file
    sniffer_rtl433.c — bitstream → pulses → demod → JSON glue for the
    ZephCore RF sniffer (RX-only, ZEPHCORE_ROLE_SNIFFER).

    Own code (MIT, ZephCore).  Links against the vendored upstream
    rtl_433 core (GPL-2.0-or-later — see VENDOR.md) exactly as the
    pinned master wires it: pulse_slicer_* demods (run_ook_demods
    dispatch mirror) → device->decode_fn → decoder_output_data →
    output_fn → one JSON line per data item via printk.

    Memory model: every allocation made by the vendored code lands in
    the static 32 KiB arena of rtl433_mem.h; snf_mem_reset() rewinds it
    after each finalized frame (print first, reset after — the JSON is
    emitted while the decoded data is still alive).
*/

/* This TU owns the arena: rtl433_mem.h declares it extern, we define it. */
#include "rtl433_mem.h"
#include "sniffer_rtl433.h"

#include "decoder.h"      /* r_device, data_t, bitbuffer, decoder_util */
#include "pulse_data.h"   /* pulse_data_t */
#include "pulse_slicer.h" /* pulse_slicer_* (upstream demods) */
#include "logger.h"       /* log_level_t, print_log/print_logf decls */

#include <stdarg.h>
#include <stdio.h>
#include <limits.h>

/* ── rtl433_mem.h arena instantiation ────────────────────────────────── */

uint8_t snf_mem_arena[SNF_MEM_ARENA_BYTES];
size_t snf_mem_used;

static size_t snf_mem_high = 0;

void snf_mem_reset(void)
{
    if (snf_mem_used > snf_mem_high) {
        snf_mem_high = snf_mem_used;
    }
    snf_mem_used = 0;
}

size_t snf_mem_highwater(void)
{
    return snf_mem_high;
}

/* ── Diagnostic output (printk on firmware, capture in sandbox) ──────── */

#ifdef SNF_RTL433_HOST
#define SNF_CAPTURE_LINES 64
#define SNF_CAPTURE_LINELEN 512
static char snf_cap[SNF_CAPTURE_LINES][SNF_CAPTURE_LINELEN];
static int snf_cap_n;
static void (*snf_print_hook)(char const *line);
/* Diagnostics: finalized-frame counter (reset per capture reset). */
static unsigned snf_finalize_count;

void snf_host_logf(char const *fmt, ...)
{
    char buf[SNF_CAPTURE_LINELEN - 8];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (snf_print_hook) {
        snf_print_hook(buf);
        return;
    }
    if (snf_cap_n < SNF_CAPTURE_LINES) {
        snprintf(snf_cap[snf_cap_n], SNF_CAPTURE_LINELEN, "%s", buf);
        snf_cap_n++;
    }
}

void snf_set_print_hook(void (*hook)(char const *line))
{
    snf_print_hook = hook;
}

void snf_test_capture_reset(void)
{
    snf_cap_n = 0;
    snf_finalize_count = 0;
    snf_rtl433_reset();
}

int snf_test_line_count(void)
{
    return snf_cap_n;
}

char const *snf_test_line(int idx)
{
    if (idx < 0 || idx >= snf_cap_n) {
        return NULL;
    }
    return snf_cap[idx];
}

int snf_test_contains(char const *needle)
{
    for (int i = 0; i < snf_cap_n; i++) {
        if (strstr(snf_cap[i], needle) != NULL) {
            return 1;
        }
    }
    return 0;
}

int snf_test_json_count(void)
{
    int n = 0;

    for (int i = 0; i < snf_cap_n; i++) {
        if (strncmp(snf_cap[i], "OOK JSON ", 9) == 0) {
            n++;
        }
    }
    return n;
}

/* Number of finalized frames since the last capture reset (diagnostics). */
unsigned snf_test_finalize_count(void)
{
    return snf_finalize_count;
}

int snf_test_json_match(char const *model, int n_needles,
                        char const *const *needles)
{
    for (int i = 0; i < snf_cap_n; i++) {
        if (strncmp(snf_cap[i], "OOK JSON ", 9) != 0) {
            continue;
        }
        if (model != NULL && strstr(snf_cap[i], model) == NULL) {
            continue;
        }
        int all = 1;
        for (int k = 0; k < n_needles; k++) {
            if (strstr(snf_cap[i], needles[k]) == NULL) {
                all = 0;
                break;
            }
        }
        if (all) {
            return 1;
        }
    }
    return 0;
}
#endif /* SNF_RTL433_HOST */

/* ── logger.h implementation (logger.c is not vendored) ──────────────── */

/* Field-friendly level: warnings/errors only, INFO/DEBUG/TRACE silenced. */
#define SNF_RTL433_LOG_LEVEL LOG_WARNING

void print_log(log_level_t level, char const *src, char const *msg)
{
    if (level > SNF_RTL433_LOG_LEVEL) {
        return;
    }
    SNF_PRINTF("[rtl433 %s] %s\n", src, msg);
}

void print_logf(log_level_t level, char const *src, char const *fmt, ...)
{
    char buf[224];
    va_list ap;

    if (level > SNF_RTL433_LOG_LEVEL) {
        return;
    }
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    SNF_PRINTF("[rtl433 %s] %s\n", src, buf);
}

/* ── Curated decoder registry ────────────────────────────────────────── */

/* Upstream decoder templates (r_device const in each vendored file). */
extern r_device const acurite_rain_896;
extern r_device const acurite_th;
extern r_device const acurite_txr;
extern r_device const acurite_986;
extern r_device const acurite_985;
extern r_device const acurite_606;
extern r_device const acurite_00275rm;
extern r_device const acurite_590tx;
extern r_device const lacrosse_tx141x; /* TX141 / TX141TH-Bv2 / Bv3 */
extern r_device const oregon_scientific; /* v2_1 and v3 sensors */
extern r_device const oregon_scientific_v1;
extern r_device const fineoffset_WH2; /* WH2 / WH2A / WH5 / Telldus */
extern r_device const fineoffset_WH0530;
extern r_device const prologue;
extern r_device const hideki_ts04;
/* TPMS (OOK/ASK only — the FSK OEM sensors cannot be demodulated from
 * an OOK envelope; see VENDOR.md registry notes). */
extern r_device const tpms_schrader_motorcycle;
extern r_device const tpms_gm;
extern r_device const tpms_smartire;

#define SNF_NUM_DEVICES 18

static r_device snf_devs[SNF_NUM_DEVICES];
static unsigned snf_n_devs;

/* OOK bit accumulator (defined size-wise below; declared here because
 * snf_rtl433_init() resets it). */
static uint32_t snf_acc_bits;

/* Frequency the capture is listening on (echoed into every OOK JSON as
 * "freq" so the host ETL can tag the band; default = classic 433.92). */
static uint32_t snf_ook_freq_hz = 433920000u;

/* Defined below (JSON writer section). */
static void snf_data_print_json(data_t *data, char const *device_name);

/* Drop device-originated log messages by default — mirror of the pinned
 * master where no log output is registered (log_fn would print nothing
 * at the default verbosity). */
static void snf_log_handler(r_device *decoder, int level, data_t *data)
{
    (void)decoder;
    (void)level;
    (void)data;
}

/* One data item = one JSON line.  data_free() afterwards mirrors the
 * pinned master (data_acquired_handler frees; with the bump arena it is
 * a no-op, the arena reset happens per frame in snf_finalize). */
static void snf_output_handler(r_device *decoder, data_t *data)
{
    if (data == NULL) {
        SNF_PRINTF("OOK JSON {\"error\":\"decoder produced no data\"}\n");
        return;
    }
    snf_data_print_json(data, decoder->name);
    data_free(data);
}

void snf_rtl433_init(void)
{
    r_device const *templates[] = {
        &acurite_rain_896,
        &acurite_th,
        &acurite_txr,
        &acurite_986,
        &acurite_985,
        &acurite_606,
        &acurite_00275rm,
        &acurite_590tx,
        &lacrosse_tx141x,
        &oregon_scientific,
        &oregon_scientific_v1,
        &fineoffset_WH2,
        &fineoffset_WH0530,
        &prologue,
        &hideki_ts04,
        &tpms_schrader_motorcycle,
        &tpms_gm,
        &tpms_smartire,
    };

    snf_n_devs = (unsigned)(sizeof(templates) / sizeof(templates[0]));
    for (unsigned i = 0; i < snf_n_devs; i++) {
        snf_devs[i]          = *templates[i];
        snf_devs[i].protocol_num = i + 1;
        snf_devs[i].verbose  = 0;
        snf_devs[i].verbose_bits = 0;
        snf_devs[i].log_fn   = snf_log_handler;
        snf_devs[i].output_fn = snf_output_handler;
        snf_devs[i].output_ctx = NULL;
        snf_devs[i].decode_ctx  = NULL;
    }

    /* NOTE: the Manchester TPMS decoders keep the upstream tolerance=0
     * DELIBERATELY.  In pulse_slicer_manchester_zerobit a nonzero
     * tolerance ENABLES the validity-window branch, and that branch
     * row-breaks on the end-of-message gap BEFORE the EOM check runs
     * (num_rows becomes 2 -> the decoders' num_rows!=1 gate aborts with
     * DECODE_ABORT_EARLY).  With tolerance=0 there is no validity
     * branch: the quantized widths (122 µs half-bit -> 100/150 µs;
     * merged 244 µs -> 200/250 µs at the 50 µs capture grid) are
     * separated purely by the 1.5× half-bit data-edge threshold
     * (183 µs), and the live-observed values all land on the correct
     * sides.  The MC TPMS tests decode real frames through this exact
     * path (they are the regression gate for the 20 kbps capture
     * clock). */

    snf_acc_bits = 0;
}

/* ── Bit accumulator ─────────────────────────────────────────────────── */

#ifndef CONFIG_ZEPHCORE_SNIFFER_DECODE_BUF_BITS
#define CONFIG_ZEPHCORE_SNIFFER_DECODE_BUF_BITS 8000
#endif
#ifndef SNF_OOK_BIT_LSB_FIRST
#if defined(CONFIG_ZEPHCORE_SNIFFER_OOK_LSB_FIRST) && CONFIG_ZEPHCORE_SNIFFER_OOK_LSB_FIRST
#define SNF_OOK_BIT_LSB_FIRST 1
#else
#define SNF_OOK_BIT_LSB_FIRST 0
#endif
#endif

/* OOK demod bit-clock period (µs per captured bit).  The firmware build
 * derives it from the Kconfig bit rate — the single source of truth
 * shared with SetOokModulationParams; host/sandbox builds default to
 * the 20 kbps / 50 µs grid the tests are authored against (the old
 * 10 kbps grid quantized 120-170 µs TPMS/Manchester half-bits into
 * ambiguous 1-2 bit runs). */
#if defined(CONFIG_ZEPHCORE_SNIFFER_OOK_BR_BPS) && CONFIG_ZEPHCORE_SNIFFER_OOK_BR_BPS
#define SNF_OOK_BIT_US (1000000u / (uint32_t)CONFIG_ZEPHCORE_SNIFFER_OOK_BR_BPS)
#elif !defined(SNF_OOK_BIT_US)
/* Host/sandbox default = the 20 kbps grid the tests are authored
 * against.  -DSNF_OOK_BIT_US=100 forces the OLD 10 kbps grid — the
 * negative-control build where the MC TPMS tests must FAIL (their
 * frames stop decoding when half-bits quantize to ambiguous 1-2 bit
 * runs). */
#define SNF_OOK_BIT_US 50u
#endif

#define SNF_ACC_BYTES ((CONFIG_ZEPHCORE_SNIFFER_DECODE_BUF_BITS + 7u) / 8u)
#define SNF_GAP_END_BITS ((30u * 1000u) / SNF_OOK_BIT_US) /* 30 ms zero tail = end of transmission */
#define SNF_FORCE_FINALIZE_BITS ((300u * 1000u) / SNF_OOK_BIT_US) /* 300 ms safety cap */
#define SNF_MAX_CHUNK_BITS (255u * 8u) /* LR2021 PLD_LEN = 255 bytes per FIFO packet */

static uint8_t snf_acc[SNF_ACC_BYTES];

static unsigned snf_get_bit(uint32_t idx)
{
    uint8_t byte = snf_acc[idx >> 3];
    uint8_t bit;

#if SNF_OOK_BIT_LSB_FIRST
    bit = (byte >> (idx & 7u)) & 1u;
#else
    bit = (byte >> (7u - (idx & 7u))) & 1u;
#endif
    return bit;
}

void snf_rtl433_reset(void)
{
    snf_acc_bits = 0;
}

void snf_rtl433_set_ook_freq(uint32_t freq_hz)
{
    snf_ook_freq_hz = freq_hz;
}

/* Runs: count trailing zero bits of the accumulator. */
static unsigned snf_tail_zeros(void)
{
    uint32_t n = 0;

    while (n < snf_acc_bits && snf_get_bit(snf_acc_bits - 1u - n) == 0u) {
        n++;
    }
    return n;
}

#ifdef SNF_RTL433_HOST
/* Diagnostics: dump glue state into the capture buffer (accumulator,
 * arena, per-decoder event/fail counters).  Used by the tests to
 * explain a frame that unexpectedly decoded nothing. */
void snf_test_dbg_dump(void)
{
    char tail[40];
    unsigned t = snf_tail_zeros();

    for (unsigned i = 0; i < 24 && i < snf_acc_bits; i++) {
        tail[i] = snf_get_bit(snf_acc_bits - 1u - i) ? '1' : '0';
    }
    tail[t < 24 ? t : 24] = '\0';
    SNF_PRINTF("DBG acc_bits=%u tail_zeros=%u last24=%s mem_used=%u n_devs=%u\n",
               (unsigned)snf_acc_bits, t, tail, (unsigned)snf_mem_used,
               (unsigned)snf_n_devs);
    for (unsigned i = 0; i < snf_n_devs; i++) {
        SNF_PRINTF("DBG dev[%u] %-24.24s mod=%u ev=%u ok=%u fails=%u,%u,%u,%u,%u\n",
                   i, snf_devs[i].name, (unsigned)snf_devs[i].modulation,
                   (unsigned)snf_devs[i].decode_events,
                   (unsigned)snf_devs[i].decode_ok,
                   (unsigned)snf_devs[i].decode_fails[0],
                   (unsigned)snf_devs[i].decode_fails[1],
                   (unsigned)snf_devs[i].decode_fails[2],
                   (unsigned)snf_devs[i].decode_fails[3],
                   (unsigned)snf_devs[i].decode_fails[4]);
    }
}
#endif

/* ── Frame finalize: run-lengths → pulse_data_t → demods → JSON ─────── */

/* JSON writer: append into a fixed buffer; truncation is silent but
 * guarded (SNF_JSON_BUFLEN is generous for one sensor reading). */
#define SNF_JSON_BUFLEN 512

typedef struct {
    char *p;
    size_t left;
} snf_jw_t;

static void jw_char(snf_jw_t *w, char c)
{
    if (w->left > 1) {
        *w->p++ = c;
        *w->p   = '\0';
        w->left--;
    }
}

static void jw_str(snf_jw_t *w, char const *s)
{
    while (*s) {
        jw_char(w, *s++);
    }
}

/* Minimal JSON string escaping: ", \, control chars -> \u00XX. */
static void jw_json_str(snf_jw_t *w, char const *s)
{
    jw_char(w, '"');
    while (*s) {
        unsigned char c = (unsigned char)*s++;

        if (c == '"') {
            jw_str(w, "\\\"");
        } else if (c == '\\') {
            jw_str(w, "\\\\");
        } else if (c < 0x20u || c == 0x7Fu) {
            char esc[8];
            snprintf(esc, sizeof(esc), "\\u%04X", (unsigned)c);
            jw_str(w, esc);
        } else {
            jw_char(w, (char)c);
        }
    }
    jw_char(w, '"');
}

/* Double as %.1f with pure integer math (no printf float support
 * required, deterministic rounding). */
static void jw_double1(snf_jw_t *w, double v)
{
    long long tenths;
    long long ipart;
    unsigned frac;
    char tmp[32];
    int pos = 0;

    tenths = (long long)(v * 10.0 + (v >= 0.0 ? 0.5 : -0.5));
    if (tenths < 0) {
        jw_char(w, '-');
        tenths = -tenths;
    }
    ipart = tenths / 10;
    frac  = (unsigned)(tenths % 10);

    /* integer part without snprintf */
    if (ipart == 0) {
        tmp[pos++] = '0';
    }
    while (ipart > 0) {
        tmp[pos++] = (char)('0' + (ipart % 10));
        ipart /= 10;
    }
    while (pos > 0) {
        jw_char(w, tmp[--pos]);
    }
    jw_char(w, '.');
    jw_char(w, (char)('0' + frac));
}

static void snf_data_print_obj(snf_jw_t *w, data_t const *data, int depth);

static void jw_value(snf_jw_t *w, data_t const *d, int depth)
{
    switch (d->type) {
    case DATA_INT:
        snprintf(w->p, w->left, "%d", d->value.v_int);
        w->left -= strlen(w->p);
        w->p += strlen(w->p);
        break;
    case DATA_DOUBLE:
        jw_double1(w, d->value.v_dbl);
        break;
    case DATA_STRING:
        if (d->value.v_ptr) {
            jw_json_str(w, (char const *)d->value.v_ptr);
        } else {
            jw_str(w, "\"\"");
        }
        break;
    case DATA_DATA:
        if (depth > 0 && d->value.v_ptr) {
            snf_data_print_obj(w, (data_t *)d->value.v_ptr, depth - 1);
        } else {
            jw_str(w, "{}");
        }
        break;
    case DATA_ARRAY: {
        data_array_t *a = (data_array_t *)d->value.v_ptr;

        jw_char(w, '[');
        if (a) {
            for (int i = 0; i < a->num_values; i++) {
                if (i > 0) {
                    jw_char(w, ',');
                }
                if (a->type == DATA_INT) {
                    snprintf(w->p, w->left, "%d", ((int *)a->values)[i]);
                    w->left -= strlen(w->p);
                    w->p += strlen(w->p);
                } else if (a->type == DATA_STRING) {
                    jw_json_str(w, ((char **)a->values)[i]);
                } else if (a->type == DATA_DOUBLE) {
                    jw_double1(w, ((double *)a->values)[i]);
                }
            }
        }
        jw_char(w, ']');
        break;
    }
    default:
        jw_str(w, "null");
        break;
    }
}

/* Nested object writer (DATA_DATA values): plain object, no prefix,
 * no model injection. */
static void snf_data_print_obj(snf_jw_t *w, data_t const *data, int depth)
{
    int first = 1;

    jw_char(w, '{');
    for (data_t const *d = data; d; d = d->next) {
        if (d->key == NULL) {
            continue;
        }
        if (!first) {
            jw_char(w, ',');
        }
        first = 0;
        jw_json_str(w, d->key);
        jw_char(w, ':');
        jw_value(w, d, depth);
    }
    jw_char(w, '}');
}

/* One data item = one "OOK JSON {...}" console line.  "model" is
 * injected from the device name when the decoder did not supply one
 * (all curated decoders do); the decoder's own "id" is kept verbatim. */
static void snf_data_print_json(data_t *data, char const *device_name)
{
    snf_jw_t w;
    char buf[SNF_JSON_BUFLEN];
    int has_model = 0;
    int first     = 1;

    w.p    = buf;
    w.left = sizeof(buf);
    buf[0] = '\0';

    for (data_t const *d = data; d; d = d->next) {
        if (d->key != NULL && strcmp(d->key, "model") == 0) {
            has_model = 1;
            break;
        }
    }

    jw_str(&w, "OOK JSON {");
    if (!has_model) {
        jw_str(&w, "\"model\":");
        jw_json_str(&w, device_name);
        first = 0;
    }
    /* Capture-frequency context: which OOK band produced this frame
     * (the multi leg rotates 433.92 / 868.35 MHz). */
    if (!first) {
        jw_char(&w, ',');
    }
    first = 0;
    jw_str(&w, "\"freq\":");
    snprintf(w.p, w.left, "%u", (unsigned)snf_ook_freq_hz);
    w.left -= strlen(w.p);
    w.p += strlen(w.p);

    for (data_t const *d = data; d; d = d->next) {
        if (d->key == NULL) {
            continue;
        }
        if (!first) {
            jw_char(&w, ',');
        }
        first = 0;
        jw_json_str(&w, d->key);
        jw_char(&w, ':');
        jw_value(&w, d, 4);
    }
    jw_char(&w, '}');

    SNF_PRINTF("%s\n", buf);
}

/* Decoder priority-ordered dispatch — mirror of the pinned master's
 * run_ook_demods() switch.  FSK cases are intentionally absent: this
 * capture path is OOK-only and never produces FSK pulse data. */
static int snf_run_ook_demods(pulse_data_t *pulses)
{
    int events = 0;
    unsigned next_priority = 0;

    for (unsigned priority = 0; !events && priority < UINT_MAX;
         priority = next_priority) {
        next_priority = UINT_MAX;
        for (unsigned i = 0; i < snf_n_devs; i++) {
            r_device *dev = &snf_devs[i];

            if (dev->priority > priority && dev->priority < next_priority) {
                next_priority = dev->priority;
            }
            if (dev->priority != priority) {
                continue;
            }

            switch (dev->modulation) {
            case OOK_PULSE_PCM:
                events += pulse_slicer_pcm(pulses, dev);
                break;
            case OOK_PULSE_PPM:
                events += pulse_slicer_ppm(pulses, dev);
                break;
            case OOK_PULSE_PWM:
                events += pulse_slicer_pwm(pulses, dev);
                break;
            case OOK_PULSE_MANCHESTER_ZEROBIT:
                events += pulse_slicer_manchester_zerobit(pulses, dev);
                break;
            case OOK_PULSE_PIWM_RAW:
                events += pulse_slicer_piwm_raw(pulses, dev);
                break;
            case OOK_PULSE_PIWM_DC:
                events += pulse_slicer_piwm_dc(pulses, dev);
                break;
            case OOK_PULSE_DMC:
                events += pulse_slicer_dmc(pulses, dev);
                break;
            case OOK_PULSE_PWM_OSV1:
                events += pulse_slicer_osv1(pulses, dev);
                break;
            case OOK_PULSE_NRZS:
                events += pulse_slicer_nrzs(pulses, dev);
                break;
            case OOK_PULSE_RZI:
                events += pulse_slicer_rzi(pulses, dev);
                break;
            default:
                /* FSK modulations cannot be demodulated from the OOK
                 * envelope — skip silently (upstream prints a warning,
                 * we don't even register FSK devices). */
                break;
            }
        }
    }
    return events;
}

/* Convert the accumulator run-lengths into a classic pulse_data_t.
 * 1-runs become pulse widths, 0-runs become gap widths; both multiplied
 * by the bit-clock period (SNF_OOK_BIT_US µs per bit — derived from the
 * Kconfig OOK bit rate).  sample_rate = 1e6 keeps all upstream
 * timing constants in microseconds unchanged. */
static void snf_finalize(void)
{
    static pulse_data_t pulses;
    uint32_t n = snf_acc_bits;
    uint32_t i;
    unsigned np = 0;
    unsigned cur, run = 0;

    if (n == 0) {
        return;
    }

    memset(&pulses, 0, sizeof(pulses));
    pulses.sample_rate = 1000000u;

    /* Skip leading zeros (noise floor / inter-frame gap). */
    i = 0;
    while (i < n && snf_get_bit(i) == 0u) {
        i++;
    }
    if (i >= n) {
        return; /* all zeros — nothing to decode */
    }

    cur = 1u;
    for (; i < n && np < PD_MAX_PULSES; i++) {
        unsigned b = snf_get_bit(i);

        if (b == cur) {
            run++;
        } else {
            if (cur == 1u) {
                pulses.pulse[np] = (int)run * (int)SNF_OOK_BIT_US;
            } else {
                pulses.gap[np] = (int)run * (int)SNF_OOK_BIT_US;
                np++;
            }
            cur = b;
            run = 1;
        }
    }
    if (run > 0 && np < PD_MAX_PULSES) {
        if (cur == 1u) {
            pulses.pulse[np] = (int)run * (int)SNF_OOK_BIT_US;
            /* Accumulator ended mid-pulse (possible on the >3000-bit
             * force path): synthesize a 30 ms terminator gap so the
             * demods see end-of-transmission. */
            pulses.gap[np] = (int)(SNF_GAP_END_BITS * SNF_OOK_BIT_US);
        } else {
            pulses.gap[np] = (int)run * (int)SNF_OOK_BIT_US;
        }
        np++;
    }
    pulses.num_pulses = np;

    if (np >= PD_MAX_PULSES) {
        SNF_PRINTF("OOK decode: pulse table full, frame truncated\n");
    }

    /* Force a terminating gap when the tail is short (possible when the
     * >3000-bit cap fired mid-transmission): demods end-of-message check
     * is gap > reset_limit. */
    if (pulses.num_pulses > 0 &&
        pulses.gap[pulses.num_pulses - 1] < (int)(SNF_GAP_END_BITS * SNF_OOK_BIT_US)) {
        pulses.gap[pulses.num_pulses - 1] = (int)(SNF_GAP_END_BITS * SNF_OOK_BIT_US);
    }

    snf_run_ook_demods(&pulses);

#ifdef SNF_RTL433_HOST
    snf_finalize_count++;
#endif

    /* Print first, reset after: the JSON above was emitted while the
     * decoded data was alive; now rewind the arena for the next frame. */
    snf_mem_reset();
    snf_acc_bits = 0;
}

/* ── Public feed API ─────────────────────────────────────────────────── */

void snf_rtl433_feed(const uint8_t *chunk, uint16_t chunk_len)
{
    /* Firmware deliveries are <= PLD_LEN (240 bits); larger buffers
     * (sandbox tests) are accepted as-is. */
    if (chunk == NULL || chunk_len == 0) {
        return;
    }

    for (uint16_t byte_i = 0; byte_i < chunk_len; byte_i++) {
        uint8_t byte = chunk[byte_i];

        for (unsigned bit_i = 0; bit_i < 8u; bit_i++) {
            unsigned bit;
#if SNF_OOK_BIT_LSB_FIRST
            bit = (byte >> bit_i) & 1u;
#else
            bit = (byte >> (7u - bit_i)) & 1u;
#endif
            if (snf_acc_bits >= (uint32_t)CONFIG_ZEPHCORE_SNIFFER_DECODE_BUF_BITS) {
                /* Overflow: drop the whole pending accumulator, the
                 * frame is hopeless. */
                SNF_PRINTF("OOK decode: accumulator overflow, dropped\n");
                snf_acc_bits = 0;
            }
            {
                uint8_t mask = (uint8_t)(1u << (7u - (snf_acc_bits & 7u)));
                /* Set OR clear — a zero bit must overwrite any stale bit
                 * left over from the previous frame in this byte
                 * (the accumulator is not zeroed between frames). */
                if (bit) {
                    snf_acc[snf_acc_bits >> 3] |= mask;
                } else {
                    snf_acc[snf_acc_bits >> 3] &= (uint8_t)~mask;
                }
            }
            snf_acc_bits++;
        }
    }

    /* End-of-transmission heuristic: >= 30 ms of zero bits at the tail
     * of the accumulator, or the safety cap. */
    if (snf_acc_bits > SNF_FORCE_FINALIZE_BITS || snf_tail_zeros() >= SNF_GAP_END_BITS) {
        snf_finalize();
    }
}
