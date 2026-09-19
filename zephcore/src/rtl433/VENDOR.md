# Vendored rtl_433 — on-chip OOK decode for the RF sniffer

Upstream: https://github.com/merbanan/rtl_433
Pinned commit: `a558034cae051651f6969b77518f57a88f6a232e` (master, 2026-09-19 clone)

License: **GPL-2.0-or-later** (rtl_433 is GPL-2.0+; the ZephCore sniffer
role links it as part of the firmware — keep this in mind for
distribution).  The glue around it (sniffer_rtl433.c/.h, rtl433_mem.h)
is MIT and ZephCore's own code.

## Why this commit / layout

The phase-3 plan referenced classic rtl_433 files (`pulse_demod.c`,
`lacrosse_tx141thbv2.c`, `lacrosse_tx141.c`, `util.c`).  The pinned
master has moved on since:

- `pulse_demod.c` no longer exists; the demods are `pulse_slicer.c`
  (`pulse_slicer_pcm/ppm/pwm/manchester_zerobit/piwm_raw/piwm_dc/dmc/
  osv1/nrzs/rzi`) and the master dispatch is `run_ook_demods()` in
  `r_api.c` — the glue mirrors exactly that switch.  The old
  `OOK_PULSE_PWM_RAW / PWM_PRECISE / PPM_RAW / PCM` names are the new
  `OOK_PULSE_PIWM_RAW / PWM (with tolerance) / PPM / PCM` entries.
- `lacrosse_tx141thbv2.c` + `lacrosse_tx141.c` were merged upstream into
  `lacrosse_tx141x.c` (one `r_device lacrosse_tx141x` covering TX141,
  TX141TH-Bv2, TX141-Bv3).
- WH1080 moved out of `fineoffset.c` into `fineoffset_wh1080.c` and is
  **FSK** in this master — out of scope for an OOK-only capture path,
  so it is deliberately NOT vendored.  WH2/WH5/Telldus + WH0530 (both
  OOK_PULSE_PWM) are in `fineoffset.c` and ARE vendored.
- Oregon Scientific v1 moved to `oregon_scientific_v1.c` (vendored);
  v2_1/v3 remain in `oregon_scientific.c` (vendored).
- Decoders report data via `data_make()` + `decoder_output_data()` and
  the registry wires `output_fn` — the glue follows the pinned master's
  `data_acquired_handler` flow, NOT an older guess.

## Vendored files (copied verbatim except the SNF_RTL433 guards below)

Headers (upstream `include/`):
`bitbuffer.h` `bit_util.h` `data.h` `r_device.h` `decoder.h`
`decoder_util.h` `pulse_data.h` `pulse_slicer.h` `c_util.h` `fatal.h`
`logger.h` `abuf.h` `list.h`

Sources (upstream `src/`):
`bitbuffer.c` `bit_util.c` `data.c` `abuf.c` `list.c` `decoder_util.c`
`pulse_slicer.c`

Decoders (upstream `src/devices/`):
`acurite.c` `lacrosse_tx141x.c` `oregon_scientific.c`
`oregon_scientific_v1.c` `fineoffset.c` `prologue.c` `hideki.c`

NOT vendored: `pulse_detect_classic.c`/`pulse_detect.c` (SDR digitizer
stage — the glue replaces it with the bitstream→pulse_data conversion),
`rtl_433.c`, `r_api.c`, `pulse_analyzer.c`, `logger.c` (the glue
provides `print_log`/`print_logf`), `compat_time.h` (dead include after
the pulse_data.h guard), anything FSK-only.

## Modifications vs upstream (all wrapped in `#ifdef SNF_RTL433`)

1. `pulse_data.h` — the `#include "compat_time.h"` is replaced by a
   `struct timeval;` forward declaration (Zephyr minimal libc has no
   `<sys/time.h>`; `pulse_data_load` is never called here).
2. `pulse_slicer.h` — the `#include "pulse_detect.h"` (SDR digitizer
   stage, not vendored) is replaced by `#include "pulse_data.h"`; the
   slicers only need `pulse_data_t`.
3. `fatal.h` — `FATAL*`/`WARN*` collapse to `((void)0)` (no `fprintf`/
   `exit` in firmware; the arena allocator signals failure with NULL).
4. `bitbuffer.c` — 2 debug `fprintf(stderr, ...)` row/col limit warnings
   guarded out.
5. `data.c` — 3 `fprintf(stderr, ...)` misuse diagnostics guarded out.
6. `pulse_slicer.c` — the `exit(1)` on an invalid decoder return value
   becomes `ret = 0` (a decoder bug must not kill the firmware).
   (`bit_util.c` needs no patch: its `fprintf` sites are in commented
   out helper functions.)

## Memory model

`rtl433_mem.h` (force-included into every vendored TU) rebinds
malloc/calloc/realloc/free to a static ~32 KiB bump arena:
realloc = alloc+copy+no-free-old, free = no-op, `snf_mem_reset()`
rewinds the arena once per finalized frame (print first, reset after).
strdup is provided on the arena (minimal libc has no strdup).
`RTL_433_REDUCE_STACK_USE` is defined: bitbuffer_t shrinks from ~6.4 KiB
to ~1 KiB so the per-frame slicer stack fits the Zephyr main thread.

## Registry

Curated OOK decoders (all `OOK_PULSE_*`): acurite_rain_896,
acurite_th, acurite_txr, acurite_986, acurite_985, acurite_606,
acurite_00275rm, acurite_590tx, lacrosse_tx141x, oregon_scientific,
oregon_scientific_v1, fineoffset_WH2, fineoffset_WH0530, prologue,
hideki_ts04.  `fineoffset_wh5rb` (disabled=1 upstream, collides with
WH5) and every FSK device are not registered.
