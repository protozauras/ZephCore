# LR2021 driver sandbox

**Why this exists:** we needed a way to validate the LR2021 command-layer
logic without flashing the XIAO nRF54L15 every iteration. Each CI build
+ pyocd flash used to take 5–10 minutes and the user capped builds on
2026-07-31. This sandbox lets us exercise `lr_cmd()` and its wrappers
in seconds, on the development host, with deterministic answers.

**What it covers**

- The `lr_cmd()` single-NSS read branch (the fix from `215f6d8` and
  `a9592e5`) for every read command used on the RX path.
- The legacy CS-toggle pathology that produced the "length is the IRQ
  word" bug (boot_log7 7/7 correlation). We deliberately *inject* it
  in the stub and verify the fixed driver still reads real data.
- The DIO IRQ → handler → `get_lora_pkt_status` → `get_rx_pkt_length`
  → `fifo_read` chain that `lr20xx_dio1_work_handler` runs on RX_DONE.
- The TX_DONE round trip.
- IRQ clear-on-read semantics of `lr_get_and_clear_irq`.

**What it does NOT cover**

- The Zephyr bus mutex / k_work glue.
- SPI bus contention or real HW timing.
- BLE / app layer / mesh routing.
- Actual RF reception — the stub returns whatever bytes we tell it to.

## Layout

```
tools/lr2021_sim/
├── stub_lr2021.h        — chip model API (g_stub + spi_transceive_dt)
├── stub_lr2021.c        — fake LR2021: command-aware atsakymai pagal datasheet
├── driver_under_test.h  — public API of the layer under test
├── driver_under_test.c  — copies of lr_cmd() + wrapper'ių is lr20xx_lora.c
├── test_lr2021_driver.c — 7 unit tests
├── Makefile
└── README.md
```

## How to run

```bash
cd tools/lr2021_sim
make           # builds + runs
make test      # runs only
make clean     # removes build outputs
```

Requires a vanilla C compiler (`cc` / `gcc` / `clang`). No Zephyr SDK,
no libopencm3 — just plain ISO C99.

## Expected output (all green)

```
==== LR2021 driver sandbox ====
Architecture: stub-only, no Zephyr, no flash

  [PASS] test_read_returns_real_data
  [PASS] test_get_and_clear_irq_clears_irq
  [PASS] test_get_rx_pkt_length_unaffected_by_legacy_bug
  [PASS] test_get_lora_packet_status_returns_st_len
  [PASS] test_rx_done_handler_flow
  [PASS] test_stub_echo_when_cs_toggled
  [PASS] test_tx_done_flow
  [PASS] test_rx_order_regression
  [PASS] test_rx_fifo_drain

==== 9 tests run, 0 failures ====
```

Exit code: 0 on success, 1 on any failure.

## How to extend

When the real `lr20xx_lora.c` is edited (rx path, IRQ handling, etc.):

1. Re-read the affected wrapper in this sandbox (`driver_under_test.c`)
   and update it to match the production code, byte-for-byte.
2. Add a test in `test_lr2021_driver.c` that uses one of the existing
   helpers (`stub_fake_irq_fire_rx_done`, `stub_inject_packet`,
   `stub_set_force_cs_toggle`, `stub_dio_pin_read`) to construct
   the bug-or-fix scenario.
3. Run `make test`.
4. Only flash once the sandbox goes green on the targeted change.

## When NOT to use this

- If the change touches Zephyr HAL (mutex, GPIO, syscalls) — those
  require either real HW or `nrf54l15bsim` (BabbleSim), not this stub.
- If the change touches time-based pacing (CAD, duty cycle) — the stub
  has no clock; Zephyr `native_sim` is the right tool.
