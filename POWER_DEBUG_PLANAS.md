# POWER DEBUG BUILD — Planas Agentui (2026-08-07)

> **Tikslas:** Rasti kur dingsta elektra. Repeater ima ~111 mA nuo baterijos, ~24 mA per USB (be carrier). Teorinės grindys ~7-8 mA.
> **Vartotojo nurodymas:** "agentas turi pasidaryti build grynai matavimams. jis matys kur nuteka elektra, arba kas labiausiai naudoja."
> **Baterija ne prie ko** — įrenginys per daug ima ir su baterija, ir be jos.
>
> **🟢 NAUJA VARTOTOJO DIREKTYVA (2026-08-07, vakaras):** factory-defaults testas — atstatyti plokštę į gamyklinius parametrus, išjungti baterijų krovimus ir kitką pagal Seeed Studio nurodymus, pamatuoti po factory defaults, TIK TADA su vartotojo leidimu suflašinti mūsų firmware. Pilnas veiksmų planas: `ELEKTRA_NEXT_AGENT_ATASKAITA.md` §8.

---

## 1. BŪSENA (ką žinoti prieš pradedant)

### Dabartinė aparatūra
| Įrenginys | UID | COM | Firmware | Matuota srovė |
|---|---|---|---|---|
| Repeater (A) | `8802F48F` | COM9 | repeater-TDM `5c1491b` | 111.8 mA (bat.), 24 mA (USB be carrier), 31.5 mA (USB su carrier) |
| Companion (B) | `64206A53` | COM8 | companion-TDM `bd3431d` | 8 mA (USB be carrier) |

- Abu XIAO identiški, iš vienos dėžutės
- USB matuoklis — grubus (absoliutūs skaičiai ± keli mA, skirtumai reikšmingi)
- Multimetras — 200 mA riba

### Ką jau žinome
- **16 mA skirtumas ant plikų XIAO (8 vs 24) = firmware/build lygmens problema**
- **TPS62843 near-dropout** — dokumentuota problema kai VSYS <3.3V; gali paaiškinti dalį 111.8 mA
- **SGM40567 kroviklis** — hardware; Charge LED gali mirksėti (~1-2 mA)
- **LR2021 veikia LDO režimu** (SIMO_OFF default) — visos datasheet srovės (5.7 mA) matuotos su SIMO
- **LOG_DEFAULT_LEVEL=3** — daug diagnostikos per UART
- **nRF54L15 DCDC (VREGMAIN)** — nežinoma ar įjungtas

### NELIESK (kartojama iš kanoninių failų)
- Sub-GHz radio parametrai (869.618/BW62/SF8/CR4/8/sync0x12/TX22)
- TX kelias, RX skaitymo eilė (#37), peek-IRQ (#14), 0x0212 (#1)
- `SET_RX_DUTY_CYCLE` (broken)
- Peer'io (GAT562) NEGALIMA flash'inti
- Sandbox `tools/lr2021_sim` turi likti 22/22

---

## 2. POWER DEBUG BUILD — Ką Reikia Padaryti

### 2.1 Naujas Kconfig: `CONFIG_ZEPHCORE_POWER_DEBUG`

Sukurti naują Kconfig simbolį kuris įjungia visą power diagnostiką:

```kconfig
config ZEPHCORE_POWER_DEBUG
    bool "Power debug instrumentation"
    default n
    help
      Enables radio state timing, TX count tracking, CPU idle stats,
      and periodic power summary logging for energy profiling.
```

### 2.2 Radio Būsenos Timing'as

Failas: `zephcore/adapters/radio/DualBandRadio.cpp` (arba naujas `power_debug.cpp`)

**Ką sekti:**
```c
// Laiko stamp'ai kiekvienam radio perėjimui
static uint64_t _radio_state_enter_ms;   // kada įėjo į dabartinę būseną
static uint32_t _time_in_rx_ms;          // akumuliuotas RX laikas
static uint32_t _time_in_standby_ms;     // akumuliuotas STBY laikas
static uint32_t _time_in_tx_ms;          // akumuliuotas TX laikas
static uint32_t _time_in_hf_window_ms;   // TDM HF lango laikas
static uint32_t _tx_count;               // TX skaičius
static uint32_t _hf_tx_count;            // HF TX skaičius
static uint64_t _last_summary_ms;        // kada paskutinį kartą išvedėm statistiką
```

**Hook'ai:**
- `lr20xx_start_rx()` → `_radio_state_enter_ms = k_uptime_get()`
- `lr20xx_set_standby()` → akumuliuoti RX laiką, pradėti STBY
- TX pradžia → akumuliuoti STBY, pradėti TX; `_tx_count++`
- TX pabaiga (DIO handler) → akumuliuoti TX laiką
- TDM HF lango pradžia/pabaiga → akumuliuoti `_time_in_hf_window_ms`

### 2.3 Periodinė Power Suvestinė

Kas **10 sekundžių** (per `k_work_delayable`) išvesti:

```
========== POWER SUMMARY (10s) ==========
Radio state times:
  RX:         8234 ms (82.3%)
  Standby:    1456 ms (14.6%)
  TX:          310 ms ( 3.1%)   [3 TX, avg 103 ms each]
  HF window:   700 ms ( 7.0%)   [10 windows, avg 70 ms]
TX counts:
  sub-GHz:     1  (HF beacon)
  HF:          2  (HF beacon copy + mesh forward)
Radio diag events:
  post-SET_RX: 27
  DIO1 irq:     5
  CAD result:   3
  RX errors:    1
CPU/MCU:
  uptime:       3600 s
  main_loop_runs: 120
==========================================
```

### 2.4 Srovės Įverčiai (iš logų)

Agentas gali **pats apskaičiuoti** teorinę srovę:

| Būsena | Tipinė srovė (iš datasheet) |
|---|---|
| LR2021 RX (LDO, sub-GHz) | ~6.5 mA (LDO ≈ +15% virš 5.7 SIMO) |
| LR2021 STBY_RC | 1.21 mA |
| LR2021 TX +22 dBm | ~105 mA (peak, ~100 ms) |
| LR2021 TX +12 dBm (HF) | ~27 mA (datasheet LF PA) |
| nRF54L15 idle (WFI) | ~0.03 mA |
| nRF54L15 active (64 MHz) | ~3.5 mA |
| SAMD11 CMSIS-DAP | ~0.5 mA |
| RF switch | ~0.5 mA |

**Vidutinė srovė = Σ(būsenos_dalis × būsenos_srovė)**

Agentas apskaičiuoja:
```
srovė = (RX_% × 6.5 + STBY_% × 1.21 + TX_% × 105 + HF_% × 27) + MCU_3.5 + SAMD11_0.5 + RF_0.5
```

Jei apskaičiuota ≠ išmatuota → yra "nematomas" vartotojas (GPIO, kroviklis, TPS62843 artefaktas).

### 2.5 CPU Idle Matavimas

nRF54L15 neturi power-states, bet galima stebėti kiek kartų per sekundę sukasi main loop'as:
- Jei 100+ kartų/s → CPU visada aktyvus (WFI neįeina)
- Jei 1-10 kartų/s → CPU daugiausia miega

```c
static uint32_t _main_loop_count = 0;
// main() ciklo pabaigoje:
_main_loop_count++;
```

---

## 3. Δ METODO BUILD'AI

Kiekvienas build'as = **vienas pakeitimas**. Agentas suflašina → paleidžia 60 s capture → išanalizuoja power suvestinę → palygina su baziniu.

| # | Build'as | Ką keičia | Ko tikėtis | Prioritetas |
|---|---|---|---|---|
| **B0** | **Bazinis** — dabartinis repeater-TDM su `POWER_DEBUG=y` | Prideda tik power debug kodą | Gauname baseline'ą su tiksliais % | 🔴 PIRMAS |
| **B1** | **SIMO/DC-DC radijui** | `lr20xx_system_set_reg_mode(DCDC 0x02)` po standby init'e | −2–5 mA (LDO→SIMO) | 🔴 |
| **B2** | **LOG_LEVEL=1** | `CONFIG_LOG_DEFAULT_LEVEL=1` (vietoj 3) | −0.5–2 mA (mažiau UART) | 🟡 |
| **B3** | **Be TDM** | `CONFIG_ZEPHCORE_RADIO_TDM=n` | −0.5–2 mA (be HF langų) | 🟡 |
| **B4** | **Be švyturio** | `CONFIG_ZEPHCORE_BEACON=n` / išjungti HF beacon | −~1.5 mA (be periodinių TX) | 🟢 |
| **B5** | **Radijas STBY** (testas) | Laikinai pakeisti `startRx` → `setStandby` | −5.7 mA (patikrina ar radijo RX tikrai ima tiek) | 🟢 |
| **B6** | **Be SAMD11 UART** | `CONFIG_UART_CONSOLE=n` | −~1 mA (UARTE + SAMD11) | 🟢 |

---

## 4. DARBO EIGA AGENTUI

### Žingsnis 0: Pasiruošimas
```bash
cd /c/Users/proto/ZephCore-nRF54L15-build/zephcore

# Patikrinti būseną
git log --oneline -5
pyocd list   # abi plokštės turi būti matomos (8802F48F, 64206A53)

# Sandbox prieš pradedant
cd ../tools/lr2021_sim && rm -f lr2021_sim_tests.exe && \
  "/c/Program Files/LLVM/bin/clang.exe" -std=c99 -Wall -O0 -g \
  stub_lr2021.c driver_under_test.c test_lr2021_driver.c \
  -o lr2021_sim_tests.exe && ./lr2021_sim_tests.exe
# Turi būti 22/22 PASS
```

### Žingsnis 1: Sukurti POWER_DEBUG Kconfig + kodą

1. Pridėti `config ZEPHCORE_POWER_DEBUG` į `zephcore/Kconfig` (radio menu)
2. Sukurti `zephcore/adapters/radio/power_debug.h` ir `.cpp`:
   - Radio būsenos timing'as (hook'ai į DualBandRadio)
   - Periodinė suvestinė kas 10 s
   - TX skaičiuoklė
   - CPU idle detector (main loop count)
3. Integruoti hook'us į esamą kodą taip kad **be `CONFIG_ZEPHCORE_POWER_DEBUG=y` jokio poveikio nebūtų** (viskas per `#if IS_ENABLED(CONFIG_ZEPHCORE_POWER_DEBUG)`)

### Žingsnis 2: CI Build B0 (bazinis)

```bash
# Trigger'inti CI su POWER_DEBUG
gh workflow run build-nrf54l15.yml -R protozauras/ZephCore --ref lr2021-rx-order-fix \
  -f extra_cmake_args="-DCONFIG_ZEPHCORE_POWER_DEBUG=y"
```

Jei CI nepalaiko extra cmake args, galima laikinai įjungti `CONFIG_ZEPHCORE_POWER_DEBUG=y` į `board.conf`.

### Žingsnis 3: Flash'inti B0 ant repeater'io (A, 8802F48F)

```bash
pyocd flash -t nrf54l -u 8802F48F <hex failas>
pyocd commander -t nrf54l -u 8802F48F -c reset
```

### Žingsnis 4: Capture 120 s + Analizė

```bash
python tools/capture_serial.py 120 "C:/tmp/power_b0.txt" COM9
```

Agentas **pats analizuoja** `power_b0.txt`:
- Ištraukia visas `POWER SUMMARY` eilutes
- Apskaičiuoja vidutinę srovę pagal formulę
- Palygina su vartotojo išmatuota verte
- Nustato didžiausią vartotoją
- Dokumentuoja rezultatą žemiau (sekcija §6)

### Žingsnis 5: Kartoti B1–B6

Po vieną build'ą: pakeisti vieną dalyką → CI build → flash → capture → analizuoti → įrašyti Δ.

---

## 5. POWER DEBUG KODO STRUKTŪRA (ką tiksliai rašyti)

### 5.1 `zephcore/adapters/radio/power_debug.h`

```c
#pragma once
#include <zephyr/kernel.h>

#if IS_ENABLED(CONFIG_ZEPHCORE_POWER_DEBUG)

// Radio būsenos (kad galėtume matuoti laiką kiekvienoje)
enum power_debug_radio_state {
    POWER_STATE_INIT,
    POWER_STATE_STANDBY,
    POWER_STATE_RX,
    POWER_STATE_TX,
    POWER_STATE_HF_WINDOW,
};

// Hook'ai — kviečiami iš DualBandRadio / driverio
void power_debug_enter_state(enum power_debug_radio_state state);
void power_debug_tx_done(void);       // +1 TX count, accumulate TX time
void power_debug_hf_tx_done(void);    // +1 HF TX count
void power_debug_radio_event(void);   // skaičiuoti radio diag įvykius
void power_debug_main_loop_tick(void); // +1 main loop run

// Periodinė suvestinė (kviečiama iš work queue)
void power_debug_print_summary(void);

// Init (paleidžia periodinį timer'į)
void power_debug_init(void);

#else
// No-ops kai POWER_DEBUG=n
#define power_debug_enter_state(s)      do {} while(0)
#define power_debug_tx_done()           do {} while(0)
#define power_debug_hf_tx_done()        do {} while(0)
#define power_debug_radio_event()       do {} while(0)
#define power_debug_main_loop_tick()    do {} while(0)
#define power_debug_init()              do {} while(0)
#endif
```

### 5.2 Hook'ų integravimo vietos

| Hook'as | Kur įterpti | Failas |
|---|---|---|
| `power_debug_enter_state(RX)` | `lr20xx_start_rx()` pabaigoje po sėkmingo `SetRx` | `lr20xx_lora.c` ~1915 |
| `power_debug_enter_state(STBY)` | `lr_set_standby()` + `lr20xx_switch_band()` grįžimo taške | `lr20xx_lora.c` |
| `power_debug_enter_state(TX)` | `lr20xx_start_tx()` pradžioje | `lr20xx_lora.c` |
| `power_debug_tx_done()` | TX_DONE DIO handler'yje | `lr20xx_lora.c` |
| `power_debug_enter_state(HF_WINDOW)` | `dm_step()` grąžina OPEN/EXTEND | `dualband_tdm.h` / DualBandRadio |
| `power_debug_radio_event()` | Kiekvieną kartą kai `LOG_DBG` radio diag | `lr20xx_lora.c` diag makro |
| `power_debug_main_loop_tick()` | `main()` ciklo pabaigoje (po `k_sleep` ar `k_yield`) | `main_repeater.cpp` |

### 5.3 Periodinės suvestinės formatas

Kad agentas galėtų lengvai pars'inti, naudoti fiksuotą formatą:

```
=== POWER_SUMMARY START ===
uptime_ms=60000
rx_ms=49230 tx_ms=3100 stby_ms=7310 hf_window_ms=3500
tx_count_subghz=1 tx_count_hf=1
radio_events=27 main_loops=1200
radio_pct_rx=82.1 radio_pct_stby=12.2 radio_pct_tx=5.2 radio_pct_hf=5.8
=== POWER_SUMMARY END ===
```

Agentas gali grep'inti `POWER_SUMMARY` ir pars'inti skaičius.

---

## 6. REZULTATŲ LENTELĖ (pildo agentas)

| Build'as | Keitimas | Srovė (vart.) | RX % | HF % | TX % | TX kiekis | Pastabos |
|---|---|---|---|---|---|---|---|
| B0 | Bazinis + debug (`afdc96e`) | 24 mA (vart. USB) | ~95-100 | 0-4.2 (pliūpsniais) | ~0.1 | 1 sub-GHz + 1 HF per 65 s švyturį | POWER_SUMMARY: rx_ms/hf_window_ms/tx_ms/tx_hf_ms/stby_ms, main_loops ~1.4/s, TX ~13 ms. |
| B1 | +SIMO/DC-DC (0x02) (`05f26cd`) | 30 mA (vart. matavimas) | ~95-100 | 0-4.2 (pliūpsniais) | ~0.1 | 2 sub-GHz + 2 HF per 120 s | 30 langų, 1 švyturys, 1 CAD timeout, 0 naujų klaidų. Revertinta `4e86f0f`. |
| B2 | +LOG=1 | ? mA | ? | ? | ? | ? | Δ nuo B1 = ? |
| B3 | +be TDM | ? mA | ? | ? | ? | ? | Δ nuo B2 = ? |
| ... | ... | ... | ... | ... | ... | ... | ... |

### B0 STEBĖJIMAI (2026-08-07, `afdc96e`, 2× capture: 120 s + 150 s, faktai iš POWER_SUMMARY)

1. Radio laikas: sub-GHz RX ~93-100%, HF langas 0-4.2% (pliūpsniais), TX <0.1% (švyturys kas 65 s, ~13 ms), stby ~0 (po boot).
2. main_loops: ~1.4/s (pirmas 60 s su LED mirksėjimu ~2/s, po to ~0.2-0.3/s housekeeping).
3. TDM HF langai: veikia po boot ~10-30 s, paskui sustoja iki kito švyturio TX; po TX vėl pliūpsnis. (Tas pats modelis 5c1491b loguose 2026-08-06.)
4. Matavimo pastaba: log laikrodis = RTC sieninis (stale reikšmė), POWER_SUMMARY laikas = uptime.

