# ZephCore LR2021 Dual-Band — GitHub Publikacija

**Handoff iš 2026-08-08 sesijos** · Next agent: publikuoti į GitHub

---

## 1. KAS VEIKIA (patvirtinta on-air 2026-08-08)

### Trys build'ai (visi CI praeina, run 31270452360)

| Build | Kconfig flag'ai | Paskirtis |
|---|---|---|
| **repeater** (default) | `CONFIG_ZEPHCORE_REPEATER=y` | Vienadažnis kartotuvas (869 MHz), be TDM |
| **companion-tdm** | `CONFIG_ZEPHCORE_RADIO_TDM=y` | Dvidažnis: sub-GHz primary + 2.4 GHz langai |
| **companion-2g4** | `CONFIG_ZEPHCORE_BAND_2G4=y` | Tik 2.4 GHz (sub-GHz išjungtas) |
| **repeater-tdm** | `CONFIG_ZEPHCORE_REPEATER=y CONFIG_ZEPHCORE_RADIO_TDM=y` | Dvidažnis kartotuvas (tiltas tarp juostų) |

### Įrodyta on-air

- ✅ 2.4 GHz ping-pong ≥8/10 (L2, 2026-08-02)
- ✅ TDM ciklai veikia — HF langas 70 ms kas 1.5 s (L3)
- ✅ Dvidažnis tiltas: sub-GHz flood → 2.4 GHz kopija (L4-U5)
- ✅ HF švyturiai: kas 60 s skelbia 2.4 GHz parametrus (L4-U6)
- ✅ **Pilna grandinė**: telefonas → companion → (sub-GHz + 2.4 GHz) → repeater → pasaulis
- ✅ **2.4 GHz-only companion** pasiekia visą mesh tinklą tik per mūsų repeater'į (2026-08-08)

### Bug fix'ai (branch `lr2021-rx-order-fix`)

| Commit | Aprašymas |
|---|---|
| `a600be9` | RSSI fallback — `lr_rssi_effective()` vietoj `snr<0` sąlygos |
| `3a23ec9` | Pašalintas `tx_active=false` iš `lr20xx_switch_band()` — fix'ina `hwSendAsync -16` |

## 2. REPO STRUKTŪRA

### Branch: `lr2021-rx-order-fix`

### Pagrindiniai dual-band failai
```
zephcore/
├── adapters/radio/
│   ├── DualBandRadio.{h,cpp}      # TDM scheduler (Zephyr glue)
│   ├── dualband_tdm.h             # Pure TDM state machine (sandbox-tested)
│   ├── dualband_route.h           # TX band routing rules
│   ├── dualband_beacon.h          # HF beacon encode/decode
│   └── LoRaRadioBase.{h,cpp}      # Base class — rx_band, force_hf_modem
├── patches/zephyr-new/drivers/lora/lr20xx/
│   ├── lr20xx_lora.c              # Driver — lr20xx_switch_band(), rx_path_for_freq()
│   └── lr20xx_lora.h              # Driver API
├── src/
│   └── Dispatcher.cpp             # checkRecv/checkSend — dual-band routing
└── app/
    └── RepeaterMesh.cpp           # getTxBandMask(), HF beacon, neighbour table
```

### CI workflow
- Failas: `.github/workflows/build-nrf54l15.yml`
- Trigger: `gh workflow run build-nrf54l15.yml -R protozauras/ZephCore --ref lr2021-rx-order-fix`
- Artefaktai: 6 build'ai (companion, companion-2g4, companion-tdm, repeater, repeater-tdm, repeater-power-debug)

### Sandbox
- `tools/lr2021_sim/` — 22 testai, testuoja `dualband_tdm.h` ir `dualband_route.h` tiesiogiai
- Paleidimas: `./lr2021_sim_tests.exe`

## 3. NELIESK

- sub-GHz parametrai (869.618 / BW62 / SF8 / CR4/8 / sync 0x12 / TX 22 dBm)
- TX kelias · RX skaitymo eilė (#37) · peek-IRQ (#14)
- 0x0212 16-bit parse
- `SET_RX_DUTY_CYCLE` (broken)
- Peer'io (GAT562) flash'inti NEGALIMA
- TX > 22 dBm NEGALIMA

## 4. PRIEŠ PAVIEŠINANT

1. **Production mode** — `CONFIG_ZEPHCORE_PRODUCTION=y` (be log'ų, mažesnis binary)
2. **README** — aprašyti LR2021 + dual-band + build variantus
3. **Padaryti release** su 4 hex failais
4. **Nukreipti į `master`** arba naują `release` branch'ą

## 5. ATEITIES PLANAI

### Observer per 2.4 GHz
Kalno repeater'is siunčia visus gautus paketus per 2.4 GHz → namų companion → internetas (MQTT). Realus srautas: ~40 paketų/min (4 kB/min). Kodas jau turi `CONFIG_ZEPHCORE_REPEATER_UPLINK` + MQTT — reikia tik nukreipti per 2.4 GHz kanalą.

### Kiti
- APC (Adaptive Power Control) — kai tinklas tankesnis
- SIMO/DC-DC režimas — elektros taupymui (datasheet: SIMO 5.7 mA, LDO ? mA)
- RX Boost išjungimas (-2 mA) — kai ryšys geras

## 6. PLOKŠČIŲ BŪSENA

| UID | COM | Firmware dabar |
|---|---|---|
| `64206A53` | COM8 | repeater-tdm (2026-08-08) |
| `8802F48F` | COM9 | companion-tdm (2026-08-08) |

## 7. KOMANDOS

```bash
# CI build
gh workflow run build-nrf54l15.yml -R protozauras/ZephCore --ref lr2021-rx-order-fix

# Flash
pyocd flash -t nrf54l -u 64206A53 firmware/zephyr-repeater-tdm-*.hex

# Serial capture
python tools/capture_serial.py 60 /tmp/log.txt COM8

# CLI (byte-by-byte — TDM spam resistant)
python C:/tmp/cli_probe.py COM8

# Sandbox
cd tools/lr2021_sim && ./lr2021_sim_tests.exe
```
