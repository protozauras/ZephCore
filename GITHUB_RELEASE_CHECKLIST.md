# GitHub Publikavimo Kontrolinis Sąrašas

**Būsena:** 🔴 Laukiama lauko testo (dual-band 2.4 GHz per atstumą)  
**Data:** 2026-08-07

---

## Prieš publikavimą

### 1. Lauko testas ✅/❌
- [ ] 2.4 GHz dual-band veikia per atstumą (≥1 km pageidautina)
- [ ] Sub-GHz 868 MHz veikia kaip anksčiau (jokių regresijų)
- [ ] App rodo teisingą RSSI (0dBi fix patikrintas lauke)
- [ ] Repeater admin login veikia per RF (su teisingu laikrodžiu + meshtimesync)

### 2. Kodas
- [ ] Branch `lr2021-rx-order-fix` paruoštas PR į `master`
- [ ] CI visi 5 build'ai žali: companion, companion-2g4, companion-tdm, repeater, repeater-tdm
- [ ] Sandbox 21/21 PASS
- [ ] Log'ų išvalymas: jokių slaptažodžių, privačių key, ACL duomenų commit'uose
- [ ] `.gitignore` patikrintas — `tools/lr2021_sim/`, `capture_serial.py`, laikini failai

### 3. Dokumentacija
- [ ] `README.md` — kas tai, kaip build'inti, kaip flašinti
- [ ] `LR2021_RADIO_STATUS.md` — atnaujintas
- [ ] `DUALBAND_RESEARCH.md` — atnaujintas (L5 uždaryta, §7 GitHub tyrimas)
- [ ] `LICENSE` — MIT (kaip upstream MeshCore)
- [ ] Atribucija: paminėti MeshCore, RadioLib, Semtech, Seeed Studio

### 4. Licencijos
- [ ] Visi trečių šalių kodai turi licencijos pastabas
- [ ] Zephyr patches — Apache 2.0
- [ ] Monocypher — atskira licencija
- [ ] RadioLib reference — MIT

### 5. Release
- [ ] Sukurti GitHub Release su tag'u (pvz. `v1.0.0-dualband`)
- [ ] Įkelti visus 5 firmware .hex failus į release
- [ ] Parašyti release notes:
  - Pirmas pasaulyje LR2021 Zephyr driveris
  - Pirmas veikiantis MeshCore dual-band (868 MHz + 2.4 GHz) TDM
  - XIAO nRF54L15 + Wio-LR2021
  - 120 km sub-GHz, ~45 km teorinis 2.4 GHz

### 6. Atributai (reikia paminėti)
- [ ] **MeshCore** (meshcore-dev) — bazinis protokolas
- [ ] **RadioLib** (jgromes) — LR2021 šablonas
- [ ] **liquidraver/ZephCore** — Zephyr porto bazė
- [ ] **Semtech** — LR2021 chip, usp_zephyr SDK
- [ ] **Seeed Studio** — XIAO nRF54L15, Wio-LR2021, LR2021 EVK

---

## Funkcijos (ką įtraukti į release notes)

| Funkcija | Statusas |
|---|---|
| LR2021 Zephyr driveris (pirmas pasaulyje) | ✅ |
| Sub-GHz 868 MHz (120 km patikrinta) | ✅ |
| 2.4 GHz ISM band (2450 MHz) | ✅ |
| Dual-band TDM (laiko dalijimas) | ✅ |
| HF švyturiai + kaimyno juostos lentelė | ✅ |
| Mesh tiltas (flood → abi juostos) | ✅ |
| Backward compatible (seni mazgai veikia) | ✅ |
| Repeater + Companion rolės | ✅ |
| BLE pairing (PIN 123456) | ✅ |
| USB CLI (laikrodis, password, stats) | ✅ |
| RSSI pataisymas (0dBi → realus) | ✅ |

## Žinomi apribojimai (įtraukti į release notes)

- 2.4 GHz TX max +12 dBm (chip ribojimas)
- RX duty cycle (sniff mode) — neveikia ant LR2021 (SX126x-only)
- 0dBi app'e — gali rodyti neteisingai per BLE (loguose RSSI teisingas)
- Reikia XIAO nRF54L15 + Wio-LR2021 (arba LR2021 EVK)
- Tik vienas LR2021 vienam mazgui (TDM, ne du atskiri radijai)
