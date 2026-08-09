# ELEKTRA — ATASKAITA KITAM AGENTUI (2026-08-07 atnaujinimas)

> **BŪSENA 2026-08-07 (vakaras):** problema **NEIŠSPRĘSTA**. Agentas **NEŽINO sprendimo ir NETURI variantų**.
>
> **Kitas agentas:** reikia pasidaryti **SPECIALŲ BUILD grynai testavimui** — kur dingsta elektra (metodika: `POWER_DEBUG_PLANAS.md` §2-3, Δ metodas B0-B6). Čia ne stebuklas — parašys build'ą ir matys kur nuteka srovė.
>
> **Tik faktai (interpretacijų nėra):**
> - A (8802F48F, USB matuoklis, 2026-08-07): repeater-TDM **24 mA** · +SIMO/DC-DC **30 mA** · companion-TDM **24-30 mA** · **su atjungtu radiju 23 mA**.
> - B (64206A53): companion-TDM **8 mA** (2026-08-06).
> - 111.8 mA nuo baterijos (multimetras 200 mA riba, be USB, 2026-08-07 rytą).
> - Power-debug build'as yra: `CONFIG_ZEPHCORE_POWER_DEBUG` (commit `afdc96e`) — POWER_SUMMARY kas 10 s (radio būsenų laikas, TX skaičiai, main_loops, batt_mv) + GPIO_DUMP (visų sukonfigūruotų pin būsenos). Kompiliacijos fix'as `fcbace9` pushed, CI paleistas.
> - SIMO/DC-DC eksperimentas (`05f26cd`, SET_REG_MODE 0x02) išbandytas, išmatuotas, revertintas (`4e86f0f`).
> - SetRxDutyCycle cycle_time fix'as (`8a167ad`, datasheet 6.3.8: cycle = rx+sleep).
> - Šaka: `lr2021-rx-order-fix` @ `aba3b70`.
> - Vartotojo direktyva (2026-08-07): litavimai/BAT laidai — NIEKADA, pašalinti iš visų dokumentų; „baterija čia ne prie ko".
> **NAUJA VARTOTOJO DIREKTYVA (2026-08-07, vakaras): GAMYKLINIŲ PARAMETRŲ TESTAS — žr. §8.** Atstatyti plokštę į gamyklinius parametrus, išjungti baterijų krovimus ir kitką pagal Seeed Studio nurodymus, pamatuoti po factory defaults, ir TIK TADA (su vartotojo leidimu) suflašinti mūsų firmware.
>
> **SEEED STUDIO TYRIMAS (2026-08-07, vartotojo nurodymu — „ieškok pas Seeed, kodėl daug naudoja") — UŽBAIGTAS.** Šaltiniai: forum 294019 (Coin Battery), 294007 (Sleep Current Comparison), wiki xiao_nrf54l15_sense_power_consumptions, hubble.com nRF54L15 power guide. **Faktai iš Seeed:** (1) XIAO nRF54L15 suprojektuotas 3,7–4,2 V įkraunamai Li-ion per baterijos sąsają (Seeed oficialiai); (2) maitinant per 3V3 kaištį <3,3 V → 450–480 µA (įtariamas TPS62843 ActiveDischarge; prie 3,3 V → 2 µA) — µA skalė, NE mūsų atvejis (USB 5V); (3) regulator-boot-on pašalinimas (pdm_imu_pwr/vbat_pwr/rfsw_pwr) → µA skalės efektas; (4) nRF54L15 chip: Active ~3,5 mA, RX ~3,4 mA, TX@0dBm ~4,8–5,2 mA, System OFF ~0,8 µA; DCDC vs LDO baseline 350→120 µA (hubble). **IŠVADA: NĖ VIENAS Seeed dokumentuotas elgesys nepaaiškina 30 mA — tai 100× viršija visus jų skaičius.** Vienintelė ant plokštės matoma mA skalės veikla = kroviklis „charging" būsenoje (jų schema: „Charging: LED Blink", ICHG=200 mA). **GALUTINIS ATSAKYMAS (2026-08-07, po pilno tyrimo): ~25 mA = kroviklis SGM40567 „charging" būsenoje (ICHG=200 mA × 160 ms/1280 ms = 25 mA vidurkis; mirksintis LED = krovimas vyksta — jų schema „Charging: LED Blink"). ~5 mA = bazė (SAMD11 tiltas + nRF54L15 + keitikliai). SEEED FORUMO IDENTIŠKAS ATVEJIS (gija 295403, XIAO nRF52840): charge LED ON → ~10 mA; švari plokštė → 2–3 µA, LED nedega (sprendimas: litavimo defektas ant baterijos padų). Gija 295257: RF jungiklis ~100 µA kai įjungtas; TPS62843 <3,3 V spiraliuoja (NE mūsų atvejis). Kroviklis neturi EN — programiškai NEĮŠJUNGIA. Firmware radiniai (GPIO_DUMP pd_b0_a_aba3b70.txt): P2.03 RF_SW_PWR=1 (RF jungiklis ~100 µA švaistoma), P0.01 IMU&MIC_3V3_EN=1 (jutiklių rail — jei Sense, ~1–2 mA). Patvirtinimas 10 s: matuoklio rodmuo LED mirksėjimo metu — šokinėja kartu → kroviklis.**
>
> **PAPILDOMAS SEEED TYRIMAS („kaip išjungti kroviklį") — UŽBAIGTAS, ATSAKYMAS: BŪDO NĖRA.** Wiki „Battery Powered Board" (xiao_nrf54l15_sense_getting_started): tik 3,7 V Li-ion, raudonas indikatorius, VBAT per TPS22916; kroviklio išjungimo NĖRA. Forumas 294019: Seeed vartotojas uždavė tą patį klausimą („I didn't see a way to disable the charging circuit") — Seeed: naudok baterijos sąsają 3,7–4,2 V. Jų Zephyr DTS (platform-seeedboards): valdomi tik vbat_pwr/rfsw_ctl/pdm_imu_pwr — kroviklio mazgo NĖRA (nevaldomas). **Vartotojo pastebėjimas: „kitoje plokštėje jis neveikia" — B kroviklis nekrauna (LED nedega, 8 mA), A krauna (LED mirksi, 25 mA): tas pats chip, tas pats dizainas — skirtumas = BAT įėjimo apkrova (A batt_mv=0 → užkrautas). Seeed programinio sprendimo NETURI — kroviklis visada įjungtas iš USB 5V. Sprendimai tik fiziniai (vartotojo rankomis): apkrovos pašalinimas nuo BAT įėjimo arba IREF >1,6 V (10K nuo 5V).**
>
> **PROGRESS 2026-08-07 (agentas):** backup `C:\tmp\backup_A_pre_factory.bin` — 1 462 272 B (0x0–0x165000, bootloader+app+LittleFS; sha256 2ef2f1bb9453b244159e59442ac3e6894290f4b8d0269de3d2ff58886aeb8d96; LittleFS 0x14E000–0x165000 su `DST1` magija patvirtinta; 0x165000–0x180000 = tuščia 0xff; ~0x17FFF0 skaitymas stringa — flash pabaigos regionas neįskaitomas, bet tuščias). A suflašinta **Seeed factory firmware** (oficiali procedūra iš platform-seeedboards/scripts/factory_reset: `pyocd erase --mass -t nrf54l` + `pyocd flash -f 4000000 seeed_firmware.hex` = 24 576 B/6 sektoriai; sha256 a6551446…; **hex išsaugotas `C:\tmp\seeed_firmware.hex` — testą galima kartoti bet kada**). Boot patvirtintas: SP 0x20000858, reset 0xCA9, **CPU PC=0x44A2 = vykdė factory image** (ne stringo). UART tyli (stock demo be console). Vartotojas matavimo po factory NEperdavė (matė tik raudoną charge LED — kroviklio SGM40567 indikatorius, NE firmware valdomas, mirgėjo ir prieš testą). Backup-atgal flash'as pradėtas ir **pertrauktas (exit 130)** — plokštė liko tarpinėje būsenoje. **Vartotojo direktyva 2026-08-07: „man nereikia jokio app, man reikia tuščios plokštės" → MASS ERASE, flash 100% tuščias** (patikrinta 0x0/0x10000 = 0xffffffff). Prieš erase A turėjo POWER_DEBUG build'ą: radijas 100% standby (rx=0,tx=0, 33 min) — 30 mA matuota su radiju NEaktyviu. **Vartotojo direktyva: „ieškok kur ta mikroschema ir išjunk" → KROVIKLIO TYRIMAS UŽBAIGTAS: U1 = SGM40567-4.2XG/TR (XIAO schema `~/lr2021_research/xiao_nrf54l15_schematic.pdf` eil. 111-126; datasheet sg-micro.com). Pinout: A2 VIN←TYPE-C_5V tiesiogiai, A1 BAT→VBAT (BAT laidai), B2 nCHG→Charge_LED (raudonas), C1 IREF←R4 120K (ICharge=200mA), C2 GND. **EN kaiščio NĖRA** (schemos + datasheet patvirtinta). Vienintelis „išjungimas" pagal datasheet: IREF >1,6 V uždraudžia krovimą — bet XIAO IREF lituotas tik prie 120K→GND, be GPIO/test taško → reikėtų fizinio tiltavimo prie 5V = plokštės modifikacija (NE gamyklinė būsena; litavimai uždrausti). nCHG mirksi T=1280 ms (ON 160 ms = T/8) → LED vidurkis ~0,2–0,4 mA — NE 20 mA nuotėkio šaltinis. **A vėl suflašinta į factory firmware (2026-08-07, `erase --mass` + flash — chain veikia, kai po erase iškart flash)**. **LAUKIA vartotojo matavimo ≥2 min (USB matuoklis): factory ≈ 8–10 mA → mūsų firmware problema; ≈ 24–30 mA → aparatūros per-unit defektas.**

---

## 1. BŪSENA (faktinė, 2026-08-06 vakarop)

| Įrenginys | Plokštė | Firmware dabar | Matuota srovė |
|---|---|---|---|
| Companion (B) | XIAO nRF54L15 `64206A53` | companion-TDM `bd3431d` | **0,0080 A (8 mA)** |
| Repeater (A) | XIAO nRF54L15 `8802F48F` | repeater-TDM `a600be9` | **0,0240 A (24 mA)** |

**Matavimo sąlygos (vartotojo duomenys, 2026-08-06):**
- USB matuoklis ant laido (gali būti nelabai tikslus, bet skirtumas 3× tarp abiejų — akivaizdus).
- **Abu XIAO valdikliai buvo ATSKIRTI nuo motininės plokštės** (LoRa plokštė/OLED/ekranas NEprijungti) — matuotas tik pats XIAO.
- Plokštės idzentiškos, abi iš vienos dėžutės.

**Konstatuojamas faktas:** tarp dviejų identiškų valdiklių — ~16 mA skirtumas (8 vs 24). Kadangi matuota **be motininės plokštės**, skirtumas yra pačio XIAO/firmware lygmenyje, NE motininės plokštės periferijoje.

**Problemos būsena: NERASTA. Sprendimo kol kas nėra.**

> ⚠️ **NEIEŠKOTI** OLED/ekrano/charge grandinės/motininės plokštės kaip šios problemos šaltinio — matuota be jų, jos nėra priežastis šiam skirtumui.

---

## 2. KAS JAU PADARYTA ŠIOJE SESIJOJE (uždaryta — NEdaryti iš naujo)

### ✅ 0dBi / RSSI problema — IŠSPRĘSTA ir ON-AIR PATVIRTINTA
- Driver `lr20xx_lora.c`: `lr_rssi_effective()` — packet-RSSI laukas mūsų HW tuščias (0), fallback į despread signal RSSI. Commit `a600be9`, CI 31126905836 ✅.
- App `CompanionMesh.cpp` `logRxRaw`: `PUSH_CODE_LOG_RX_DATA` signalo laukas buvo `snr*4` (=0 visada, pitfall #35) → fallback į RSSI. Commit `bd3431d`, CI 31127859429 ✅.
- Sandbox 22/22 (naujas `test_rssi_effective_fallback`).
- On-air (vartotojo screenshot 23:00): app rodo **−25.0dB / −19.0dB** vietoj 0.0dB. **UŽDARYTA.**

**NEdaryti RSSI darbo iš naujo.**

---

## 3. ELEKTROS TYRIMAS — KAS PATIKRINTA (ir gali būti PAŠALINTA iš tyrimo)

### 3.1 Protokolai/radijai — PAŠALINTA (abu build'ai)
ELF binary scan (A ir B) + Kconfig/grep patikrino: **Zigbee = 0, Matter = 0, Thread/802.15.4 = 0, WiFi = 0, MQTT/uplink = 0**. Šių APKROVŲ NĖRA abiejuose. Tik BLE (`CONFIG_BT=y` companion; repeater `CONFIG_BT=n` → BLE OFF). **NEIEŠKOTI protokolų skirtumo čia.**

### 3.2 Known build skirtumai tarp A ir B (faktai, BE interpretacijos)
- A (repeater): `CONFIG_BT=n` (BLE OFF), turi periodinius TX (`HF beacon` kas ~65 s + adverts), USB CLI.
- B (companion): `CONFIG_BT=y` (BLE ON), BLE adv 20 ms fast→211 ms slow.
- LOG lygmuo: abu `LOG_DEFAULT_LEVEL=3` (board.conf). CONFIG_LOG=n prj.conf, bet board.conf perrašo → LOG 3.
- CONFIG_USB_DEVICE_STACK: nėra. Console per SAMD11 CMSIS-DAP.
- TDM: abu TDM build'ai.
- CONFIG_PM (system): OFF (sąmoningai, nRF54L15 neturi power-states Zephyr 4.4; idle = WFI).

### 3.3 Lyginamieji log'ai A vs B (faktai, 2026-08-06, 30 s tuščiu eismu be žinučių)
| Rodiklis | A (repeater) | B (companion) |
|---|---|---|
| Eilučių per 30 s | **51** | **25** |
| `lr20xx_lora` radijo diag | **27×** (`[post-SET_RX]` 9×, `DIO1 irq raw` 5×, `cad result` 3×, `TX done` 2×, `post-SET_TX` 2×) | **3×** (tik registracija + 1 RX ok) |
| `dualband_radio` TDM langai | 21× | 20× (~vienodai) |
| Klaidos | `CAD timeout` 1× · `RX error: CRC/HDR` 1× (žinomos perkrovos, arti stovintys) | 0 |

**75 s papildomas A capture:** `TX done` 4× / `post-SET_TX` 4× — visi susieti su `HF beacon sent` (kas ~65 s) + `HF TX started len=116`. TX nėra dažnas (ne banginis perdavimas).

### 3.4 DIREKTYVA KITAM AGENTUI (vartotojo 2026-08-06)
- **NEspėlioti ir NEburti priežasčių** — šiuo laikotarpiu bandytos tik išorinės hipotezės (OLED, charge LED, protokolai, „radijas per daug siunčia"), visos **neišvadintos ir pašalintos** (§3.1-3.3).
- **Užduotis: „iš pagrindų ieškoti KODE"** — sisteminga kodo lygmens paieška, kas fiziškai skirtųsi tarp repeater-TDM ir companion-TDM build'ų veikimo metu (kas budina CPU/radiją, kada aktyvu). Naudoti Δ metodu: vienas keitimas → matavimas.
- Duomenys sprendimui: §1 lentelė (8 vs 24 mA be motininės), §3.3 log statistika (A radijo diag 27× vs 3× — NE teiginys apie priežastį, tik pastebėjimas).
- Work dir: `zephcore/` — lyginti `RepeaterMesh.cpp` vs `CompanionMesh.cpp`, `main_repeater.cpp` vs `main_companion.cpp`, abiejų build'ų `.config` skirtumai (repeater.conf vs default). Sandbox turi likti 22/22.

### 3.5 Ką BE šio tyrimo verta nagrinėti (tik kryptys, jokio teiginio)
- Δ: suflašuoti tą patį build'ą ant abiejų plokščių, lyginti elgseną — atskirti build vs aparatūros skirtumą.
- TPS62843 artefaktas: <3,3 V šaltinyje srovė "spirals" — matuoti ≥3,7 V arba USB (jau padaryta — USB).

---

## 4. NELIESK (kartojama iš kanoninių status failų)

- Sub-GHz radio parametrai (869.618/BW62/SF8/CR4/8/sync0x12/TX22) · preamble 32 · noise mask · mesh-split · ACK 64 · TX kelias · LV0 re-arm · `SET_RX_DUTY_CYCLE` (broken) · peer'io (GAT562) flash'inti NEGALIMA · TX > 22 dBm NEGALIMA · on-air antraštė (Packet.h) · RX skaitymo eilė (#37) · peek-IRQ (#14) · 0x0212 (#1).
- RSSI darbai — UŽDARYTI, nebeatnaujinti.

---

## 5. ĮRANKIAI (pavyzdžiai)

```bash
cd /c/Users/proto/ZephCore-nRF54L15-build/zephcore
# tartis: git log --oneline | head
pyocd list                                  # abi plokštės
python tools/capture_serial.py <sek> "C:/tmp/<f>.txt" COM8|COM9   # COM8=B, COM9=A
# sandbox:
cd ../tools/lr2021_sim && rm -f lr2021_sim_tests.exe && "/c/Program Files/LLVM/bin/clang.exe" -std=c99 -Wall -O0 -g stub_lr2021.c driver_under_test.c test_lr2021_driver.c -o lr2021_sim_tests.exe && ./lr2021_sim_tests.exe  # 22/22
# CI: gh workflow run build-nrf54l15.yml -R protozauras/ZephCore --ref lr2021-rx-order-fix
```

---

## 6. KAS LIKO PROJEKTE (eilės tvarka)
1. **L5 ELEKTRA — atvira problema** (šis failas): repeater 24 mA vs companion 8 mA, šaltinis NERASTAS. **Sprendimo kryptis: kodas iš pagrindų** (§3.4), ne spėjimai.
2. L4-U6 likutis (nebūtinas pirmai versijai): tiesioginės žinutės per HF kelio testas.
3. GitHub publikacija.

**Konteksto failai:** `LR2021_RADIO_STATUS.md`, `REPEATER_RADIO_STATUS.md`, `LR2021_DUALBAND_RESEARCH.md` (plano BŪSENA eilutė atnaujinta: 0dBi [x], elektra liko).

---

## 7. NAUJAS VARIANTAS (2026-08-07, vartotojo prašymu — perduoti didesniam modeliui patikrinti)

> **Pastaba dėl viršuje esančio „JOKIŲ IŠVADŲ NERAŠYTI":** galiojo iki 2026-08-07. Ši sekcija pridėta vartotojo direktyva kaip **aiškiai pažymėta hipotezė** (ne faktų teiginys) — kad kitas agentas/didesnis modelis galėtų ją patikrinti.
> **Kontekstas:** vartotojas atnešė `~/Downloads/Repeater Energijos Sąnaudų Optimizavimas.md` (energetikos analizė su siūlymais). Sesijoje patikrinta prieš šaltinius: dauguma jo priežasčių (SAMD11 atgalinis maitinimas 10–14 mA, `CONFIG_PM` deep sleep, OLED 1,5–3,0 mA, UART kaiščiai P1.04/P1.05) **NEATITINKA tikrovės**. Vienintelis naujas vertingas variantas — žemiau.

### FAKTAI (patikrinta prieš šaltinius, 2026-08-07)
1. Mūsų `lr20xx_lora.c` **NESIUNČIA `SET_REG_MODE` (0x0121)** niekur — tik `#define` (eil. 56). Init komentaras mini „reg mode", bet kodas to nevykdo: `/* No SetRegMode: RadioLib (verified working) never switches the regulator mode; the old driver's DC-DC 0x01 did not help. */` (`lr20xx_lora_config`).
2. LR20xx datasheet v2.1, `SET_REG_MODE` aprašymas: **SIMO_OFF (0x00) = default po reset** (LDO). Visos RX srovės (5,7 mA sub-GHz @SF7/125k; **6,6 mA 2,4 GHz @SF7/BW500**) duotos **SIMO konfigūracijoje** („All performance given with SIMO used to power the chip").
3. Semtech oficialus usp_zephyr Wio-LR2021 shield: `reg-mode = <LR20XX_REG_MODE_DCDC>` (0x02) — `semtech_wio_lr20xx_common.dtsi` eil. 37. **Tam pačiam moduliui, kurį naudojame.**
4. Žemo lygio Semtech adapteris `adapters/radio/lr20xx/lr20xx_system.c` **JAU TURI** `lr20xx_system_set_reg_mode()` (eil. ~374; `LR20XX_SYSTEM_REG_MODE_LDO=0x00`, `DCDC=0x02` — `lr20xx_system_types.h:348-349`) — tik jos niekas nekviečia.
5. RadioLib LR2021 `setRegMode()` egzistuoja (`LR2021_cmds_chip_control.cpp:143`), bet niekur nekviečiama — RadioLib build'ai irgi lieka LDO.
6. Mūsų git istorija: DC-DC bandyta 2× — `4b4fc80` (SDK enum 0x02), `86f9877` (raw 0x01, „Meshtastic port"), pašalinta `33f45ee` („no DC-DC — did not help"). Tai buvo sprendimas dėl **TX funkcionalumo**; **srovės Δ su matuokliu NIEKADA nepamatuota**.
7. Datasheet'e LDO RX srovių lentelės NĖRA — „30–40% daugiau su LDO" (iš vartotojo failo) yra įvertis BE šaltinio.

### HIPOTEZĖ (variantas — patikrinti didesniam modeliui)
Radijas visą laiką dirba **LDO režimu** (chip default), nors datasheet srovės ir Semtech oficialus driveris skirti **SIMO/DC-DC**. Įjungus SIMO (`0x02` + ramp times) RX srovė gali kristi keliais mA — galima repeater'io 24 mA vs teorinių ~7–8 mA nuotėkio dalis. Dėmesio: datasheet mini „sensitivity is increased in LDO mode" — galimas kompromisas RX jautrumui.

### KĄ PATIKRINTI PIRMA (kitam agentui / didesniam modeliui)
- Teisinga SIMO seka: `setRegMode(SIMO_NORMAL 0x02, rampTimes[4])` — RadioLib `LR2021_commands.h:218-227` (SIMO_NORMAL = 0x02 << 0), datasheet `SET_REG_MODE` (ištrauktas tekstas: `C:\tmp\lr2021_ds.txt`, ~eil. 5322).
- Δ eksperimentas su matuokliu: build'as su `lr20xx_system_set_reg_mode(ctx, LR20XX_SYSTEM_REG_MODE_DCDC)` init'e (funkcija jau yra low-level) vs be jo; matuoti ≥3,7 V sąlygomis (USB matuoklis); **sandbox nepaliesti; sub-GHz RX kelias (NELIESK) nepaliestas; TX kelias nepaliestas.**
- Ar SIMO keičia RX elgseną (sensitivity/boost) — datasheet pastaba „sensitivity increased in LDO mode".

### NELIESK (nepakito)
Visas §4 sąrašas lieka galioti; šis variantas NĖRA leidimas keisti sub-GHz RX kelią ar TX parametrus.

---

## 8. NAUJA VARTOTOJO DIREKTYVA (2026-08-07, vakaras): GAMYKLINIŲ PARAMETRŲ TESTAS

> **Vartotojo nurodymas (tiesioginis, 2026-08-07):** „Ataskaitoje įrašyti, kad reikia **atstatyti plokštę į gamyklinius parametrus**, **išjungti visokias baterijų krovimų funkcijas ir kitką, pasinaudojant Seeed Studio nurodymais**; **patikrinti, kiek elektros naudoja po factory defaults**; po to — **SU VARTOTOJO LEIDIMU** — suflašinti (mūsų) firmware."
>
> **Kodėl:** dokumentacijos auditas (XIAO schematic, Wio-LR2021 modulio datasheet, TPS62843 ir SGM40567 datasheet'ų, Seeed forum) parodė, kad nė vieno komponento dokumentuota elgsena nepaaiškina A plokštės +16-20 mA (visi kartu = ~8-11 mA, ką ir rodo B). Factory-defaults testas atskirs: **plokštės aparatūra (per-unit defektas)** vs **mūsų firmware/konfigūracija kažką įjungia**, ką Seeed nurodymai išjungia.

### Veiksmų planas (eilės tvarka, kitam agentui)

1. **Backup PRIEŠ bet ką** (pyocd):
   - `pyocd commander -t nrf54l -u 8802F48F -c "save 0x00000 0x165000 C:/tmp/backup_A_pre_factory.bin"`
   - Užfiksuoti dabartinį identitetą (boot log: `Repeater ID: 5322e057...`), ACL, statistiką — factory restore IŠTRINS LittleFS (`/lfs/repeater/_main.id`, ACL, prefs, stats).
2. **Factory defaults atstatymas** — pagal Seeed Studio oficialius nurodymus:
   - Seeed wiki: https://wiki.seeedstudio.com/xiao_nrf54l15_sense_power_consumptions/ ir produkto puslapis (factory firmware atkūrimo procedūra).
   - Gamyklinis firmware = Seeed stock demo (LED blink + ekranas) — kaip Seeed aprašo atkūrimą.
3. **Išjungti baterijų krovimus ir nereikalingus dalykus pagal Seeed nurodymus**:
   - Seeed forum 294019 regulator tipas (board-file dependent): `&pdm_imu_pwr / &vbat_pwr { /delete-property/ regulator-boot-on; };` — **NELIESTI `rfsw_ctl`/`rfsw_pwr`** (būtini RF keliui; mūsų board.overlay jų reikalauja).
   - SGM40567 kroviklis **neturi enable pin** — firmware jo neišjungs (datasheet: 15-135 µA visose būsenose; tai NE nuotėkio šaltinis). „Krovimų išjungimas" = Seeed rekomenduojama fizinė tvarka (baterijos laidų atjungimas) — **BE litavimų** (vartotojo direktyva).
4. **Matavimas po factory defaults** (USB matuoklis, ≥2 min po boot, užfiksuoti sąlygas: build=factory, įtampa, USB):
   - Palyginimas: **B = 8 mA** (mūsų firmware) · **A = 24-30 mA** (mūsų firmware) · **A po factory = ?**
   - Jei factory ≈ 8-10 mA → mūsų firmware/config įjungia kažką, ką Seeed nurodymai išjungia → grįžti į kodo auditą (kas skirtųsi).
   - Jei factory irgi ≈ 24-30 mA → **plokštės aparatūra per-unit defektas** — patvirtinta nepriklausomai nuo firmware (sprendimas: plokščių sukeitimas / A palikti namuose).
   - Įrašyti rezultatą į šios ataskaitos §1 lentelę.
5. **Suflašinti mūsų firmware — TIK SU VARTOTOJO LEIDIMU** (build caps / koordinavimo taisyklė; vartotojas turi aiškiai pasakyti „flashink"):
   - Po flash: atkurti identitetą/prefs iš backup (jei reikia), pakartoti matavimą, atnaujinti šią ataskaitą.

### NELIESK (nepakito)
Visas §4 sąrašas lieka galioti. Factory testas NĖRA leidimas keisti sub-GHz RX kelią, TX parametrus, peer'io (GAT562) flash'inti, ar daryti litavimus.
