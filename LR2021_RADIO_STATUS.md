# ZephCore XIAO nRF54L15 + Wio-LR2021 — HANDOFF (2026-08-02, sesija 4, vakaras)

**Branch:** `lr2021-rx-order-fix` — **HEAD `bd3431d`** · `origin` = `github.com/protozauras/ZephCore` (vartotojo GitHub; `upstream` = `liquidraver/ZephCore` — bazė: MeshCore 1.16.7 + Zephyr portas, tag `1.16.7-zephcore`) · atsišakojimas nuo `master` @ `433499d` (2026-08-01) · **25 commit'ų**: 10× LR2021 driveris/RX eilė (`1c0cb6f`→`0d389de`) + 4× repeater + **9× dvidažnis/L5**: 1× HF kelias (7b552a1), 2× L1-U2 preset/build parinktis (cbba1d0 + b4b1c51), 1× **L3-U4A driver band-switch (7995e56)**, 1× **L3-U4B TDM scheduler (70ad17a)**, 1× **L4-U5 mesh tiltas (7f9ebf9)**, 1× **L4-U5 on-air fix (b1f1a52)**, 1× **L4-U6 HF švyturys (5c1491b)**, 1× **L5 0dBi RSSI driver fix (a600be9)**, 1× **L5 0dBi Rx Log signal field fix (bd3431d)** · `git describe` = `1.16.7-zephcore-64-gbd3431d`
**CI (paskutinis):** run **31127859429** @ `bd3431d` — ✅ PASS (companion + companion-2g4 + companion-tdm + repeater + repeater-tdm); UICI: **31126905836** @ `a600be9` PASS (L5 driver RSSI); anksčiau run **30937299359** @ `5c1491b` (L4-U6), **30925474266** @ `b1f1a52`, **30771498746** @ `7f9ebf9` (L4-U5 SW), **30769715472** @ `70ad17a` (L3-U4B), **30769104440** @ `7995e56` (L3-U4A)
**🗓️ DVIDAŽNIS (2026-08-02):** planas `~/Downloads/LR2021_DUALBAND_RESEARCH.md`. **L1-U1** (`7b552a1`): driveris HF 2.4 GHz kelias — CI ✅. **L1-U2** (`cbba1d0` + `b4b1c51`): Kconfig `ZEPHCORE_BAND_2G4` + 2.4 GHz preset (2450/BW500/SF8/CR4/5/@12dBm) — CI ✅, 2G4 artefaktas `xiao_nrf54l15-companion-2g4`, binary scan patvirtino preset'ą (žr. planą; **spąstas: flag'as turi būti `-DCONFIG_ZEPHCORE_BAND_2G4=y`**). **L2-U3 ✅ PADARYTA (bench, su vartotoju):** abi plokštės suflašintos `firmware/zephyr-2g4-b4b1c51.hex` (#1 `64206A53` ant maitinimo, #2 `8802F48F` prie COM9), ping-pong 2450 MHz **≥8/10 pasiektas** (10 žinučių + retry'ai, RSSI ≈ -44…-47 dBm), LDRO nepasireiškė, XTAL veikia — TCXO nereikalingas. Atstumo testas 2.4 GHz lieka lauko testui. **L3-U4A (driveris) 🟢 COMMITTED `7995e56` + CI ✅ `30769104440`** — `lr20xx_switch_band()` liesas band switch be 50 ms FE cal + `lr_cal_fe_single_bin_hz()` + `lr_band_switch_needs_cal()` (Δf≥20 MHz, RadioLib 1:1) — sandbox 18/18. virtual void self() {}**L3-U4B (scheduler TDM) 🟢 COMMITTED `70ad17a` + CI ✅ `30769715472`** — `DualBandRadio` + `dualband_tdm.h`, Kconfig `ZEPHCORE_RADIO_TDM` (default n), naujas CI artefaktas `xiao_nrf54l15-companion-tdm`, sandbox 19/19, binary scan patvirtino scheduler'į tik TDM artefakte. **L4-U5 (mesh tiltas) 🟢 COMMITTED `7f9ebf9` + on-air fix `b1f1a52` + CI ✅ `30771498746`/`30925474266`** (SW + **orlaivio HF TX PATIKRINTA 2026-08-03**) — `dualband_route.h` (pure `db_tx_band_mask`), `RxPacket.rx_band` + `getLastRxBand()`, `setTxBand()`/forced-HF + `onTxComplete()`, juostos-sąmoningas isRadioReady + in-window HF TX + HF kopijos stash, `checkRecv` žymėjimas + `checkSend` band rezoliucija, `RepeaterMesh` `NeighbourInfo.band` + `getTxBandMask` override. Sandbox 20/20, binary scan TIK TDM artefakte. **⚡ ON-AIR SPĄSTAS + FIX `b1f1a52`:** `7f9ebf9` HF TX neėjo — base `startSendRaw` virtualus `isRadioReady()` lango metu reikalauja `mask == DB_BAND_HF`, stash buvo `DB_BAND_BOTH` → tylus false; fix: `startHfTx` laikinai `setTxBand(DB_BAND_HF)`. **On-air įrodymas (2026-08-03):** plokštė #2 + TDM build, 6 žinutės → **11× `TDM: HF TX started len=22`**, kito galiuko pristatymas veikia (4,6,2; „fail" app'e = ACK round-trip, L2 pitfall). **L4-U6 (švyturiai) ✅ SW+CI COMMITTED `5c1491b` + CI `30937299359` — sandbox 21/21, binary scan patvirtino (švyturio eilutės tik repeater-tdm artefakte). On-air liko — 2× repeater-TDM plokštės, reikia vartotojo. Kitas = L5 (elektra) arba L4-U6 on-air.**
**Įrenginiai (2026-08-06, po L4-U6 on-air — ✅ UŽDARYTAS):** **#2 `8802F48F` (T1, aukštuminė stotis) — repeater-TDM `firmware/zephyr-repeater-tdm-5c1491b.hex`** (švyturys kas ~65 s per sub-GHz; `boot_log_tdm_beacon_test.txt`). **#1 `64206A53` (namų mazgas) — repeater-TDM tas pats hex** — priėmė švyturį → `HF neighbour via beacon` 3×, T1 užregistruotas HF kaimynu (`boot_log_beacon_rx_test.txt`). Abi plokštės dabar baterijomis/logika veikia kaip dvidažnės stotys; **pastaba: abi turi tą patį `/lfs/repeater/_main.id` identitetą** (realiame tinkle vienam mazgui reikia `erase` + naujo identity). Repeater f88692c hex: `firmware-repeater/zephyr-f88692c.hex`; TDM companion (L4-U5): `firmware/zephyr-tdm-b1f1a52.hex`. **🟢 SAVITIKRA (2026-08-06 vakarop, naujas agentas, be vartotojo):** sandbox 21/21 PASS; abi plokštės gyvos, repeater↔repeater pakartotas — `HF beacon sent` 2×/2×, `HF neighbour via beacon` 2× (B pusė), TDM ciklai 16/16 (B) ir 39/39 (A) langai nesustoję, `HF TX started` abiejose (švyturio HF kopijos). Žinomos perkrovos klaidos A pusėje: `IRQ hardware ERROR: 0x00070170` ×1 + **naujas kodas `0x00030320` ×1** (HDR klaida, sig -21 dBm, po jos RX atsistato pats — `RX ok` per 1,2 s, ne blokeris; dokumentuota žr. `boot_log_selfcheck_ab/ba_20260806.txt`). **🟢 PILNA GRANDINĖ ON-AIR UŽDARYTA (2026-08-06, su vartotoju):** B `64206A53` perflašinta į **companion-TDM `firmware/zephyr-tdm-5c1491b.hex`** (vartotojo leidimas; A lieka repeater). Vartotojo FLOOD iš telefono: B siuntė per HF langą (`HF TX started len=111` ×9), A gavo (RX ok 22×) + persiuntė (len=112); app rodė flood aidą `1 hop [53]` ir A švyturius `Type 0x05`. **Naujas B identitetas `933bb63b…`** (perflašinimas į companion išsprendė „abu tuo pačiu ID"). Logai `C:/tmp/live_com8.txt`/`live_com9.txt`.
 **Testai:** 19/20 žinučių (95%) · driver 22/22 paketai švariai · **🏆 LAUKO TESTAS: SIGNALAS NUKELIAVO 120+ km PER TARPINIUS (2026-08-02) — PROJEKTAS PAVYKO**
**Radio būsena:** ✅ **VEIKIA** — RX/TX patvirtinta (lauko testas: 120+ km per tarpinius), bet **KOL KAS TIK VIENU DAŽNIU** — fiksuota `869.618 MHz`. Kelių dažnių / dažnio perjungimo **NĖRA** — vienas kanalas, keičiamas tik per build/config.
 **Dabartinis prioritetas:** ~~⚡ **ELEKTRA** — vartotojo matavimai (31,5 mA) **teoriniai, jais pasikliauti negalima** (2026-08-06 komisija); agento darbas: peržiūrėti kas kur elektrą ima + **nuotėkio paieška** (§4.1 metodika, žr. ageneto skiltis plane).~~ **✅ ELEKTRA UŽDARYTA (2026-08-07). AGENTAI: nebetaisyti, neieškoti priežasčių. A plokštė (8802F48F) = SGM40567 kroviklis pažeistas (~30 mA), naudojama kaip companion ant USB. B plokštė (64206A53) = sveika (~17 mA).** **✅ 0dBi app problema IŠSPRĘSTA (2026-08-06, commit `a600be9` + `bd3431d`, on-air patvirtinta vartotojo screenshot):** app Rx Log dabar rodo realų RSSI (pvz. −25.0dB / −19.0dB) vietoj 0.0dB. Dvi priežastys: (1) driver `lr20xx_lora.c` — `rssi_pkt` laukas mūsų HW tuščias (0), o senoji sąlyga `snr<0 && rssi_signal>rssi` niekad nevykdo (snr=0 + rssi_signal>0 false) → nauja `lr_rssi_effective()` grąžina despread `rssi_signal` kai paketo laukas 0; (2) app PUSH_CODE_LOG_RX_DATA signalo laukas buvo `snr*4` (=0 visada, pitfall #35) → `logRxRaw` dabar fallback į RSSI. Sandbox 22/22 (naujas `test_rssi_effective_fallback`), CI ✅×2 (31126905836, 31127859429). [ ] Repeaterio pusėje (A) RSSI driver fix jau yra; atstumo skirtumai (−25 aiškus, −19 per hop) matomi app'e.

**Skill:** `zephcore-firmware` — pitfall #36 (kopijuoti upstream), #21 (sandbox veidrodis), #32 (logus skaityti pirma), #7 (build caps — HARD STOP be vartotojo leidimo), #34 (0x0212 layout).

---

## 1. BŪSENA — KAS PADARYTA IR KAIP (0d389de, „MeshCore kepurė")

**Metodika (GitHub žmonėms):** mūsų driverio RX/TX kelią palyginti 1:1 su veikiančiu upstream ir PERKOPIJUOTI jų seką (Zephyr vietoj Arduino). Referencai (TIK jie):
1. `jgromes/RadioLib` master — `src/modules/LR2021/LR2021.cpp` (`stageMode` RX eil. 953-1022, `readData` eil. 566-616), `LR2021_cmds_chip_control.cpp`.
2. `meshcore-dev/MeshCore` main @ `03b6ef4` — vietinis klonas `.hermes/meshcore-upstream/`; `src/helpers/radiolib/RadioLibWrappers.cpp`, `src/Dispatcher.cpp`.
3. **MeshCore PR #2739** (c03rad0r, NiceRF LoRa2021) — vienintelė žinoma MeshCore LR2021 adaptacija; jie RX sekos nelietė (tik board variantas) = įrodymas, kad stock seka veikia.
4. `TheClams/lr2021` (Rust) — nepriklausomas 0x0212 layout patvirtinimas.
5. Zephyr oficialiai LR2021 driverio NĖRA — mūsų pirmas pasaulyje.

**Fix'ai (0d389de):**
| # | Buvo | Upstream | Fix |
|---|---|---|---|
| A | `start_rx` kvietė `apply_modem_config` — FE kalibracija + 50 ms sleep + dažnio perrašymas KIEKVIENĄ kartą (TX→RX = 55 ms kurčia skylė) | `stageMode(RX)` tik: `setRxPath` → `setDioIrqConfig` → `clearIrqState` → `setLoRaPacketParams(255)` → `setRx`; kalibracija tik `doResetAGC` | Pilnas config → `lora_config()` (kartą) + `lr20xx_reset_agc()` (AGC intervalas). **Re-arm 55 → 15 ms; bundle'ai nebesiformuoja** |
| B | `send_async` laikė SPI mutex per visą TX (poll iki 6 s) | `startTransmit()` grąžina iškart | Mutex atleistas po `SetTx`; DIO handler TX_DONE (edge) + poll fallback; be dvigubo re-arm/raise |
| C | (ankstesnė) viso likusio FIFO skaitymas | vienas paketas per RX_DONE | Su A pasiektas prompt service — bundle'ų nebėra; mesh-split lieka fallback'u |

---

## 2. TESTŲ REZULTATAI

| Testas | Rezultatas |
|---|---|
| Sandbox `tools/lr2021_sim` | **12/12 PASS** |
| CI build (0d389de) | ✅ PASS |
| Boot: TX→RX re-arm | **15 ms** (buvo ~55 ms) |
| Boot: RX vieno paketo | 8/8 + 22/22 `st_len=22`, 0 bundle ilgių, 0 klaidų |
| Testas 1 (abipusis) | 9/10 — RF kolizijos (abu siuntė vienu metu) |
| Testas 2 (vienpusis, kas 3 s) | **19/20** — driver 22/22 švariai; trūko tik RF kolizijoje |
| **LAUKO TESTAS** | **🏆 120+ km per tarpinius** — signalas nukeliavo, projektas pasisekė |
| Įrodymai | `boot_log_mchat_0d389de.txt`, `boot_log_mchat_test2.txt` |

Kosmetika: „0.0dB" = chip grąžina snr=0 (pitfall #35); žinučių eilės tvarka = normali (retry'ai).

---

## 3. NELIESK (PATVIRTINTA VEIKIA / NEBANDYTI)

- Noise mask · mesh-split fallback (05806cf) · ACK lentelė 64 (a1dc499) · TX kelias (100%) · radio parametrai (freq **869.618 MHz** / BW **62** / SF **8** / CR **4/8** / sync **0x12** / TX **22 dBm**) · preamble 32.
- **NEGALIMA** flash'inti peer'io (GAT562). **NEGALIMA** TX power > 22 dBm.
- **NEGALIMA** „peek IRQ" keitimų (pitfall #14) · RX skaitymo eilės keitimo (pitfall #37) · aklai kopijuoti RadioLib 16-bit 0x0212 parse (spąstas §5.1).
- Ekranas: **neveikia ir NEREIKIA** — jokių ekrano/UI darbų (OLED I2C išjungtas `CONFIG_I2C=n`).

---

## 4. TOLIMESNI DARBAI (prioritetų tvarka)

### 4.1 ⚡ ELEKTRA — PRIORITETAS #1

**⚠️ 2026-08-07 — NAUJAS MATAVIMAS + PLANAS:** vartotojas išmatavo multimetru (200 mA riba): repeater nuo baterijos (be USB) = **111.8 mA**. Per USB be carrier = 24 mA, su carrier = 31.5 mA. Companion = 8 mA. **PROBLEMA NERASTA.** Sukurtas sistemingo tyrimo planas: **`POWER_DEBUG_PLANAS.md`** (repo šaknis) — kitas agentas turi pasidaryti `CONFIG_ZEPHCORE_POWER_DEBUG` build'ą su radio būsenų timing'u + periodine power suvestine, ir Δ metodu (po vieną keitimą) rasti kur dingsta elektra. Pilna istorija: `ELEKTRA_NEXT_AGENT_ATASKAITA.md`.

**Kur dingsta — DIAGNOZĖ (2026-08-03, interneto tyrimas; skaičiai PATIKRINTI su šaltiniais):**

| Šaltinis | ~mA | Įrodymas / šaltinis | Ką daryti |
|---|---|---|---|
| **XIAO maitinimas <3.3V (TPS62843 buck)** | **+5-15 (artefaktas!)** | Seeed forum `295257` („wrong design for low power") + `294019` + Nordic DevZone `128109` (verified): žemiau ~3.3V srovė „spirals out of control" (buck near-dropout / ActiveDischarge); 3.0V → 450-480 µA net system off; ≥3.3V → 2 µA. Rekomendacija: 3.7-4.2V (LiPo) arba USB | **PIRMA MATAVIMO SĄLYGOS:** jei 31,5 mA matuota <3.3V šaltinyje — dalinai artefaktas. Matuoti tik LiPo 3.7-4.2V arba USB 5V |
| **LR2021 cont. RX** | **5.7** (ne 4.6) | Semtech LR2021 datasheet v1.1 (Mouser PDF): „Rx current 5.7mA @sub-GHz"; **sleep 470 nA**. Mūsų `rx-boosted` + TCXO + RF switch gali pridėti ~1-2 | Repeater: inherent, neliesk. **Companion: radio sleep** — kopijuoti RadioLib LR11x0 `sleep(retainConfig)` (MeshCore `LR11x0Reset.h` jau taip daro: `sleep(true)` po TX) + periodinis wake → RX langas ≥70 ms (preamble 32 @ SF8/125k = 65,5 ms) → vėl sleep ⇒ vidurkis <0.5 mA. `SET_RX_DUTY_CYCLE` broken — nenaudoti |
| **nRF54L15 idle (CONFIG_PM)** | **~0.5-2** | Hubble guide (nRF54L15 DK, NCS 2.7): Active 64MHz ~3.5 mA · System ON Idle ~30 µA · Sleep ~1.8 µA · OFF ~0.3 µA. **Mūsų Zephyr `722a6f9` (2026-07-16): `nrf54l15.dtsi` NETURI power-states (patikrinta, grep=0)** → `CONFIG_PM=y` vienas nieko neduotų; reikėtų port'inti states iš NCS + wake kelius | Kol kas NEDARYTI — radio (5.7) dominuoja. Grįžti tik po radio sleep, jei MCU taps didžiausias vartotojas |
| **SAMD11 CMSIS-DAP** | **~0.1-0.5** (ne 5-10!) | Seeed forum `294019`: be USB, ≥3.3V, system off → **2 µA visa plokštė**; 450 µA buvo 3.0V TPS62843 artefaktas, NE SAMD11 | Matuoti USB atjungtas vs prijungtas (delta). Software neišjungiamas — maža hardware tax |
| **BLE advertising (companion)** | **~1-4** (fast adv), ~0.3-1 (connected 48 ms) | Hubble: 1 s adv @0 dBm ~42 µA avg; 30 ms conn interval ~300 µA avg. Mūsų: 20 ms fast 60 s + 8 dBm | Matuoti ≥2 min po boot (po fast adv lango). Po 60 s jau lėtas intervalas |
| **UART console + LOG** | **~0.1-0.5** | Hubble: logging off = didžiausias vienas šuolis (350→180 µA DK) | Produkcija: `LOG_DEFAULT_LEVEL` 3→1 (repeater CLI lieka) |
| **RF switch regulators** | **~0.1-1** | Seeed forum `294019`: „100uA wasted just to use the radio" (RF switch klasė) | Būtini RF keliui — neliesti |
| **LED (P2.00)** | **~0** | Zephyr board def: `led0` alias; repeater main OFF boot'e (513-522); companion neliečia → off. **Mirgantis LED = ne mūsų firmware (Seeed stock/blink) — patikrinti koks build'as suflašintas** | LED užduotis žemiau (laukianti leidimo) |
| OLED (SSD1306) | ~0 | I2C=n — išjungtas. Jei fiziškai prijungtas ant LoRa Plus — patikrinti ar netraukia | — |

**MeshCore / GitHub palyginimas (tikslai):** RAK4631 repeater **12.7 mA** (GPS off — MeshCore FB grupė); EasySkyMesh `PowerSaving10` (github.com/IoTThinks/EasySkyMesh): Heltec V3 **~9.1 mA**, RAK4631 **~8.4 mA**, +sensoriai ~10 mA (nodakmesh.org blogas). **Mūsų grindys (repeater):** LR2021 5.7 + rfsw ~0.5 + MCU ~1 + SAMD11 ~0.5 ≈ **7-8 mA** — pasiekiama BE radio sleep. Dabar 31,5 → pirma patikrinti matavimo sąlygas (TPS62843 <3.3V + BLE fast adv + USB), tada etapais žemyn.

**Matavimo metodika (etapai — vienas keitimas vienu metu; Δ = to komponento srovė; rezultatus įrašyti čia):**
0. **Sąlygos:** šaltinis ≥3.7V arba USB; matuoti ≥2 min po boot; užfiksuoti build (companion/repeater), įtampą, USB taip/ne, BLE būseną (adv/connected).
1. Radijas STBY vs RX (per repeater CLI / laikinas build) → tikimasi ~5.7 mA Δ.
2. BLE off vs on — tas pats įrenginys, repeater build (be BLE) vs companion → BLE Δ.
3. `CONFIG_LOG_DEFAULT_LEVEL` 3→1 → ~0.1-0.5 Δ.
4. USB atjungtas (baterija) → SAMD11 Δ.
5. (Vėliau, atskira užduotis) radio sleep prototipas companion → 5.7 → <0.5.

**Sėkmės kriterijus (elektra):** repeater ≤10 mA (benchmark 8.4-9.1); companion su radio sleep ≤2-3 mA vidurkio; kiekvieno komponento srovė žinoma ir užrašyta.

**LED užduotis (vartotojo direktyva):** diodus išjungti; **trumpas blink įjungus repeater'į, po poros minučių visai užgesta**. Dabar: OFF iškart (main_repeater.cpp:512-522). Implementacija: boot'e 3× blink (50 ms ON/200 ms OFF), tada OFF visam laikui (arba timer 2 min). Led0 = P2.00 ACTIVE_LOW. **NEdaryti be vartotojo leidimo — build caps (pitfall #7); vartotojas dabar matuoja srovę — firmware keitimas sugadintų matavimus.**

### 4.2 🔁 REPEATER — ⚠️ VISO REPEATER KONTEKSTO PERKELTA Į **`REPEATER_RADIO_STATUS.md`**

Repeater darbai (LED, stats.daily, laikrodžio/login problemos, USB setup checklist, LED inventorius 3 vnt., slaptažodžiai) — žiūrėk **`REPEATER_RADIO_STATUS.md`** (atskiras failas, nes per daug skiriasi nuo companion). Čia lieka tik nuoroda.

### 4.3 🍴 GitHub fork / publikacija
Pirmas LR2021 Zephyr driveris pasaulyje. Fork'as su driveriu kaip švariu moduliu (DTS + driveris + Kconfig, be ZephCore app) + dokumentacija „kaip padaryta" (šis failas = pagrindas, §1 + §5 spąstai). Gatavas produktas GitHub žmonėms.

### 4.4 📡 Lauko testas — ✅ ATLIKTAS (2026-08-02)
**Signalas nukeliavo 120+ km per tarpinius** — projektas pasisekė. Detalės (taškai, SNR) — vartotojo rankose, pridėti jei pateiks.

### 4.5 Ekranas
Neveikia ir NEREIKIA — nesiimti jokių ekrano/UI darbų.

---

## 5. SPĄSTAI (išmokta — GitHub žmonėms)

1. **0x0212 (GetRxPktLength) Wio-LR2021:** `[stat16][0x14 STATUSO baitas][len]` — resp[2] = **0x14, NE len_hi**; resp[3] = ilgis. RadioLib/TheClams skaito `data0<<8|data1` — jų aparatūroje veikia, mūsų modulyje duotų 0x14xx. **Skaitome `& 0xFF`.** Patikrinta LIVE.
2. **One-transaction-behind:** pirma statuso komanda po RX_DONE = IRQ echo; tvarka: GetRxPktLength (echo) → GetLoRaPacketStatus (st_len) → GetRxPktLength (realus likutis).
3. **DIO edge TX_DONE prarandamas** — poll fallback veikia (≤2 ms); netaisyti be reikalo.
4. **Dvigubas `[post-SET_RX]` po TX** — nekenkia, sąmoningai nefixuota.
5. **`setLoRaPacketParams(pld_len=255)`** prieš KIEKVIENĄ SetRx (RadioLib issue #1804).
6. **Pašalinė mesh platforma UŽDRAUSTA** kaip šaltinis — ignoruoti istorinių commit pavadinimų formuluotes.

---

## 6. ĮRANKIAI

```bash
# Upstream klonas (jau yra):
.hermes/meshcore-upstream/            # meshcore-dev/MeshCore main, 03b6ef4
# RadioLib: GitHub jgromes/RadioLib master (src/modules/LR2021/ + LR11x0/)
# PR2739 (MeshCore LR2021): github.com/meshcore-dev/MeshCore/pull/2739
# Repeater info: upstream examples/simple_repeater/ + docs/faq.md §3

# Sandbox (12 testų):
cd tools/lr2021_sim && rm -f lr2021_sim_tests.exe && \
"/c/Program Files/LLVM/bin/clang.exe" -std=c99 -Wall -O0 -g stub_lr2021.c driver_under_test.c test_lr2021_driver.c -o lr2021_sim_tests.exe && ./lr2021_sim_tests.exe   # 12/12

# CI + flash (companion ARBA repeater artifact):
gh workflow run build-nrf54l15.yml -R protozauras/ZephCore --ref lr2021-rx-order-fix
gh run watch <ID> -R protozauras/ZephCore --exit-status --interval 30
gh run download <ID> -R protozauras/ZephCore -n xiao_nrf54l15-companion -D /tmp/dl   # arba -n xiao_nrf54l15-repeater
cp /tmp/dl/firmware/zephyr.hex firmware/zephyr.hex     # firmware/zephyr.hex = 0d389de companion
pyocd flash -t nrf54l -u 8802F48F firmware/zephyr.hex
pyocd commander -t nrf54l -u 8802F48F -c reset
python tools/capture_serial.py 300 boot_log_<tag>.txt  # failas rašomas TIK pabaigoje!
```

**Sėkmės kriterijus (radio):** 10 žinučių kas ~3 s iš vienos pusės → ≥8 pristatytos; driver: kiekvienas RX ok = vienas paketas.
**Sėkmės kriterijus (elektra):** žinoti kiekvieno komponento srovę; tikslas — sumažinti bendrą (31,5 mA dabar), ypač per CONFIG_PM + logging + LED.
