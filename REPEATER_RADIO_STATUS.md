# ZephCore REPEATER — HANDOFF (XIAO nRF54L15 + Wio-LR2021, kartotuvas)

**Atskiras failas nuo `LR2021_RADIO_STATUS.md` (tas = companion/bendras). Visas repeater kontekstas — ČIA.**
**Branch:** `lr2021-rx-order-fix` — **HEAD `bd3431d`** (ant `a600be9`; a600be9 = L5 0dBi driver RSSI fix, bd3431d = L5 0dBi Rx Log signal field; 5c1491b = L4-U6 HF švyturys; b1f1a52 = L4-U5 on-air HF TX fix; 7f9ebf9 = L4-U5 mesh tiltas; 70ad17a = L3-U4B TDM scheduler; 7995e56 = L3-U4A driver band-switch) · `git describe` = `1.16.7-zephcore-64-gbd3431d` (21 commit'ų; dvidažnis: L1-U1+U2 CI ✅, **L2-U3 2.4 GHz radio įrodymas ✅ PADARYTA bench su vartotoju 2026-08-02**, **L3-U4A driveris ✅ COMMITTED `7995e56` + CI `30769104440`**, **L3-U4B TDM scheduler ✅ COMMITTED `70ad17a` + CI `30769715472`**, **L4-U5 mesh tiltas ✅ COMMITTED `7f9ebf9` + on-air HF TX patikrinta `b1f1a52` (CI `30925474266`) 2026-08-03** + **L4-U6 HF švyturys ✅ COMMITTED `5c1491b` + CI `30937299359` (SW+CI)** + **L5 0dBi ✅ + L5 ELEKTRA UŽDARYTA (2026-08-07)**; planas `~/Downloads/LR2021_DUALBAND_RESEARCH.md`, vykdyti BAIGIMO PROTOKOLĄ)

**ELEKTRA (2026-08-07 — UŽDARYTA):** A plokštė (8802F48F) turėjo SGM40567 kroviklio defektą (~30 mA). CTRL-AP ERASEALL + 3 skirtingi firmware (factory, ZephCore) nepadėjo → aparatinis. Naudojama kaip companion ant USB. B plokštė (64206A53) sveika, repeater-TDM ~17 mA. **AGENTAI: elektros problema išspręsta — nebetaisyti, neieškoti priežasčių, neflašinti A plokštės "pataisymams".**
**Branch tiksliai:** `origin` = **`github.com/protozauras/ZephCore`** (vartotojo GitHub; `upstream` = `liquidraver/ZephCore` — bazė: MeshCore 1.16.7 + Zephyr portas, tag `1.16.7-zephcore`) · atsišakojimas nuo `master` @ `433499d` (2026-08-01) + **16 commit'ų** virš master: **10× LR2021 Zephyr driveris / RX eilė** (`1c0cb6f`→`0d389de`: RX ilgio skaitymo tvarka, bundle'ai, non-blocking TX, „MeshCore kepurė" re-arm), **4× repeater** (`8bec55e` LED+stats, `c2ba84c` 64-bit, `95e9801` heartbeat stop, `f88692c` **login root-cause fix**) ir **2× dvidažnio L3-U4** (`7995e56` driver-switch, `70ad17a` TDM scheduler)
**CI:** run **31127859429** @ `bd3431d` — ✅ **PASS** (companion + companion-2g4 + companion-tdm + repeater + repeater-tdm; L5 0dBi fix); anksčiau run **30937299359** @ `5c1491b` — **PASS** (L4-U6 švyturiai)
**Įrenginiai (2026-08-06, po L4-U6 on-air — ✅ UŽDARYTAS):** **#2 `8802F48F` (T1, aukštuminė stotis) — repeater-TDM** (`firmware/zephyr-repeater-tdm-5c1491b.hex`), švyturys kas ~65 s. **#1 `64206A53` (namų mazgas) — repeater-TDM tas pats hex** — priėmė švyturį → `HF neighbour via beacon` 3× (`boot_log_beacon_rx_test.txt`). **Pastaba: abi plokštės turi tą patį `/lfs/repeater/_main.id` identitetą** — realiame tinkle vienam mazgui reikia `erase` + naujo identity. Buvęs f88692c repeater build'as: `firmware-repeater/zephyr-f88692c.hex`. **🟢 SAVITIKRA (2026-08-06 vakarop, naujas agentas, be vartotojo):** sandbox 21/21; abi gyvos — `HF beacon sent` 2×/2×, `HF neighbour via beacon` 2× (B), TDM 16/16 (B) ir 39/39 (A) langai, `HF TX started` abiejose. A pusėje žinomos perkrovos klaidos (`0x00070170` ×1 + naujas `0x00030320` HDR — po jos RX atsistato, ne blokeris; `boot_log_selfcheck_ab/ba_20260806.txt`).
**Skill:** `zephcore-firmware` — pitfall #36 (kopijuoti upstream), #21 (sandbox), #7 (build caps — HARD STOP be leidimo).
**Radio būsena:** ✅ **VEIKIA** — RX/TX patvirtinta (lauko testas 2026-08-02: signalas 120+ km per tarpinius), bet **KOL KAS TIK VIENU DAŽNIU** — fiksuota `869.618 MHz` (BW62/SF8/CR4/8, TX 22 dBm). Kelių dažnių / dažnio perjungimo (hopping) **NĖRA** — vienas kanalas, keičiamas tik per build/config.

**⚠️ BŪSENA (2026-08-02 vakaras, po RF testo):** **ADMIN PANELĖS PER LORA NEPAVYKO SUTVARKYTI.** f88692c suflašintas, setup atliktas, boot švarus — bet vartotojo admin login per telefoną vis tiek NEVEIKIA („nejungia admin"). Problema sutampa su upstream #983 (atvira 8 mėn., be sprendimo) + #2955 (atvira). Žr. §7 ir §8.

---

## 🚀 NEXT AGENT — ACTION PLAN (daryti eilės tvarka, be vartotojo)

**Login fix (`f88692c`) yra PADARYTAS ir verifikuotas (harness 11/11 + CI statosi). Vartotojas išėjo — liko SUFLASHINTI IR PATIKRINTI, kad veikia.**

1. **Patikrinti CI**: `gh run watch 30726933415 -R protozauras/ZephCore --exit-status --interval 30` (jei FAIL — žiūrėti logus; pataisyti ir paleisti iš naujo).
2. **Atsisiųsti + suflašinti repeater f88692c**:
   ```bash
   gh run download 30726933415 -R protozauras/ZephCore -n xiao_nrf54l15-repeater -D "C:/tmp/dl"
   cp "C:/tmp/dl/zephyr.hex" firmware-repeater/zephyr-f88692c.hex
   pyocd flash -t nrf54l -u 8802F48F firmware-repeater/zephyr-f88692c.hex
   pyocd commander -t nrf54l -u 8802F48F -c reset
   ```
3. **Boot patikra** (parallel: `python tools/capture_serial.py 40 boot_log_f88692c.txt` + reset): "Daily stats: N/90", radio `freq=869.62 ... pwr=22`, jokių klaidų.
4. **Setup (jei prefs nepergyveno / dėl saugumo perklausyti)**:
   ```bash
   # per serialą: time <date +%s> · set meshtimesync on · password zephcore123 ·
   # get prv.key → backup · set agc.reset.interval 4 · set tx 22 · clock · get meshtimesync
   ```
5. **⏳ PAGRINDINIS TESTAS — vartotojui grįžus**: paprašyti per telefoną: admin login `zephcore123` (JEI APP TURI SENĄ SESIJĄ — ištrinti/įdėti kontaktą iš naujo: pubkey `5322E0578AB9375460E769CA30F9F929D318BA99F46ACA065600C67867CF727F`, tipas Repeater). **Pirma TIK admin, ne guest.**
6. **Acceptance (root-cause fix įrodymas)**: po sėkmingo admin login'o — `get acl` turi rodyti admin klientą; tada **perkrauti kartotuvą (power cycle)** ir be jokio app veiksmo vėl bandyti admin komandas — **sesija turi išgyventi reboot** (ACL dabar persistuojamas iškart). Jei išgyvena — fix'as veikia, uždaryti problemą.
7. **Jei login vis dar neveikia**: live capture (`python tools/capture_serial.py 150 login_test3.txt`) kol vartotojas bando; ieškoti: `Login success` / `Invalid password` / `Login rate-limited` / `Possible login replay attack!` / `no peer could decrypt message`. Su rezultatu — analizuoti pagal §0.
8. **NEPAMIRŠTI**: rezultatus įrašyti į šį failą (sekcija "TESTŲ REZULTATAI" žemiau), atnaujinti HEAD/CI eilutes viršuje.

**Likusios užduotys (NE blokuoja login testo):**
- LED #2 (XIAO charge/PWR) + #3 (LoRa Plus Power LED) — žr. §2: gauti schemas, patikrinti ar GPIO; jei ne — fizinis sprendimas. Vartotojo direktyva: "vienu metu viską tvarkyti".
- USB setup prieš išnešimą į lauką — §3 checklist.

---

## 0. DABARTINĖ PROBLEMA — remote admin login NEVEIKĖ ⚠️ → PATAISYTA f88692c, BET RF TESTAS NEPRAĖJO (2026-08-02 vakaras: admin login per LoRa VIS TIEK NEVEIKIA)

**Simptomai (vartotojas, 2026-08-02):** per USB CLI veikia; per LoRa: "password blogas arba nepasiekiamas". Vieną kartą admin prisijungė, paskui nebe. **Guest prisijungia** (guest password `linux1`), bet per guest ne viskas veikia.

**Įrodymas (live capture `login_test2.txt`, 2026-08-02):**
```
RX ok: raw_len=4 st_len=53 ... data=22 00 53 32 ...
onRecvPacket: no peer could decrypt message
```
Kartotuvas GIRDĖJO app'o paketą, bet NEGALĖJO IŠŠIFRUOTI. Per 150 s atėjo tik VIENAS paketas ir jis buvo **direct tipo** (REQ/TXT_MSG — šifruojamas peer shared secret iš ACL), NE šviežias ANON login (kuris šifruojamas ECDH ir ACL nereikia).

**Root cause analizė (kodas patikrintas):**
1. **ACL tuščias** (`get acl` → 0 įrašų; `/lfs/repeater/acl` failo NĖRA — boot'e "file open error (-2)"). Guest įrašai sąmoningai nepersistuojami (upstream elgsena, ClientACL.cpp:123). Admin įrašas su shared secret dingo (niekada nepersistavo ARBA guest login perrašė perms prieš 5 s lazy save → save praleido guest).
2. **App'as laiko seną sesiją**: siunčia direct paketus su senu shared secret (iš to vienintelio sėkmingo login'o), o kartotuvas to rakto nebeturi → "no peer could decrypt" → tyliai numeta → app: "password blogas/nepasiekiamas".
3. **Laikrodis 1970 po perkrovimo** (nėra RTC/GPS; USB atjungimas = maitinimo netektis) — login atsakymai su 1970 laiku app'ui nepriimtini.
4. Žinoma upstream problema: **GitHub #983** ("Cannot remote admin Repeater from companion", atvira) + **#2955** ("admin login fails on direct path; Reset Path padeda"). Sprendimai iš thread'o: Reset Path, "Use Companion Clock for CLI" (app v1.39+ eksperimentinis), companion reboot, repeater reboot.

**SPRENDIMO ŽINGSNIAI (eilės tvarka):**
1. **App'e: IŠTRINTI repeater kontaktą → ĮDĖTI IŠ NAUJO** (priverčia šviežią ANON login — ECDH, ACL nereikia):
   - Pubkey: `5322E0578AB9375460E769CA30F9F929D318BA99F46ACA065600C67867CF727F` (get public.key)
   - Tipas: **Repeater** · Admin password: **`zephcore123`**
   - **SVARBU: pirma bandyti TIK admin. NEloginti guest pirma** — guest login perrašo ACL perms į guest ir shared secret.
2. Jei neveikia: app → 3 taškai → **Reset Path**; app → Settings → Experimental → **"Use Companion Clock for CLI"**; perkrauti companion'ą.
3. Jei vis dar ne: **debug build** (MESH_DEBUG=1 + MESH_PACKET_LOGGING=1) + live capture → matyti ANON kelią (ar ECDH decrypt veikia, ar "Login success"/"Invalid password").
4. Patikrinti laikrodį: `clock` → jei 1970, `time <epoch>` per USB.

**Kodas 1:1 su upstream** (onAnonDataRecv login dispatch identiškas upstream MyMesh.cpp:580).

**✅ PATAISYTA (f88692c, 2026-08-02) — root cause fix, 3 pataisos `RepeaterMesh::handleLoginReq`:**
1. **ACL persistinamas IŠKART po admin login'o** (ne po 5 s lazy save) — sesija (shared secret) išgyvena maitinimo netektį/perkrovimą; app'o direct paketai po reboot vėl iššifruojami.
2. **Guest login NEnuša admin įrašo** — "once admin, stays admin" (role keičiasi tik aukštyn: guest→admin). Guest login'as daugiau nebekriaplina sesijos ("per guest ne viskas veikia" nebegali atsitikti).
3. **Replay check retry-tolerant**: `sender_timestamp < last_timestamp` (buvo `<=`) — app'o teisėtas to paties paketo retry po dingusio atsakymo dabar praeina; tik tikrai senesnis timestamp = replay. Saugu: password jau patikrintas prieš replay check.
- Verifikacija: ad-hoc harness **11/11 PASS** (demotion, replay, retry po lost response) · CI ✅ · **LAUKIA vartotojo RF testo**.
- **Divergencija nuo upstream — sąmoninga, dokumentuota** (upstream: 5 s lazy save, demotion, `<=`). Login protokolas (paketų formatas) nepaliestas.

**Liko (jei po f88692c vis dar neveikia):** app'e ištrinti/įdėti kontaktą iš naujo (sena sesija iš prieš-fix laikų vis tiek negyva), tada debug build jei reikės.

---

## 1. KAS PADARYTA (95e9801)

| Commit | Kas |
|---|---|
| `8bec55e` | **LED boot blink**: led0 mirksi 500 ms ON/OFF **pirmą minutę**, po to OFF visam laikui (`MESH_EVENT_LED_TICK` BIT(8) + `led_blink_timer` K_TIMER_DEFINE + `led_blink_tick`). **`stats.daily` CLI**: dienos skaitikliai (rx_flood, rx_direct, fwd, tx, admin_login, guest_login) → flash žiedas **90 d. × 28 B ≈ 2,6 KB** (`/lfs/repeater/stats_daily.bin` per RepeaterDataStore, tmp+rename), persistinama **kas valandą + dienos pasikeitime**, pilnas žiedas → seniausia diena ištrinama (failas niekada neauga). **Veikia per RF** (jokio serial-only gate — skirtingai nei stats-*). Diena = UTC (laikrodis nustatytas) arba boot-relative. Ataskaita ≤160 B (RF reply buffer 161). |
| `c2ba84c` | **64-bit uptime fix** — `k_uptime_get()` yra int64; uint32 apsiverstų po 49,7 d. ir boot-relative dienų skaičius lūžtų. Pagauta ad-hoc harness testu (21/21 PASS). |
| `95e9801` | **UI LED heartbeat sustabdytas** — `ui_led` modulis (helpers/ui/ui_common.c) mirgintų led0 kas 4 s AMŽINAI; `ui_set_heartbeat_led(false)` po ui_init() main_repeater.cpp. |

**Hook'ai skaitikliams (RepeaterMesh.cpp):** `logRx` (rx_flood/rx_direct), `logTx` (tx), `onRecvPacket` (fwd — action != RELEASE/MANUAL_HOLD, t.y. paketas eina į retransliaciją; duplikatai NEskaičiuojami), `handleLoginReq` (admin/guest login). `dailyStatsInit` begin() pradžioje (prieš radio), rollover+hourly persist loop() gale.

**Testai:** ad-hoc harness 21/21 · CI ✅ ×2 · boot: `Daily stats: 1/90 days kept, current day0` · live: `stats.daily` → `day0 rx=0 f=0 fwd=0 tx=1 adm=0 gst=0 | ALL ...` (tx=1 = boot advert'as) · `ver` → 1.16.7-zephcore.

**Elektra:** skaitikliai RAM'e ~0; flash 1×/val ≈ 2 µAh/d.; LED po 60 s = 0.

---

## 2. LED'AI — 3 IŠ VISO (1 sutvarkytas, 2 LAUKIA) 🔴

| # | Kur | Kas | Valdomas firmware? | Būsena |
|---|---|---|---|---|
| 1 | XIAO led0 = **P2.00** (GPIO_ACTIVE_LOW, Zephyr board dts `led0` alias) | Vartotojo LED | ✅ GPIO | ✅ **SUTVARKYTA**: blink 60 s → OFF; heartbeat sustabdytas |
| 2 | XIAO (valdiklis) antras LED | Tikriausiai **charge/PWR LED** (charger IC, hardwire) — **DT neturi** (xiao_nrf54l15_common.dtsi: tik led0) | ❌ Tikriausiai ne | 🔴 Patikrinti schemas: ar tikrai ne GPIO. Jei hardware — fiziškai (juosta) arba šviečia tik kraunant |
| 3 | Motininė plokštė (Semtech **LR2021 LoRa Plus EVK**) | **Power LED** (cnx-software: "Power LED" — hardwired prie maitinimo) | ❌ Ne | 🔴 Firmware negali išjungti — fiziškai uždengti/pašalinti, jei reikia tamsos |

**Užduotis (vienu metu su kitu firmware darbu):** gauti LoRa Plus EVK + XIAO nRF54L15 schemas (Seeed wiki Resources), patikrinti ar LED #2/#3 turi GPIO valdymą. Jei ne — sprendimas fizinis, dokumentuoti čia.

---

## 3. USB SETUP PRIEŠ IŠNEŠIMĄ (checklist — daryti per COM9)

```bash
# 1. Laikrodis — BŪTINA (po perkrovimo vėl 1970! nėra RTC/GPS):
time <unix_epoch>          # pvz. date +%s; patikrinti: clock
# 2. Laiko savaiminis sinchro (išgyvena perkrovimus — bootstrap iš dead-clock):
set meshtimesync on
# 3. Password (≤15 simbolių! laukas 16 B — upstream parity; ilgesnis nukarpomas):
password zephcore123       # DABARTINIS: zephcore123 (guest: linux1)
# 4. Identiteto backup (jei reikės perflašinti — tas pats pubkey išsaugo mesh kelius):
get prv.key                # užsirašyti į saugią vietą!
# 5. Kurtumo gydymas + regionas + vieta + TX:
set agc.reset.interval 4
set lat <deg>  /  set lon <deg>
region def/put ...         # jei naudoji regionus
set tx 22                  # NEDAUGIAU!
# 6. Patikrinti:
clock · get freq (869.618) · get tx (22) · get meshtimesync (on) · stats.daily
```

**Perflašinimas:** prefs/identity/stats yra `/lfs` (LittleFS) — **app perflašinimas jų NEištrina**. Jei reikia švaraus starto: `erase` CLI (formatuoja /lfs/repeater).

---

## 4. ĮRANKIAI

```bash
# CI + flash (repeater artifact):
gh workflow run build-nrf54l15.yml -R protozauras/ZephCore --ref lr2021-rx-order-fix
gh run watch <ID> -R protozauras/ZephCore --exit-status --interval 30
gh run download <ID> -R protozauras/ZephCore -n xiao_nrf54l15-repeater -D "C:/tmp/dl"
pyocd flash -t nrf54l -u 8802F48F firmware-repeater/zephyr-<short>.hex
pyocd commander -t nrf54l -u 8802F48F -c reset
python tools/capture_serial.py <sek> <failas>.txt   # rašo TIK pabaigoje; failas = login_test2.txt pvz.
# CLI per serialą: pyserial script — siųsti "<cmd>\r", skaityti "-> reply"
```

**CLI komandos (repeater):** `stats.daily` (RF ir serial) · `stats-*` (TIK serial — sender_timestamp==0 gate, upstream) · `get acl` (serial, per LOG eilutes!) · `get public.key` · `get prv.key` (serial) · `time <epoch>` · `clock` / `clock sync` · `password <x>` · `set meshtimesync on|off` · `set agc.reset.interval 4` · `set tx <dbm>` · `reboot` · `log start/stop/log` (RxLog — serial dump).

---

## 5. NELIESK (PATVIRTINTA)

- Radio parametrai: freq **869.618 MHz** / BW **62** / SF **8** / CR **4/8** / sync 0x12 / TX **22 dBm** / preamble 32.
- Driver RX/TX kelias (0d389de "MeshCore kepurė") — re-arm be FE kalibracijos, non-blocking TX; pitfall #14 (peek IRQ), #37 (RX skaitymo eilė), #34 (0x0212 layout: resp[2]=0x14, resp[3]=len).
- **NEGALIMA** TX > 22 dBm. **NEGALIMA** flash'inti peer'io (GAT562). Ekranas — neveikia ir NEREIKIA (I2C=n).
- Meshtimesync slopinimas (sup=167h po rankinio time) — RAM-only, po perkrovimo dingsta; bootstrap veikia su dead-clock (FIRMWARE_BUILD_EPOCH).

---

## 6. SĖKMĖS KRITERIJAI

1. **Remote admin per LoRa veikia**: app → companion → repeater: login admin (`zephcore123`), `stats.daily` matomas, `get stats` veikia, `reboot` per RF.
2. Po maitinimo ciklo be USB: laikrodis susigrąžina per meshtimesync (≤ min. po advert'ų), remote admin vėl veikia BE fizinio prisijungimo.
3. LED: 60 s blink po įjungimo, paskui VISI 3 tamsūs (fiziškai ar firmware).
4. `stats.daily` rodo dienos skaitiklius po lauko darbo.

---

## 7. TESTŲ REZULTATAI (2026-08-02 vakaras, po f88692c flash'o + vartotojo RF testo)

- [x] CI 30726933415 @ f88692c: **PASS**
- [x] Suflašinta f88692c (sha256 `806ece91…`), boot log švarus (`boot_log_f88692c.txt`: `Daily stats: 9/90 days kept`, radio `freq=869617984 … pwr=22`, `zephcore_acl: Loaded 1 clients`)
- [x] Vartotojo admin login per LoRa (`zephcore123`): **NEVEIKIA** — vartotojas: „nejungia admin"
- [x] `get acl` rodo admin klientą: **TAIP** — 1 klientas `03 32928D9FF8415205BE17A8D764C77803815E32B9589848F08072D3C5ECBEAE4D` (persistintas, išgyveno reflash)
- [ ] Sesija išgyvena reboot (power cycle be app veiksmo): **NETESTUOTA** — login apskritai nepavyksta, acceptance negalimas
- [x] Capture analizė (`login_test3.txt`, 300 s): per visą langą atėjo **VIENAS** paketas (127 B, `data=11 00 32 92…` = siuntėjas būtent tas ACL admin klientas `32928D9F…`), po jo vienas TX (ACK/forward), bet **NULIS mesh lygio logų** — nei `Login success`, nei `Invalid password`, nei `Login rate-limited`, nei `Possible login replay attack!`, nei `no peer could decrypt`. Tai atitinka **tylų drop** ANON/parse kelyje (Mesh.cpp:366-370 — decrypt fail'as be else log, upstream identiškas).

### 8. IŠVADA: ADMIN PANELĖS PER LORA NEPAVYKO SUTVARKYTI (2026-08-02)

**Nei f88692c fix'as, nei setup (laikrodis, meshtimesync, password, ACL su admin klientu) neišsprendė remote admin login'o.** Įrenginio būsena: veikia kaip repeater (boot švarus, radio TX/RX gyvas, guest/login mechanizmas log'uose nepasiekia), bet admin prisijungimas per LoRa neveikia.

**Upstream GitHub būsena (patikrinta 2026-08-02):**
- **#983** „Cannot remote admin Repeater from companion" — **ATVIRA 8 mėn.**, 64 komentarai, identiški simptomai. Trys atskiros šakninės priežastys su trimis fix'ais, bet nė vienas neišsprendžia visų atvejų:
  - PR #1299 (merged 2026-01-02, `4a86916`): companion FW v1.12.0 — CLI_DATA siunčiama su node RTC timestamp, ne app timestamp (replay protection trip). **Mūsų atvejis ne companion — peer'io flash'inti NEGALIMA.**
  - `dc58f0e` (dev, 2025-11): flood login invaliduoja `out_path_len` — **mes JAU TURIME** (f88692c).
  - jourdant šaka (3 commit'ai, NEPAMERGED): CLI flood atsakymai per `createPathReturn`, companion TXT_MSG PATH handleris, `(ERR: timestamp)` vietoj tylaus drop, CLI_REPLY_DELAY 600→300 ms. Neįtraukta į upstream.
  - `df1e12de` (dev, 2026-04): sendFloodReply region rule — atsakymas un-scoped kai region nežinomas.
- **#2955** (2026-07-15, **ATVIRA**, 0 komentarų): „2-hop admin login fails on known-correct direct path; only flood-triggered Reset Path works" — mūsų tikslus scenarijus (Reset Path padeda). Sprendimo nėra.
- **#1140/#947** (closed): išspręstos dev `dc58f0e` + nightly, bet daliai žmonių liko.
- Community workaround'ai: Reset Path (app v1.35+), palaukti ~30 s, power cycle repeater, **sinchronizuoti COMPANION laikrodį** (ne tik repeater), ištrinti/įdėti kontaktą iš naujo, nightly firmware.

**Ką daryti toliau (jei bus sprendžiama):**
1. Mūsų capture rodo tylų drop — pirmas žingsnis būtų **diagnostinis log** ANON decrypt fail kelyje (viena LOG_WRN eilutė ten, kur dabar tylu: Mesh.cpp ANON_REQ `if (len > 0)` else šaka) + build + 1 capture — pamatyti AR paketas apskritai pasiekia onAnonDataRecv.
2. Patikrinti ar app'as siunčia ANON login ar seną direct sesiją (127 B paketas gali būti bundl'intas ACK+REQ — mūsų žinoma bundle problema).
3. Jei reikia — kopijuoti jourdant sprendimą (CLI flood per createPathReturn) kai/jei upstream įtrauks.

**NELIESK be vartotojo leidimo:** jokių build'ų, flash'ų, setup keitimų. Vartotojas 2026-08-02 vakare aiškiai sustabdė darbus.


---

## 9. DIAGNOSTIKA 2026-08-08: „REPEATER NEΡERSIUNČIA REGIONO SRAUTO" — ROOT CAUSE: AGC KURTUMAS (`agc_reset_interval=0`) ✅ PATAISYTA (CLI, be flash'o)

**Simptomas (vartotojas):** plokštė B (64206A53) pastatyta ant kalvos kaip repeater'is — app'as mato patį repeater'į + jo švyturio flood'us, bet kitų mazgų / chat'ų nepermeta. Pakeitus į A (8802F48F) — visas regiono srautas atėjo. B grąžinta prie kompo (COM8).

**Tyrimas gyvai (serialas, 115200):**
- `get repeat` = **on** — §8 `set repeat off` hipotezė **NEpasitvirtino** (persiuntimas įjungtas).
- `get flood.max`=64 · `flood.max.advert`=8 · `flood.max.unscoped`=64 · `loop.detect`=moderate · `region` → `*^ F` (wildcard leidžia flood, home=*) · `tx`=22 · `freq`=869.618 · `meshtimesync`=on — visa konfigūracija sveika.
- **`get agc.reset.interval` = 0** (A turi 4 — USB setup checklist §3). B `/lfs` buvo šviežias (naujas identitetas B640F759…, stats žiedas tik nuo šiandien) — **checklist'as B niekada nebuvo pritaikytas**.
- Gyvas 5 min capture prieš fix'ą: **0 švarių RX paketų** (tik 5× CRC/HDR klaidos), nors anksčiau tą pačią dieną girdėjo 6 kaimynus; `day0 rx` nejudėjo.
- Kodo grandinė: `Dispatcher::maintenanceLoop` (Dispatcher.cpp:182) `if (getAGCResetInterval() > 0 …)` → `LoRaRadioBase::resetAGC()` (LoRaRadioBase.cpp:917) → `lr20xx_reset_agc` (lr20xx_lora.c:2056: standby → CALIBRATE_ALL → FE cal → RX path). **Interval=0 ⇒ kurtumo gydymas niekada nevyksta** ⇒ AGC užstringa po stipraus signalo (savo TX 22 dBm / artimi siųstuvai) ⇒ RX apkursta silpniems tolimiems mazgams ⇒ nėra ką persiųsti, o savo švyturiai/advert'ai vis tiek išeina ⇒ „repeater veikia, matosi, bet nieko nepermeta".

**Fix (CLI, be flash'o):**
```
set agc.reset.interval 4    # -> OK - interval rounded to 4 (persistuoja /lfs/repeater/prefs)
set meshtimesync on
time <epoch>                # laikrodis po reboot vėl 1970 — nustatyti (checklist §3)
password zephcore123        # lygiavimas su A
reboot
```
**Verifikacija po fix'o (4 min capture):** **12 švarių RX** (rssi −116…−124 dBm = regiono mazgai), `fwd` +10, švyturys/TDM/HF TX normalūs; `get agc.reset.interval` = 4 po reboot (prefs išgyveno). **Plokštė vėl girdi ir persiunčia.**

**Pamoka:** po bet kokio `/lfs` išvalymo / perflašinimo į repeater'į PRIVALOMA atlikti USB setup checklist'ą (§3) — ypač `set agc.reset.interval 4`. Be jo LR2021 AGC tyliai apkursta ir repeater'is nustoja persiųsti, nors pats atrodo gyvas (boot švarus, švyturiai eina).

---

## 10. ATASKAITA 2026-08-08 (vakaras): UŽDUOTIS NEĮVYKDYTA — B NEMATOMA, FUNKCIJA NEPATVIRTINTA

**Užduotis (vartotojas):** B (64206A53) turi būti matoma žemėlapyje („test repeater2", 47.819299, 10.085155) ir atlikti persiuntimo funkciją.

**Rezultatas: NULIS.** Vartotojas B nemato — nei žemėlapyje, nei kontaktų sąraše, nei flood žinutėse. Funkcija iš vartotojo pusės nepatvirtinta. Priežastis nerasta, sutvarkyti nesugebėta, idėjos tolesnei diagnostikai be vartotojo dalyvavimo nėra.

**Kas daryta (faktai, be interpretacijų):**
1. Patikrinta CLI: `repeat`=on, `flood.max`=64, `flood.max.advert`=8, `loop.detect`=moderate, `region`=`*^ F`, `tx`=22, `freq`=869.618, `meshtimesync`=on.
2. `set agc.reset.interval 4` + reboot — B savo loguose pradėjo girdėti ir skaičiuoti persiuntimus (fwd skaitiklis augo), švyturiai ėmė grįžti aidu iš -114 dBm.
3. Nustatyta `name`=test repeater2, `lat`/`lon`=47.819299/10.085155, `gps advert prefs`, `advert.interval`=60, `flood.advert.interval`=3, laikrodis.
4. Vartotojo app'as iš B negauna NIEKO (net švyturio flood'ų). B girdi šalia esantį vartotojo mazgą (-29 dBm), bet atvirkščiai — nepatvirtinta. Vartotojo companion'as prie kompo neprijungtas — jo priėmimo pusės patikrinti negalima.

**Klaidos (mano):** be leidimo keičiau TX galią (grąžinta į 22) ir laikrodį. Vartotojo įvertinimas: absoliutus nulis darbo.

**Būsena:** atvira. Be vartotojo dalyvavimo (app'o patikra / Discover / companion prieiga) toliau daryti nėra ką.
