# AGENT HANDOFF REPORT — XIAO nRF54L15 + Wio-LR2021 + ZephCore
# Data: 2026-07-28
# Plokštė: Seeed Studio XIAO nRF54L15 + Wio-LR2021 LoRa Plus
# UID: 8802F48F (CMSIS-DAP)
# COM port: COM9, 115200 baud

## UŽDUOTIS

Flashinti ZephCore (github.com/liquidraver/ZephCore) į XIAO nRF54L15 + LR2021.
Repeater režimas, matomas žemėlapyje (MeshCore app), veikia nuo baterijos.
Useris Vokietijoje, pozicija: 47.816972, 10.078722.

## 🔴 KAS NEVEIKIA — RADINYS NR.1

### Radijas nesiunčia/neima — LR2021 naudoja neteisingą driverį

**board.overlay** turi:
```dts
compatible = "semtech,sx1262";
```

Tai užkrauna **SX1262 Zephyr driverį**, kuris **nesupranta LR2021 SPI komandų**.
LR2021 ir SX1262 turi SKIRTINGĄ komandų setą, nepaisant pavadinimo "compatible".

**ZephCore turi atskirą LR2021 adapterį:**
- `zephcore/adapters/radio/LR2021Radio.cpp` — naudoja `lr20xx_lora.h` API
- `zephcore/adapters/radio/SX126xRadio.cpp` — naudoja Zephyr SX1262 driver
- Radio adapteris parenkamas **compile-time** — reikia rasti kur ir kaip

**Simptomai:**
- `advert` grąžina "OK - Advert sent" bet oru niekas neišeina
- CAD (Channel Activity Detection) rodo **0 visur** — 0 paketų, 0 trigerių
- Kitas MeshCore įrenginys (kambaryje, tas pats dažnis 869.618 MHz) nemato repeaterio
- Firmware boot'ina, serial CLI veikia, nėra crash — tik radijas tyli

**Ką reikia padaryti:**
1. Rasti teisingą DTS `compatible` stringą LR2021 ZephCore kontekste
2. Patikrinti kaip ZephCore parenka radio adapterį (compatible string? Kconfig?)
3. Perbuildinti per GitHub Actions ir perflashinti
4. Patikrinti ar rfsw_ctl/rfsw_pwr node'ai egzistuoja nRF54L15 DTS

**Overlay reference** naudoja `compatible = "seeed,lr2021"`:

## Kas padaryta ✅

| Veiksmas | Rezultatas |
|----------|-----------|
| Flash ZephCore v1.16.7 repeater | Serial CLI veikia COM9 |
| Konfigūracija | name=Narvydas-RPT, lat=47.816972, lon=10.078722, password=narvydas |
| Radio nustatymai | 869.618 MHz, BW 62.5, SF8, CR8 (EU default, sutampa su companion) |
| Advertisements | kas 60 min su pozicija (gps advert prefs) |
| SRAM override | cpuapp_sram = 256KB (be to hard-fault) |
| RF switch regulators | rfsw_ctl + rfsw_pwr regulator-boot-on |
| CONFIG_NRFX_POWER | board.conf — DCDC/LDO reguliatoriai |

## Repo būsena

- **Fork:** `protozauras/ZephCore` (master) — 3 commit'ai virš upstream
- **Upstream:** `liquidraver/ZephCore` v1.16.7 (naudoja SX1262 pinout — neteisingas LR2021)
- **Workflow:** `.github/workflows/build-nrf54l15.yml` — stato companion + repeater
- **Board overlay:** `zephcore/boards/nrf54l/xiao_nrf54l15/board.overlay`
- **Board conf:** `zephcore/boards/nrf54l/xiao_nrf54l15/board.conf`

Mūsų fork'o skirtumai nuo upstream (tik 3 failai):
```
.github/workflows/build-nrf54l15.yml         — naujas workflow
zephcore/boards/nrf54l/xiao_nrf54l15/board.conf   — NRFX_POWER + "LR2021" pavadinimas
zephcore/boards/nrf54l/xiao_nrf54l15/board.overlay — LR2021 pinout + SRAM 256KB + rfsw + I2C22
```

## Dabartinis board.overlay (PROBLEMINIS)

```dts
/* Pin mapping:
 * D0 (P1.04) = DIO8/IRQ    D8  (P2.01) = SPI SCK
 * D1 (P1.05) = BUSY         D9  (P2.04) = SPI MISO
 * D2 (P1.06) = RESET        D10 (P2.02) = SPI MOSI
 * D3 (P1.07) = CS (NSS)     D4  (P1.10) = I2C SDA (OLED)
 *                             D5  (P1.11) = I2C SCL (OLED)
 */

&cpuapp_sram { reg = <0x20000000 DT_SIZE_K(256)>; ranges = <0x0 0x20000000 DT_SIZE_K(256)>; };
&rfsw_ctl { regulator-boot-on; status = "okay"; };
&rfsw_pwr { regulator-boot-on; status = "okay"; };
&i2c22 { status = "okay"; };

&spi00 {
    cs-gpios = <&gpio1 7 GPIO_ACTIVE_LOW>;  /* D3 = P1.07 CS */
    lora: lora@0 {
        compatible = "semtech,sx1262";  /* ← NETEISINGA, reikia LR2021 */
        reg = <0>;
        spi-max-frequency = <8000000>;
        dio1-gpios = <&gpio1 4 (GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH)>;  /* D0 = P1.04 */
        busy-gpios = <&gpio1 5 GPIO_ACTIVE_HIGH>;   /* D1 = P1.05 */
        reset-gpios = <&gpio1 6 GPIO_ACTIVE_LOW>;    /* D2 = P1.06 */
    };
};
```

## Baterija / krovimas

**BQ25100 yra hardware-controlled** — firmware nereikalingas:
- USB prijungtas → krauna bateriją + maitina iš USB
- USB atjungtas → automatiškai persijungia į bateriją
- `CONFIG_NRFX_POWER=y` turėtų padėti su reguliatoriais ant baterijos
- VBAT ADC nežinomas (schematic 403 Forbidden, wiki neegzistuoja)
- Energijos sanaudos: ~25mA / 0.128W (normalus LoRa repeater lygis)

## Flash komandos

```bash
# Flash
pyocd flash -t nrf54l -u 8802F48F <path>/zephyr.hex

# Reset
pyocd reset -t nrf54l -u 8802F48F

# Status
pyocd cmd -t nrf54l -u 8802F48F -c "status"

# Backup (DAR NEPADARYTA!)
pyocd save -t nrf54l -u 8802F48F firmware-backup.hex
```

## Serial CLI komandos (COM9 115200)

```
ver                     — versija (1.16.7-zephcore)
board                   — XIAO nRF54L15
neighbors               — kaimynų sąrašas
advert                  — siųsti reklamą dabar
advert.zerohop          — 0-hop reklama
clock                   — laikas
reboot                  — perkrauti
set lat/lon <value>     — pozicija
set name <value>        — pavadinimas
password <value>        — admin slaptažodis
set guest.password <x>  — guest slaptažodis
set radio <freq> <bw> <sf> <cr>  — radijo parametrai
get radio               — skaityti radiją
get lat / get lon       — skaityti poziciją
gps advert prefs/share/none — lokacijos dalinimasis
set advert.interval <min>   — reklamos intervalas (60-240)
region                  — regiono medis
set repeat on/off       — pakartojimas
discover.neighbors      — paieška
```

## Prioritetas kitam agentui

1. **Rasti teisingą DTS compatible LR2021** — `grep -rn "lr2021\|lr20xx\|seeed,lr" zephcore/`
2. **Patikrinti kaip radio adapteris parenkamas** — `grep -rn "LR2021Radio\|SX126xRadio" zephcore/app/ zephcore/src/`
3. **Perbuildinti** su teisingu compatible → GitHub Actions
4. **Perflashinti** ir patikrinti ar CAD rodo >0
5. **Patikrinti rfsw_ctl/rfsw_pwr** — ar šie node'ai egzistuoja nRF54L15 DTSi
6. **Backup firmware** — `pyocd save` prieš kiekvieną flash

## Local failai

| Failas | Kelias |
|--------|--------|
| ZephCore klonas | `C:\Users\proto\ZephCore-nRF54L15-build\` |
| Repeater firmware | `C:\Users\proto\fw-repeater\zephyr.hex` |
| Companion firmware | `C:\Users\proto\fw-companion\build\zephyr\zephyr.hex` |
| LR2021Radio.cpp | `C:\Users\proto\ZephCore-nRF54L15-build\zephcore\adapters\radio\LR2021Radio.cpp` |
| SX126xRadio.cpp | `C:\Users\proto\ZephCore-nRF54L15-build\zephcore\adapters\radio\SX126xRadio.cpp` |
