# Elektrа — matavimo planas (po RSSI on-air patvirtinimo)

## Tikslas (vartotojo komisija 2026-08-06)
Vartotojo 31,5 mA — TEORINIS skaičius, juo pasikliauti negalima.
Reikia: (1) patikrinti matavimo SĄLYGAS, (2) surasti NUOTĖKĮ (nepaaiškinamą srautą),
(3) kiekvieno komponento srovę iš datasheet, ne vartotojo skaičiaus.

## Ką jau patikrinau kode (2026-08-06)
| Eilutė | Būsena | Pastaba |
|---|---|---|
| CONFIG_PM (system) | OFF (sąmoningai) | nRF54L15 neturi power-states Zephyr 4.4; idle = WFI System ON |
| CONFIG_PM_DEVICE | ON | tik device PM, nieko automatiškai nesuspenduoja |
| CONFIG_USB_DEVICE_STACK | NĖRA | USB periferija neįjungta firmware — USB "tax" mažas |
| Console | UART20 → SAMD11 CMSIS-DAP → USB CDC | LOG ir CLI eina per SAMD11 tiltą |
| LOG_DEFAULT_LEVEL=3 (board.conf!) | **AKTYVUS** | prj.conf LOG=n, bet board.conf perrašo → LOG 3 per UART. Kandidatas mažinti 3→1 |
| BLE adv | 20 ms fast pirmas 60 s, po to 211 ms slow | po 60 s jau mažas vidurkis |
| RF switch regulator | boot-on (būtini) | neliesti |
| LED led0 (P2.00) | blink 60 s → OFF (repeater 8bec55e) | patvirtinti gyvai + companion |
| Motininės plokštės LED | **hardwire Power LED** | firmware neišjungs — fizinis sprendimas |

## Matavimo protokolas (vartotojas su multimetru; po RSSI testo)
0. SĄLYGOS: šaltinis ≥3,7 V arba USB; matuoti ≥2 min po boot; užfiksuoti build,
   įtampą, USB taip/ne, BLE būseną (adv/connected).
   ⚠️ TPS62843 artefaktas: <3,3 V srovė "spiral" — matuoti tik ≥3,7 V arba USB.
1. Radijas STBY vs RX (Δ ~5,7 mA) — per repeater CLI / laikinas build.
2. BLE off vs on (repeater vs companion) → BLE Δ.
3. CONFIG_LOG_DEFAULT_LEVEL 3→1 → Δ (~0,1-0,5 mA). Produkcijai log lygmuo mažinti.
4. USB atjungtas (baterija) → SAMD11 Δ (ir LED #3 motininės — ar jis tikrai hardwire).
5. (vėliau, atskira užduotis) radio sleep prototipas companion → 5,7 → <0,5 mA.

## Sėkmės kriterijai
- repeater ≤10 mA (benchmark 8,4-9,1); companion su sleep ≤2-3 mA.
- Kiekvieno komponento srovė žinoma ir užrašyta; nuotėkis dokumentuotas.
- LED: blink 60 s → visi 3 tamsūs (1 firmware + 2 hardware/fiziškai).
