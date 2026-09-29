# RTKdata-Station — Einrichtung

Willkommen! Diese Station ist eine präzise GNSS-Referenzstation. Einmal aufgebaut,
misst sie ihre Position zentimetergenau ein und liefert Korrekturdaten. Die
Einrichtung dauert nur wenige Minuten.

## Lieferumfang
- RTKdata-Station (Gerät)
- GNSS-Antenne
- Gewindestange (30 cm)
- Antennenkabel (9 m)
- WLAN-Antenne
- USB-Kabel

## Zusätzlich benötigt
- USB-Netzteil 5 V für Dauerbetrieb (keine Powerbank)
- Smartphone oder Laptop
- Zugangsdaten deines WLANs am Standort (Name + Passwort)
- Ein Montageort mit freier Sicht zum Himmel

## Aufbau

**1. Antenne montieren**
Schraube die GNSS-Antenne auf die Gewindestange. Befestige die Stange fest an einem
Ort mit möglichst **freier Sicht zum Himmel** (Dach, Mast, Geländer), rundum offen,
weg von Wänden, Metall und anderen Antennen. Je freier der Himmel, desto schneller
und genauer misst sich die Station ein.

> **Wichtig:** Die Antenne bleibt dauerhaft an dieser Stelle. Die Station misst ihre
> exakte Position ein und darf danach nicht mehr bewegt werden.

**2. Antennenkabel anschließen**
Verlege das 9-m-Kabel von der Antenne zur Station und schraube es handfest am
Antennenanschluss der Station an (nicht überdrehen).

**3. WLAN-Antenne anschrauben**
Schraube die WLAN-Antenne an die Station.

**4. Strom anschließen**
Verbinde die Station per USB-Kabel mit einem 5-V-Netzteil. Die Station läuft im
Dauerbetrieb (24/7).
→ Die **rote** LED leuchtet: die Station hat Strom.

**5. WLAN einrichten (einmalig)**
Beim ersten Start blinkt die **blaue** LED (sucht WLAN). Die Station öffnet ein
eigenes WLAN namens **`RTKdata_XXXX`** (der genaue Name steht auf dem Gerät).
1. Verbinde Smartphone/Laptop mit diesem WLAN (offen, kein Passwort).
2. Öffne den Browser → **`192.168.4.1`** (oder das Anmeldefenster öffnet sich von selbst).
3. Wähle dein Standort-WLAN, gib das Passwort ein und speichere.

→ Die Station verbindet sich. Die **blaue** LED leuchtet jetzt **dauerhaft** = WLAN verbunden.

**6. Einmessung & Livegang**
Sobald die Antenne Satellitendaten liefert, **blinkt die grüne** LED. Die Station
meldet sich automatisch im RTKdata-Netz an und beginnt, ihre präzise Position
einzumessen.
→ Nach **ca. 2 Tagen** ist die Station vollständig eingemessen und live. Einfach
laufen lassen, nicht bewegen, nicht ausschalten.

## LED-Status

| LED | Zustand | Bedeutung |
|---|---|---|
| 🔴 Rot | leuchtet | Strom an, Gerät läuft |
| 🔵 Blau | blinkt | sucht WLAN / keine WLAN-Verbindung |
| 🔵 Blau | leuchtet dauerhaft | mit WLAN verbunden |
| 🟢 Grün | aus | keine Satellitendaten |
| 🟢 Grün | blinkt | empfängt Satellitendaten |

**Normalbetrieb:** 🔴 Rot leuchtet · 🔵 Blau dauerhaft · 🟢 Grün blinkt.

## Problemlösung
- **Blaue LED blinkt weiter** nach dem Setup → WLAN-Name oder Passwort falsch, oder
  Station außer Reichweite. WLAN neu einrichten (siehe unten).
- **Grüne LED bleibt aus** → Antennenkabel prüfen (fest angeschraubt?) und freie
  Himmelssicht sicherstellen.
- **WLAN neu einrichten / Station zurücksetzen** → Reset-Taste gedrückt halten
  (ca. 6-8 Sekunden), bis die Station neu startet. Das hinterlegte WLAN wird
  gelöscht; die Station öffnet danach wieder ihr Setup-WLAN `RTKdata_XXXX`.

## Support
Fragen? **jonas@rtkdata.com**
