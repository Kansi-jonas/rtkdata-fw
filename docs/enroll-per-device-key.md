# Design-Vertrag: Per-Device Enroll-Key (ersetzt das Fleet-Secret)

Status: ENTWURF, wartet auf Freigabe (Jonas). Kein Code geaendert solange nicht freigegeben.

## Ziel / Threat-Model

Heute HMAC-signiert jede Firmware den Enroll mit EINEM flottenweiten `RTK_ENROLL_SECRET`,
einkompiliert in jede `.bin`. Risiko: ein Flash-Dump ODER die oeffentliche `ota/*.bin`
leakt das Secret, und damit kann jeder Enrolls faelschen -> Fake-Basen -> gefaelschte
RTCM-Korrekturen ins Netz.

Fix: KEIN geteiltes Secret in der Firmware. Jedes Geraet bekommt einen EIGENEN Enroll-Key,
beim Flashen ins NVS geschrieben (nie in die `.bin`). Der Key wird aus einem Master-Key
abgeleitet, der NUR auf der IE + dem Flash-Host lebt.

Eigenschaften:
- Ein kompromittiertes Geraet leakt nur SEINEN Key (Ableitung ist einweg, Master bleibt sicher).
- IE-Verify ist ZUSTANDSLOS (kein per-Device-Registry): die IE leitet den Key beim Enroll
  selbst aus der im Request mitgesendeten chip_id + Master ab.
- Master ist rotierbar (key_version-Feld).

## Krypto (FW und IE muessen das BYTE-EXAKT gleich tun)

KDF (ein 256-bit Subkey, daher reicht ein HMAC, kein volles HKDF-Expand):

    k_bytes      = HMAC_SHA256( key = MASTER_KEY_RAW(32 bytes),
                                msg = "rtkdata-enroll-key-v1:" + chip_id_hex )   # 32 bytes
    enroll_key   = hex(k_bytes)                                                  # 64 lowercase hex chars

- `MASTER_KEY` = 32 zufaellige Bytes, gehalten als 64-hex. Der KDF-HMAC verwendet die
  ROHEN 32 Bytes als Key. Lebt NUR auf IE (ENV) + Flash-Host, NIE in der Firmware.
- `chip_id_hex` = die ESP32 Base-MAC (`esp_read_mac(ESP_MAC_WIFI_STA)` == `esptool read_mac`),
  6 Bytes als **12 lowercase hex chars, OHNE Trenner**, z.B. "a1b2c3d4e5f6".
  IE: `chip_id_hex = normalize(req.mac)` mit `normalize = [^0-9a-fA-F] entfernen, lowercase`.
  (req.mac kommt als "A1:B2:..." -> "a1b2..."; identisch zu dem, was der Flash-Host aus
  `esptool read_mac` ableitet.)
- WICHTIG: `enroll_key` wird als 64-Zeichen-Hex-STRING verwendet, wenn damit der canon-HMAC
  gekeyt wird. Grund: beide Seiten keyen heute mit einem String (FW `strlen`, Node utf8);
  ein roher 32-Byte-Key koennte ein 0x00 enthalten und der FW-`strlen`-Key wuerde abschneiden.
  Der Hex-String hat keine Nullbytes.
- `key_version` = 1 (erlaubt spaetere Master-Rotation ohne Re-Flash aller Geraete:
  die IE haelt {1: master_v1, 2: master_v2} und waehlt nach `key_version` aus dem Request).

Enroll-Signatur (kanonischer String bleibt EXAKT wie heute in provisioning.c::enroll):

    canon = device_id + "|" + mac + "|" + fw_version + "|um980|" + nonce   # nonce als %u (uint32)
    hmac  = HMAC_SHA256( key = enroll_key (64-hex string), msg = canon )   # hex, lowercase

## NVS (Geraeteseite)

Neuer NVS-Key (Namespace wie die uebrigen rtk_-Keys):
- `rtk_enroll_key`  : string, 64 lowercase hex chars (= hex des abgeleiteten 32-Byte-Keys).
                      NICHT in die .bin, NICHT ins git.
- `rtk_key_ver`     : u8 (welche Master-Version diesen Key erzeugt hat; Default 1).

FW-Verhalten:
- `enroll()` liest `rtk_enroll_key` aus NVS. Fehlt er -> Geraet ist NICHT provisioniert,
  Enroll wird gar nicht versucht (Log: "unprovisioned: no enroll key"), Status bleibt UNCLAIMED.
- Das `#define RTK_ENROLL_SECRET` und alle Faelle, die es als HMAC-Key nutzen, werden ENTFERNT.
- Heartbeat nutzt weiterhin den `station_token` (post-enroll, unveraendert).

## IE-Verify (`/api/edge/enroll`)

1. `key_version` + `mac` aus dem Request lesen.
2. `enroll_key = HMAC_SHA256(master[key_version], "rtkdata-enroll-key-v1:" + normalize(mac))`.
3. `verifyEdgeHmac(enroll_key, canon, req.hmac)` -> bei Fehlschlag 401.
4. Rest unveraendert (insertEdgeDevice, upsertEdgeUploadCredential, station_token zurueck).

ENV: `EDGE_ENROLL_MASTER_V1` (64 hex). `EDGE_ENROLL_SECRET` (alt, flottenweit) entfaellt
nach der Migration.

## Flash-Host-Tool (neu, klein)

Pro Geraet, am Flash-Host (Master liegt hier in einer ENV/Datei, nie im Repo):

1. chip_id lesen: `esptool.py read_mac` -> normalisieren (lowercase, ohne ':').
2. `enroll_key = HMAC_SHA256(master, "rtkdata-enroll-key-v1:" + chip_id_hex)`.
3. NVS-Partition mit `rtk_enroll_key`(blob) + `rtk_key_ver`(u8) erzeugen
   (`nvs_partition_gen.py`) und in die nvs-Partition flashen (Rest des Flashs unveraendert,
   App-`.bin` traegt KEIN Secret mehr).
4. Optional Log/Inventar: device_id + chip_id (KEIN key) festhalten.

Kein IE-Registry-Schritt noetig (zustandsloses Verify).

## Migration (das eine Test-Geraet)

- Master_v1 erzeugen, in IE-ENV + Flash-Host hinterlegen.
- Test-Geraet einmal mit dem NVS-Key re-provisionieren (Tool Schritt 1-3).
- Enroll end-to-end gegen die IE beweisen, BEVOR ein zweites Geraet ausgerollt wird.

## Ausdruecklich NICHT in diesem Schritt (separate Haertung)

- #2 OTA-Kanal/Repo privat bzw. token-gated (heute public raw.githubusercontent).
- #3 Flash-Encryption + Secure-Boot (eFuse) + signierte OTA-Images.
- Diese reduzieren weiteres Risiko, sind aber unabhaengig; #1 nimmt das katastrophale
  Public-Leak-Risiko bereits raus.

## Restrisiko nach #1

- Master-Key-Leak (server/host) = Flotte wieder faelschbar -> Master streng schuetzen
  (IE-ENV + Flash-Host only, nie ins Repo/CI-Log), rotierbar via key_version.
- Ohne #3 ist ein einzelner Geraete-NVS-Dump weiterhin moeglich -> leakt aber nur DESSEN Key.
