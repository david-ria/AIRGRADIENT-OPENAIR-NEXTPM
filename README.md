# AIRGRADIENT-OPENAIR-NEXTPM

[![firmware-build](https://github.com/david-ria/AIRGRADIENT-OPENAIR-NEXTPM/actions/workflows/firmware-build.yml/badge.svg)](https://github.com/david-ria/AIRGRADIENT-OPENAIR-NEXTPM/actions/workflows/firmware-build.yml)

ESP32-C3 firmware for an AirGradient-compatible OpenAir board equipped with a **NextPM** particulate matter sensor, **Senseair S8** CO₂ sensor, and **Sensirion SGP41** gas sensor (TVOC/NOx). Publishes to the **AirSentinels backend** (`station.airsentinels.fr`, PocketBase) over HTTPS and exposes a **local web dashboard + JSON API** for direct monitoring and diagnostics.

---

## Features

- **NextPM** (UART1, 115200 8E1, RX=GPIO0, TX=GPIO1)
  - PM1 / PM2.5 / PM10 (µg/m³) from simple protocol `0x12` (1-min avg)
  - Cumulative particle count ≥ 0.3 µm in `pcs/dL` (fed to AirGradient as `pm003Count`)
- **Senseair S8** (UART0, 9600 8N1, RX=GPIO20, TX=GPIO21)
  - CO₂ in ppm
- **Sensirion SGP41** (I²C @ 0x59, SDA=GPIO7, SCL=GPIO6)
  - TVOC index and NOx index via the official Sensirion Gas Index Algorithm (1 Hz sampling, ~2-5 min baseline)
- **Sensirion SHT4x** (I²C @ 0x44, optional — skipped if not detected)
  - Temperature (°C) and Relative Humidity (%) — used to compensate SGP41 readings
- Local **Wi-Fi captive portal** on first boot (SSID `airgradient-xxxxxx`, password `cleanair`) to configure Wi-Fi, Sensor ID, and the **AirSentinels device token**
- **Persistent Sensor ID** in NVS (defaults to the STA MAC) — `device_serial` = the bare 12-hex
- **Local HTTP dashboard** on the device's LAN IP — see *Endpoints* below
- Posts to the **AirSentinels backend** every **30 s** over HTTPS (TLS, ISRG Root X1 pinned), authenticated with the device token (`X-Device-Token` header)
- **NTP-synced UTC timestamps** (`ts`); if NTP isn't synced yet the server stamps ingestion time

---

## Hardware (AirGradient OpenAir O-1PST, ESP32-C3)

| Peripheral | UART / Bus | RX / SDA | TX / SCL | Baud / Speed |
|---|---|---|---|---|
| NextPM | UART1 | GPIO0 | GPIO1 | 115200 8E1 |
| Senseair S8 | UART0 | GPIO20 | GPIO21 | 9600 8N1 |
| SGP41 + SHT4x | I²C | GPIO7 (SDA) | GPIO6 (SCL) | 100 kHz |

USB-CDC is used for `Serial` console (no pin conflict with UART0).

---

## AirSentinels backend

JSON `POST` every 30 s over HTTPS to:

```
https://station.airsentinels.fr/api/openair/ingest
```

Headers: `Content-Type: application/json` and `X-Device-Token: <device token>`.
TLS validated against the pinned **ISRG Root X1** root CA (Let's Encrypt).

The backend is a PocketBase instance behind Traefik. The ingest hook resolves
`device_serial → device`, auto-creating the device on first contact, and writes
a row to the `readings` collection.

### Example payload

```json
{
  "device_serial": "d83bda1d7888",
  "ts": "2026-06-25T07:35:00Z",
  "rssi": -43,
  "postAvgSec": 60,
  "pm1": 2.1,
  "pm25": 3.4,
  "pm10": 5.6,
  "pm003_dL": 32,
  "cntPM1_dL": 32,
  "pm_60s_ok": true,
  "pm_60s_pm25": 3.4,
  "co2": 502,
  "atmp": 24.60,
  "rhum": 51.20,
  "tvoc_index": 43,
  "nox_index": 1,
  "tvoc_index_avg": 41,
  "nox_index_avg": 1,
  "sensor_ok": true,
  "sgpConditioning": false
}
```

Fields are included only when the corresponding sensor read succeeded. The three
NextPM averaging windows (`pm_10s_*`, `pm_60s_*`, `pm_15m_*`) are all sent each
cycle. `tvoc_index` / `nox_index` are suppressed during the first ~10 s (SGP41
conditioning). `ts` is omitted until NTP syncs, in which case the server stamps it.

### Provisioning a station

On first boot, connect to the captive portal (`airgradient-xxxxxx` / `cleanair`)
and fill in: Wi-Fi credentials, optional Sensor ID, and the **AirSentinels device
token**. The token is the shared secret stored server-side in `/root/openair/.env`
(`OPENAIR_DEVICE_TOKEN`). It is persisted in NVS — re-entering it is only needed
to rotate it.

---

## OTA updates (pull-based)

Stations update themselves over the air — no field visits to reflash. The flow:

1. Each release bumps `FW_VERSION` in `src/main.cpp`.
2. `scripts/publish-firmware.sh` compiles, computes the MD5, and uploads the
   binary + a manifest to `station.airsentinels.fr/firmware/`.
3. Deployed stations poll `firmware/openair-nextpm.json` (45 s after boot, then
   every 6 h). If `version` differs from their `FW_VERSION`, they download the
   `.bin` over the same pinned-TLS channel, verify the MD5 during write
   (`Update.setMD5`), flash the spare OTA slot, and reboot.
4. **Self-healing:** a freshly installed image must produce a successful POST
   within 5 min or it rolls back to the previous slot. The `min_spiffs`
   partition scheme already provides two ~1.9 MB OTA slots, so no repartition is
   needed.

Publish a new release:

```bash
# bump FW_VERSION in src/main.cpp first, mirror to sketch/, then:
scripts/publish-firmware.sh
```

The manifest is uploaded last and atomically, so a station never sees a manifest
pointing at a binary that isn't there yet.

## Local Dashboard & Endpoints

Once connected to Wi-Fi the device serves:

| URL | Description |
|---|---|
| `http://<device-ip>/` | Live HTML dashboard (auto-refresh 2 s) |
| `http://<device-ip>/json` | Latest values + raw diagnostic bytes |
| `http://<device-ip>/macinfo` | Stored Sensor ID, STA MAC, AP MAC — useful when the ID drifts from the MAC |
| `http://<device-ip>/setid?id=<12hex>` | Overwrite the stored Sensor ID (saved to NVS) |
| `http://<device-ip>/clearid` | Reset Sensor ID back to STA-MAC-derived |
| `http://<device-ip>/settoken?token=<t>` | Set/rotate the AirSentinels device token (saved to NVS) |
| `http://<device-ip>/setwifi?ssid=<s>&pass=<p>` | Switch the station to another Wi-Fi (saved to NVS) |
| `http://<device-ip>/nextpmcmd?cmd=<hex>` | Send one NextPM simple-protocol read command (`11/12/13` mass, `25/26/27` bins, `16` T/RH) and dump the raw response — handy to check which channels a sensor firmware supports |
| `http://<device-ip>/probe` | NextPM Modbus register dump (`regs 0..229`) |
| `http://<device-ip>/i2cscan` | I²C bus scan across candidate SDA/SCL pin pairs |
| `http://<device-ip>/s8scan` | S8 UART RX-pin sweep for wiring debug |

mDNS alias: `http://openair-nextpm.local/` (works on OSes that resolve `.local`).

---

## Build

### Arduino CLI (recommended)

`src/main.cpp` is the **canonical source**; `sketch/sketch.ino` is a generated
mirror (the Arduino build target). The build scripts re-mirror automatically, so
edit `src/main.cpp` and never hand-copy.

Reproducible builds use the pinned profile in `sketch/sketch.yaml` (ESP32 core +
every library locked to a known-good version):

```bash
# build (mirrors src -> sketch, compiles with the pinned profile)
scripts/build.sh

# build + flash
scripts/build.sh --flash COMxx
```

Equivalent raw commands:

```bash
cp src/main.cpp sketch/sketch.ino
arduino-cli compile --profile openair --output-dir build sketch
arduino-cli upload  --profile openair --port COMxx --input-dir build sketch
```

For provisioning without the captive portal, copy `sketch/secrets.h.example` to
`sketch/secrets.h` (gitignored) and fill in Wi-Fi creds + the device token.
**Never** build a public OTA image with a real token in `secrets.h`.

### Arduino IDE

Open `sketch/sketch.ino`, select **ESP32C3 Dev Module**, enable **USB CDC On Boot**, set **Partition Scheme: Minimal SPIFFS**, upload.

---

## First-time setup

1. Flash the firmware.
2. A Wi-Fi AP named `airgradient-xxxxxx` appears (password `cleanair`). Connect, pick your SSID, set the **AirSentinels device token** (and optionally a Sensor ID).
3. The device restarts, connects to Wi-Fi, syncs NTP, brings up the local dashboard and starts POSTing to `station.airsentinels.fr`.
4. The device auto-registers in the backend on its first successful POST (a `devices` row keyed by `device_serial`). Check the PocketBase admin at <https://station.airsentinels.fr/_/>.

### If the stored ID doesn't match the chip MAC

Can happen after reflashing blank firmware on top of a unit that was previously configured manually. Call:

```
http://<device-ip>/macinfo             # see current stored ID and real STA MAC
http://<device-ip>/setid?id=<12hex>    # align stored ID with MAC
# or:
http://<device-ip>/clearid             # reset to MAC-derived ID
```

---

## Notes

- `pm003_dL` is taken directly from the NextPM simple-protocol response (bytes 3-4), not from Modbus registers 128-137 — those turned out to be empty on this firmware revision.
- SGP41 requires ~2-5 min of 1 Hz sampling before its gas index algorithm produces meaningful TVOC/NOx values. Early readings are `0` and will climb to ~100 in clean air.
- POST period is 30 s.
- TLS uses a pinned root CA (ISRG Root X1, valid until 2035). Let's Encrypt rotates its *intermediates*, not the root, so the pin survives cert renewals. If the backend ever moves off Let's Encrypt, update `ISRG_ROOT_X1_PEM` in the firmware.
