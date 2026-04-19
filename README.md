# AIRGRADIENT-OPENAIR-NEXTPM

ESP32-C3 firmware for an AirGradient-compatible OpenAir board equipped with a **NextPM** particulate matter sensor, **Senseair S8** CO₂ sensor, and **Sensirion SGP41** gas sensor (TVOC/NOx). Publishes to the **AirGradient Cloud** and exposes a **local web dashboard + JSON API** for direct monitoring and diagnostics.

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
- Local **Wi-Fi captive portal** on first boot (SSID `airgradient-xxxxxx`, password `cleanair`) to configure Wi-Fi and Sensor ID
- **Persistent Sensor ID** in NVS (defaults to the STA MAC)
- **Local HTTP dashboard** on the device's LAN IP — see *Endpoints* below
- Posts to AirGradient Cloud every **30 s** (free-tier friendly; previous 10 s was hit by HTTP 429)

---

## Hardware (AirGradient OpenAir O-1PST, ESP32-C3)

| Peripheral | UART / Bus | RX / SDA | TX / SCL | Baud / Speed |
|---|---|---|---|---|
| NextPM | UART1 | GPIO0 | GPIO1 | 115200 8E1 |
| Senseair S8 | UART0 | GPIO20 | GPIO21 | 9600 8N1 |
| SGP41 + SHT4x | I²C | GPIO7 (SDA) | GPIO6 (SCL) | 100 kHz |

USB-CDC is used for `Serial` console (no pin conflict with UART0).

---

## AirGradient Cloud

JSON `POST` every 30 s to:

```
http://hw.airgradient.com/sensors/airgradient:<serial>/measures
```

where `<serial>` = STA MAC, lowercase, no colons.

### Example payload

```json
{
  "wifi": -43,
  "pm01": 2.1,
  "pm02": 3.4,
  "pm10": 5.6,
  "pm003Count": 32,
  "rco2": 502,
  "atmp": 24.60,
  "rhum": 51.20,
  "tvoc_index": 43,
  "nox_index": 1,
  "tvoc_raw": 27437,
  "nox_raw": 15516
}
```

Fields are included only when the corresponding sensor read succeeded. `tvoc_index` / `nox_index` are suppressed during the first ~10 s (SGP41 conditioning).

---

## Local Dashboard & Endpoints

Once connected to Wi-Fi the device serves:

| URL | Description |
|---|---|
| `http://<device-ip>/` | Live HTML dashboard (auto-refresh 2 s) |
| `http://<device-ip>/json` | Latest values + raw diagnostic bytes |
| `http://<device-ip>/macinfo` | Stored Sensor ID, STA MAC, AP MAC — useful when the ID drifts from the MAC |
| `http://<device-ip>/setid?id=<12hex>` | Overwrite the stored Sensor ID (saved to NVS) |
| `http://<device-ip>/clearid` | Reset Sensor ID back to STA-MAC-derived |
| `http://<device-ip>/probe` | NextPM Modbus register dump (`regs 0..229`) |
| `http://<device-ip>/i2cscan` | I²C bus scan across candidate SDA/SCL pin pairs |
| `http://<device-ip>/s8scan` | S8 UART RX-pin sweep for wiring debug |

mDNS alias: `http://openair-nextpm.local/` (works on OSes that resolve `.local`).

---

## Build

### Arduino CLI (recommended)

```bash
arduino-cli core install esp32:esp32
arduino-cli lib install "WiFiManager" \
                        "Sensirion I2C SGP41" \
                        "Sensirion Gas Index Algorithm" \
                        "Sensirion Core" \
                        "Sensirion I2C SHT4x"

arduino-cli compile \
  --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=min_spiffs,FlashSize=4M,CPUFreq=160,FlashFreq=80,FlashMode=qio" \
  sketch

arduino-cli upload \
  --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=min_spiffs,FlashSize=4M,CPUFreq=160,FlashFreq=80,FlashMode=qio" \
  --port COMxx \
  sketch
```

The `sketch/` folder is the Arduino-style build target. The same source is mirrored at `src/main.cpp` for PlatformIO users.

### Arduino IDE

Open `sketch/sketch.ino`, select **ESP32C3 Dev Module**, enable **USB CDC On Boot**, set **Partition Scheme: Minimal SPIFFS**, upload.

---

## First-time setup

1. Flash the firmware.
2. A Wi-Fi AP named `airgradient-xxxxxx` appears (password `cleanair`). Connect, pick your SSID, optionally set a Sensor ID.
3. The device restarts, connects to Wi-Fi, brings up the local dashboard and starts POSTing.
4. Register the Sensor ID (shown on `/macinfo` as `STA MAC (ag serial)`) in your AirGradient account at <https://app.airgradient.com/>. Until this is done, POSTs return HTTP 400.

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

- `pm003Count` is taken directly from the NextPM simple-protocol response (bytes 3-4), not from Modbus registers 128-137 — those turned out to be empty on this firmware revision.
- SGP41 requires ~2-5 min of 1 Hz sampling before its gas index algorithm produces meaningful TVOC/NOx values. Early readings are `0` and will climb to ~100 in clean air.
- POST period is 30 s. The AirGradient free tier rate-limits faster cadences with HTTP 429.
