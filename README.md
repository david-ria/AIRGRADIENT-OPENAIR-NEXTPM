# AIRGRADIENT-OPENAIR-NEXTPM

ESP32-C3 firmware for connecting a **NextPM particulate matter sensor** and a **Senseair S8 CO₂ sensor** to the **AirGradient Cloud**.

This project provides an open implementation to collect high-quality air quality data (PM and CO₂) and publish it periodically to AirGradient.

---

## ✨ Features

- 📡 Reads **PM1 / PM2.5 / PM10** (µg/m³) from **NextPM** (simple protocol, 1-min average).  
- 📊 Reads **binned particle counts** (0.2–10 µm, Cnt/L) from **NextPM** via Modbus.  
- 🌫 Estimates **PM ≥ 0.3 µm** proxy counts in pcs/dL from binned channels.  
- 🌍 Reads **CO₂ concentration** (ppm) from **Senseair S8**.  
- 📶 Publishes sensor data + Wi-Fi RSSI to **AirGradient Cloud** every 10 seconds.  
- ⚙️ Wi-Fi setup via **captive portal** (includes custom Sensor ID).  
- 💾 Persistent **Sensor ID** stored in NVS (defaults to ESP32 MAC).

---

## 🛠 Hardware

- **ESP32-C3**  
- **NextPM sensor**  
  • UART1 @ 115200, 8E1  
  • RX = GPIO0, TX = GPIO1  
- **Senseair S8 CO₂ sensor**  
  • UART0 @ 9600, 8N1  
  • RX = GPIO20, TX = GPIO21

---

## 📡 Cloud Integration

Data is sent as JSON to:

POST http://hw.airgradient.com/sensors/airgradient:<serial>/measures

yaml
Copier le code

Where `<serial>` = STA MAC in lowercase without “:”.

---

### Example JSON payload

```json
{
  "wifi": -63,
  "pm01": 4.2,
  "pm02": 6.8,
  "pm10": 10.1,
  "pm003Count": 157,
  "rco2": 742
}
```
Fields are included only when valid.


🚀 How to Use
Clone this repository:

bash
Copier le code
git clone https://github.com/david-ria/AIRGRADIENT-OPENAIR-NEXTPM.git
Open in Arduino IDE or PlatformIO.

Upload to your ESP32-C3.

On first boot, a Wi-Fi captive portal shows up:

SSID: airgradient-xxxxxx

Password: cleanair
Enter your Wi-Fi credentials and optionally set a Sensor ID.

The device will begin posting data to AirGradient Cloud.

📂 Repository Structure
bash
Copier le code
AIRGRADIENT-OPENAIR-NEXTPM/
 ├── src/
 │    └── main.cpp       # Firmware source code
 ├── README.md           # Project documentation
 ├── LICENSE             # License (e.g. MIT)
 └── .gitignore          # Ignore build output etc.
📖 Notes
Default Sensor ID = airgradient:<MAC>.

pm003Count is a proxy based on binned channels (approx ≥ 0.3 µm).

Posting interval = 10 seconds (modifiable via POST_PERIOD_MS).

