# ESP32 Flower Care Bridge

An ESP32 bridge for the **Xiaomi Flower Care (MiFlora)** plant sensor. It reads the sensor over Bluetooth LE and publishes the readings to **Home Assistant** through MQTT auto-discovery. There are no cloud services or phone apps, and nothing needs configuring in YAML.

It has a web page for readings and settings, a WiFiManager setup portal, OTA updates and a configurable deep sleep, so the ESP32 can run on a battery.

## Features

- **Sensor readings:** soil moisture (%), temperature (°C), light (lux), fertility/conductivity (µS/cm), battery (%) and the sensor's firmware version
- **Home Assistant auto-discovery:** all entities appear under one device, with proper unique IDs, device classes and units
- **Needs water binary sensor:** turns on when moisture drops below a level you set, so you can use it in an automation to get a notification
- **Diagnostics:** Bluetooth and WiFi signal strength, and the time of the last reading
- **Web page:** current readings, a history of recent readings (kept through deep sleep), power and sleep controls, status and an event log
- **Settings page:** all configuration is done in the browser. It includes a **Bluetooth scan** to find your sensor's MAC address
- **WiFiManager setup portal:** on first boot, enter your WiFi, MQTT details and sensor MAC. Nothing is hard-coded
- **OTA updates:** from the Arduino IDE (network port) or by uploading a `.bin` on the web page
- **Deep sleep:** stays awake for a set time after power-on, then wakes on a timer, reads, publishes and goes back to sleep
- **Maintenance mode:** a switch in Home Assistant that keeps a sleeping board awake at its next wake, so you can update or reconfigure it without power-cycling
- **Light/dark theme** for the web page

## Hardware

- Any ESP32 with Bluetooth (for example an ESP32 DevKit or WROOM-32 board)
- A Xiaomi Flower Care / MiFlora sensor (HHCCJCY01) within Bluetooth range of the ESP32, usually up to about 10 m depending on walls

No wiring is needed. The ESP32 talks to the sensor over Bluetooth.

> ESP32-S2 boards have no Bluetooth and won't work. ESP32-C3, -C6 and -S3 boards have BLE and should work, but they haven't been tested.

## Software requirements

**Arduino IDE** with the **ESP32 board package 3.x** (Espressif).

Install these libraries from the Library Manager:

| Library | Author | Version |
|---|---|---|
| WiFiManager | tzapu | 2.0.x |
| PubSubClient | Nick O'Leary | 2.8 |
| ArduinoJson | Benoit Blanchon | 7.x |

BLE, WebServer, HTTPUpdateServer, ArduinoOTA and LittleFS come with the ESP32 core.

## Board settings

| Setting | Value |
|---|---|
| Board | ESP32 Dev Module (or your board) |
| Partition Scheme | **Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)** |

The partition scheme matters. Bluetooth, WiFi and the web server together are too big for the default 1.2 MB app partition.

## First-time setup

1. Flash the sketch over USB.
2. On your phone or laptop, connect to the WiFi network **`FlowerCare-Setup`**. The portal should open automatically. If it doesn't, browse to `192.168.4.1`.
3. Pick your WiFi network, enter its password, then fill in:
   - MQTT broker host/IP, port, username and password
   - your Flower Care sensor's Bluetooth MAC address. Leave this blank if you don't know it yet, and use the scan on the Settings page later.
4. Save. The board joins your WiFi and stays awake for 15 minutes.
5. Open `http://flowercare.local/` (or the board's IP address) to see the readings.
6. In Home Assistant, the device appears under **Settings → Devices & services → MQTT**.

### Finding the sensor's MAC address

Go to **Settings → Flower Care sensor → Scan for sensors**. Flower Care devices are listed first. Click an address to fill it in, then click **Save & restart**.

## How the power and sleep cycle works

| Event | What happens |
|---|---|
| Power-on / restart | Stays awake for **Awake after power-on** minutes (default 15). The web page and OTA are available, and the sensor is read every **Read interval while awake** (default 5 min). |
| Awake time over | Deep sleeps for **Deep sleep between readings** minutes (default 360 = 6 hours). |
| Timer wake | Connects WiFi and reads the sensor at the same time, publishes to MQTT, then sleeps again. It's usually awake for 5–15 s and the web page isn't started. |
| Maintenance mode on | Stays awake until maintenance mode is turned off. |

The web page also has **Read sensor now**, **Stay awake longer**, **Maintenance on/off** and **Sleep now** buttons.

### Reaching a sleeping board

Turn on the **Maintenance mode (stay awake)** switch on the device in Home Assistant. The command is retained by the broker, so the board picks it up the next time it wakes and stays awake. Do your OTA update or settings change, then turn maintenance mode off. The board waits a minute and goes back to sleep.

## Home Assistant entities

| Entity | Type | Notes |
|---|---|---|
| Moisture | sensor | % |
| Temperature | sensor | °C |
| Light | sensor | lx |
| Fertility | sensor | µS/cm (soil conductivity) |
| Battery | sensor | % |
| Needs water | binary_sensor | On when moisture is below the alert level |
| Sensor firmware | sensor | diagnostic |
| Bluetooth signal | sensor | dBm, diagnostic |
| WiFi signal | sensor | dBm, diagnostic |
| Last reading | sensor | diagnostic |
| Maintenance mode (stay awake) | switch | config |

Sensors show as **unavailable** if two sleep periods pass without a reading, which makes a dead battery or an out-of-range sensor easy to spot.

### Example automation: water reminder

```yaml
automation:
  - alias: "Plant needs water"
    trigger:
      - platform: state
        entity_id: binary_sensor.plant_sensor_needs_water
        to: "on"
    action:
      - service: notify.notify
        data:
          message: >
            The plant needs watering. Moisture is
            {{ states('sensor.plant_sensor_moisture') }}%.
```

Adjust the entity IDs to match your device name.

## MQTT topics

The default base topic is `flowercare`. You can change it on the Settings page.

| Topic | Direction | Payload |
|---|---|---|
| `flowercare/state` | published, retained | JSON with all readings |
| `flowercare/maintenance` | published, retained | `ON` / `OFF` |
| `flowercare/maintenance/set` | subscribed | `ON` / `OFF` |

Example state payload:

```json
{"temperature":24.3,"moisture":38,"light":5120,"conductivity":412,"battery":97,
 "firmware":"3.2.2","ble_rssi":-71,"wifi_rssi":-58,"needs_water":"OFF",
 "time":"2026-10-31 09:00:12","boot":42}
```

## Web pages

| Page | Purpose |
|---|---|
| `/` | Readings, history, power controls, status, log |
| `/settings` | Device name, hostname, timezone, theme, sensor MAC and scan, moisture alert level, sleep timing, MQTT |
| `/update` | Firmware upload (`.bin` from **Sketch → Export Compiled Binary**) |

## Running more than one sensor

Use one ESP32 per sensor. On each board, set a different **device name**, **hostname** and **base topic** on the Settings page.

## Troubleshooting

- **"could not connect to aa:bb:…":** check that the MAC address is right and that the sensor is in range and has battery. Move the ESP32 closer, or check the Bluetooth signal on the web page. Below about −90 dBm, reads become unreliable.
- **Sketch too big when compiling:** select the **Minimal SPIFFS** partition scheme.
- **The board keeps going to sleep before I can reach it:** power-cycle it to get 15 minutes awake, or turn on maintenance mode in Home Assistant.
- **Starting the WiFi setup again:** use **Settings → Reset WiFi**. The board restarts into the `FlowerCare-Setup` portal.

## Credits

The Flower Care BLE protocol details come from the community reverse-engineering of the MiFlora sensor.
