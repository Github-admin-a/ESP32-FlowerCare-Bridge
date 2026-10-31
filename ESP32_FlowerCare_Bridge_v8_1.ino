/*
  Flower Care Bridge  —  ESP32 + Xiaomi Flower Care (MiFlora) BLE sensor   (v8.1)
  ===========================================================================
  Reads a Xiaomi Flower Care plant sensor over Bluetooth LE and publishes the
  readings to Home Assistant over MQTT (auto-discovery). Same style as the
  433/915 MHz Pool/Weather bridge: WiFiManager setup portal, web page with a
  shared header + tabs, light/dark theme, settings in LittleFS, OTA.

  POWER / SLEEP
    - Power on (or any restart that is not a deep-sleep wake): stays awake for
      "Awake after power-on" minutes (default 15). The web page and OTA work
      during this time, and the sensor is read every "Read interval while
      awake" (default 5 min) - the first read is straight after boot.
    - Then deep sleeps for "Sleep time" minutes (Settings page, default 360).
    - Each timer wake: WiFi + sensor read in parallel, publish to MQTT, back
      to sleep (typically 5-15 s awake, no web page).
    - To reach a sleeping board (OTA, settings) without power-cycling it, turn
      on the "Maintenance mode" switch in Home Assistant (or on the web page).
      The command is retained, so the board picks it up at its next wake and
      stays awake until maintenance mode is turned off again.
    - Web page buttons: Read now, Stay awake longer, Maintenance on/off,
      Sleep now.

  WEB PAGES (http://<ip>/ or http://flowercare.local/)
    /           Plant readings, history, power/sleep controls, status, log
    /settings   Device, MQTT, sensor MAC (with BLE scan), sleep timing
    /update     Firmware upload (OTA)

  MQTT: state JSON at <base>/state (retained, default base "flowercare").
  Entities: Moisture, Temperature, Light, Fertility (conductivity), Battery,
  Needs water (moisture below the alert level - use it in an automation in
  place of the old Blynk notification), plus diagnostics and a Maintenance
  mode switch. Entities go "unavailable" if two sleep periods pass without a
  reading. The old v7 entities (homeassistant/sensor/temperature etc.) are
  removed from Home Assistant once, on the first MQTT connect.

  First boot opens the WiFi portal "FlowerCare-Setup": enter your WiFi, MQTT
  broker / username / password and the sensor's Bluetooth MAC there (or
  later on the Settings page, which can also scan for the sensor). Nothing
  network- or account-specific is hard-coded in this sketch.

  v8.1: no built-in broker address, MQTT username or sensor MAC - all are
        entered in the setup portal / Settings page.

  Libraries: WiFiManager (tzapu) 2.0.x, PubSubClient 2.8, ArduinoJson 7.x
             (BLE, WebServer, HTTPUpdateServer, ArduinoOTA, LittleFS come
             with the ESP32 core 3.x)
  Board: "ESP32 Dev Module"
         Partition Scheme: "Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)"
         (BLE + WiFi + web is too big for the default 1.2 MB app partition)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <time.h>
#include <BLEDevice.h>
#include <esp_sleep.h>

#define FW_VERSION "8.1"

// ------------------------------------------------------------------
//  Types (kept above the first function for the Arduino IDE's
//  auto-generated prototypes)
// ------------------------------------------------------------------
struct Config {
  char deviceName[32]  = "Plant Sensor";
  char hostname[32]    = "flowercare";
  char deviceId[24]    = "";                 // stable HA identity, not tied to MAC
  char mqttHost[64]    = "";                 // set in the portal / Settings
  uint16_t mqttPort    = 1883;
  char mqttUser[32]    = "";
  char mqttPass[64]    = "";
  char baseTopic[48]   = "flowercare";
  char discPrefix[32]  = "homeassistant";
  char floraMac[18]    = "";                 // sensor Bluetooth MAC, set in the portal / Settings
  uint16_t awakeMins   = 15;     // awake after power-on / restart
  uint16_t sleepMins   = 360;    // deep sleep between readings (6 h)
  uint16_t readSecs    = 300;    // read interval while awake
  uint8_t moistureLow  = 15;     // % - "Needs water" below this
  uint8_t theme        = 2;      // 0 = follow device, 1 = light, 2 = dark
  char tz[40]          = "AEST-10";
  uint8_t oldCleaned   = 0;      // v7 HA entities removed
} cfg;

struct Reading {
  bool valid = false;
  float temp = NAN;          // C
  int moisture = -1;         // %
  uint32_t lux = 0;          // lx
  int cond = -1;             // uS/cm
  int batt = -1;             // %
  char fw[8] = "";
  int rssi = 0;              // BLE dBm
  time_t when = 0;
  unsigned long ms = 0;
};

struct Sample {              // history kept in RTC memory (survives deep sleep)
  uint32_t t;
  int16_t temp10;
  uint8_t moist;
  int8_t batt;
  uint32_t lux;
  uint16_t cond;
};

struct ScanHit { char mac[18]; char name[24]; int rssi; };

enum { BLE_IDLE = 0, BLE_BUSY = 1, BLE_OK = 2, BLE_FAIL = 3 };
enum { BLE_READ = 1, BLE_SCAN = 2 };

// ------------------------------------------------------------------
//  RTC memory: kept through deep sleep, cleared on power-on
// ------------------------------------------------------------------
#define HIST_N 24
RTC_DATA_ATTR Sample   hist[HIST_N];
RTC_DATA_ATTR uint8_t  histHead = 0, histCount = 0;
RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR uint32_t readOk = 0;
RTC_DATA_ATTR uint32_t bleFails = 0;     // failed reads in a row
RTC_DATA_ATTR bool     maintenance = false;

// ------------------------------------------------------------------
//  Globals
// ------------------------------------------------------------------
WebServer server(80);
HTTPUpdateServer httpUpdater;
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

static BLEUUID serviceUUID("00001204-0000-1000-8000-00805f9b34fb");
static BLEUUID uuidData("00001a01-0000-1000-8000-00805f9b34fb");
static BLEUUID uuidFirmware("00001a02-0000-1000-8000-00805f9b34fb");
static BLEUUID uuidMode("00001a00-0000-1000-8000-00805f9b34fb");
BLEClient* pClient = nullptr;

Reading cur;                       // latest good reading
Reading bleResult;                 // written by the BLE task
char bleMsg[96] = "";              // error text from the BLE task
volatile uint8_t bleState = BLE_IDLE;
volatile uint8_t bleJob = BLE_READ;
unsigned long bleStartMs = 0;
#define BLE_STUCK_MS 150000UL      // a read taking longer than this is stuck

#define SCAN_MAX 16
ScanHit scanHits[SCAN_MAX];
volatile int scanCount = 0;

String lastError = "";
String resetReason = "Unknown";
bool timerWake = false;
bool otaInProgress = false;
bool maintLocal = false;           // changed on the web page while MQTT was down
unsigned long awakeUntil = 0;
unsigned long nextReadMs = 0;
unsigned long lastMqttAttempt = 0;
bool sleepRequested = false;
unsigned long sleepReqMs = 0;

// ------------------------------------------------------------------
//  Helpers
// ------------------------------------------------------------------
void copyStr(char* dst, size_t n, const char* src) {
  strncpy(dst, src ? src : "", n - 1);
  dst[n - 1] = 0;
}

String topic(const char* suffix) { return String(cfg.baseTopic) + "/" + suffix; }

static float r1(float v) { return roundf(v * 10.0f) / 10.0f; }

static String htmlEsc(const String& s) {
  String o; o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&': o += "&amp;"; break;
      case '<': o += "&lt;"; break;
      case '>': o += "&gt;"; break;
      case '"': o += "&quot;"; break;
      case '\'': o += "&#39;"; break;
      default: o += c;
    }
  }
  return o;
}

// Formats a time, or "--" if the clock has never been set.
static void fmtTime(time_t t, char* out, size_t len, const char* fmt) {
  if (t < 1700000000) { copyStr(out, len, "--"); return; }
  struct tm tmv; localtime_r(&t, &tmv);
  strftime(out, len, fmt, &tmv);
}

String timeStamp() {
  time_t now = time(nullptr);
  char b[20];
  if (now > 1700000000) {
    fmtTime(now, b, sizeof(b), "%H:%M:%S");
  } else {
    unsigned long s = millis() / 1000;
    snprintf(b, sizeof(b), "+%lu:%02lu:%02lu", s / 3600, (s / 60) % 60, s % 60);
  }
  return String(b);
}

String fmtUptime() {
  unsigned long s = millis() / 1000;
  char b[24];
  snprintf(b, sizeof(b), "%lud %02lu:%02lu:%02lu", s / 86400, (s / 3600) % 24, (s / 60) % 60, s % 60);
  return String(b);
}

String resetText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "Power on";
    case ESP_RST_SW:        return "Software restart";
    case ESP_RST_PANIC:     return "Crash (panic)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:       return "Watchdog";
    case ESP_RST_DEEPSLEEP: return "Wake from deep sleep";
    case ESP_RST_BROWNOUT:  return "Brownout (supply voltage dipped)";
    case ESP_RST_EXT:       return "Reset button";
    default:                return "Other (" + String((int)esp_reset_reason()) + ")";
  }
}

// ------------------------------------------------------------------
//  Event log
// ------------------------------------------------------------------
const int LOG_SIZE = 30;
String logBuf[LOG_SIZE];
int logHead = 0, logCount = 0;

void addLog(const String& msg) {
  String line = timeStamp() + "  " + msg;
  Serial.println(line);
  logBuf[logHead] = line;
  logHead = (logHead + 1) % LOG_SIZE;
  if (logCount < LOG_SIZE) logCount++;
}

// ------------------------------------------------------------------
//  Config (LittleFS /config.json)
// ------------------------------------------------------------------
void saveConfig() {
  JsonDocument d;
  d["deviceName"] = cfg.deviceName;  d["hostname"]   = cfg.hostname;
  d["deviceId"]   = cfg.deviceId;    d["mqttHost"]   = cfg.mqttHost;
  d["mqttPort"]   = cfg.mqttPort;    d["mqttUser"]   = cfg.mqttUser;
  d["mqttPass"]   = cfg.mqttPass;    d["baseTopic"]  = cfg.baseTopic;
  d["discPrefix"] = cfg.discPrefix;  d["floraMac"]   = cfg.floraMac;
  d["awakeMins"]  = cfg.awakeMins;   d["sleepMins"]  = cfg.sleepMins;
  d["readSecs"]   = cfg.readSecs;    d["moistureLow"] = cfg.moistureLow;
  d["theme"]      = cfg.theme;       d["tz"]         = cfg.tz;
  d["oldCleaned"] = cfg.oldCleaned;
  File f = LittleFS.open("/config.json", "w");
  if (f) { serializeJson(d, f); f.close(); }
}

void loadConfig() {
  File f = LittleFS.open("/config.json", "r");
  if (f) {
    JsonDocument d;
    if (!deserializeJson(d, f)) {
      copyStr(cfg.deviceName, sizeof(cfg.deviceName), d["deviceName"] | cfg.deviceName);
      copyStr(cfg.hostname,   sizeof(cfg.hostname),   d["hostname"]   | cfg.hostname);
      copyStr(cfg.deviceId,   sizeof(cfg.deviceId),   d["deviceId"]   | "");
      copyStr(cfg.mqttHost,   sizeof(cfg.mqttHost),   d["mqttHost"]   | cfg.mqttHost);
      cfg.mqttPort = d["mqttPort"] | cfg.mqttPort;
      copyStr(cfg.mqttUser,   sizeof(cfg.mqttUser),   d["mqttUser"]   | cfg.mqttUser);
      copyStr(cfg.mqttPass,   sizeof(cfg.mqttPass),   d["mqttPass"]   | "");
      copyStr(cfg.baseTopic,  sizeof(cfg.baseTopic),  d["baseTopic"]  | cfg.baseTopic);
      copyStr(cfg.discPrefix, sizeof(cfg.discPrefix), d["discPrefix"] | cfg.discPrefix);
      copyStr(cfg.floraMac,   sizeof(cfg.floraMac),   d["floraMac"]   | cfg.floraMac);
      cfg.awakeMins   = d["awakeMins"]   | cfg.awakeMins;
      cfg.sleepMins   = d["sleepMins"]   | cfg.sleepMins;
      cfg.readSecs    = d["readSecs"]    | cfg.readSecs;
      cfg.moistureLow = d["moistureLow"] | cfg.moistureLow;
      cfg.theme       = d["theme"]       | cfg.theme;
      copyStr(cfg.tz, sizeof(cfg.tz), d["tz"] | cfg.tz);
      cfg.oldCleaned  = d["oldCleaned"]  | 0;
    }
    f.close();
  }
  if (cfg.deviceId[0] == 0) {
    snprintf(cfg.deviceId, sizeof(cfg.deviceId), "flowercare_%08x", (unsigned)esp_random());
    saveConfig();
  }
  if (cfg.awakeMins < 1) cfg.awakeMins = 1;
  if (cfg.sleepMins < 1) cfg.sleepMins = 1;
  if (cfg.readSecs < 30) cfg.readSecs = 30;
}

// ------------------------------------------------------------------
//  Flower Care over BLE (runs in its own task so the web page keeps
//  working during the few seconds a read takes)
// ------------------------------------------------------------------
static bool readFlora(Reading& r, char* msg, size_t n) {
  r = Reading();
  if (strlen(cfg.floraMac) != 17) { snprintf(msg, n, "no sensor MAC set - see Settings"); return false; }
  BLEAddress addr(cfg.floraMac);
  bool conn = false;
  for (int a = 1; a <= 2 && !conn; a++) {
    Serial.printf("BLE: connecting to %s (try %d)\n", cfg.floraMac, a);
    conn = pClient->connect(addr);
    if (!conn) delay(1500);
  }
  if (!conn) { snprintf(msg, n, "could not connect to %s", cfg.floraMac); return false; }

  bool ok = false;
  do {
    BLERemoteService* svc = pClient->getService(serviceUUID);
    if (!svc) { snprintf(msg, n, "Flower Care service not found"); break; }

    // Put the sensor into real-time data mode
    BLERemoteCharacteristic* ch = svc->getCharacteristic(uuidMode);
    if (!ch) { snprintf(msg, n, "mode characteristic not found"); break; }
    uint8_t mode[2] = {0xA0, 0x1F};
    ch->writeValue(mode, 2, true);

    // Battery + firmware: [batt] [?] [firmware "x.y.z"]
    ch = svc->getCharacteristic(uuidFirmware);
    if (ch) {
      auto v = ch->readValue();
      const uint8_t* b = (const uint8_t*)v.c_str();
      if (v.length() >= 7) {
        r.batt = b[0];
        memcpy(r.fw, b + 2, 5);
        r.fw[5] = 0;
      }
    }

    // Data: temp(int16 LE, 0.1 C) [?] light(uint32 LE, lux) moisture(%) conductivity(uint16 LE)
    ch = svc->getCharacteristic(uuidData);
    if (!ch) { snprintf(msg, n, "data characteristic not found"); break; }
    auto v = ch->readValue();
    if (v.length() < 10) { snprintf(msg, n, "short data (%d bytes)", (int)v.length()); break; }
    const uint8_t* b = (const uint8_t*)v.c_str();
    int16_t t10 = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
    r.temp     = t10 / 10.0f;
    r.lux      = (uint32_t)b[3] | ((uint32_t)b[4] << 8) | ((uint32_t)b[5] << 16) | ((uint32_t)b[6] << 24);
    r.moisture = b[7];
    r.cond     = b[8] | (b[9] << 8);
    if (r.moisture > 100 || r.temp < -30 || r.temp > 70) {
      snprintf(msg, n, "implausible data (temp %.1f, moisture %d)", r.temp, r.moisture);
      break;
    }
    r.rssi = pClient->getRssi();
    ok = true;
  } while (false);

  pClient->disconnect();
  for (int i = 0; i < 20 && pClient->isConnected(); i++) delay(100);
  return ok;
}

class ScanCb : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) {
    if (!d.haveName()) return;
    String mac = d.getAddress().toString().c_str();
    for (int i = 0; i < scanCount; i++) {
      if (mac.equalsIgnoreCase(scanHits[i].mac)) { scanHits[i].rssi = d.getRSSI(); return; }
    }
    if (scanCount >= SCAN_MAX) return;
    ScanHit& h = scanHits[scanCount];
    copyStr(h.mac, sizeof(h.mac), mac.c_str());
    copyStr(h.name, sizeof(h.name), d.getName().c_str());
    h.rssi = d.getRSSI();
    scanCount = scanCount + 1;
  }
};

static void doScan() {
  static ScanCb cb;
  scanCount = 0;
  BLEScan* s = BLEDevice::getScan();
  s->setAdvertisedDeviceCallbacks(&cb, true);
  s->setActiveScan(true);
  s->setInterval(100);
  s->setWindow(99);
  s->start(6, false);
  s->clearResults();
}

void bleTask(void*) {
  bool ok = true;
  if (bleJob == BLE_SCAN) doScan();
  else ok = readFlora(bleResult, bleMsg, sizeof(bleMsg));
  bleState = ok ? BLE_OK : BLE_FAIL;
  vTaskDelete(nullptr);
}

bool startBle(uint8_t jobType) {
  if (bleState != BLE_IDLE) return false;
  bleJob = jobType;
  bleMsg[0] = 0;
  bleState = BLE_BUSY;
  bleStartMs = millis();
  if (xTaskCreate(bleTask, "ble", 8192, nullptr, 1, nullptr) != pdPASS) {
    bleState = BLE_IDLE;
    addLog("Could not start BLE task");
    return false;
  }
  return true;
}

void addHistory(const Reading& r) {
  Sample& s = hist[histHead];
  s.t      = (uint32_t)r.when;
  s.temp10 = (int16_t)lroundf(r.temp * 10.0f);
  s.moist  = (uint8_t)r.moisture;
  s.batt   = (int8_t)r.batt;
  s.lux    = r.lux;
  s.cond   = (uint16_t)r.cond;
  histHead = (histHead + 1) % HIST_N;
  if (histCount < HIST_N) histCount++;
}

void publishReading();

// Called from loop() (and once in a wake cycle) to pick up the BLE task's result.
void processBle() {
  if (bleState == BLE_BUSY) {
    if (millis() - bleStartMs > BLE_STUCK_MS) {
      addLog("Sensor read stuck - restarting");
      delay(300);
      ESP.restart();
    }
    return;
  }
  if (bleState == BLE_IDLE) return;
  bool ok = (bleState == BLE_OK);
  uint8_t jobType = bleJob;
  bleState = BLE_IDLE;

  if (jobType == BLE_SCAN) {
    addLog("BLE scan finished: " + String(scanCount) + " named device(s)");
    return;
  }
  if (ok) {
    cur = bleResult;
    cur.valid = true;
    cur.when = time(nullptr);
    cur.ms = millis();
    readOk++;
    bleFails = 0;
    lastError = "";
    addHistory(cur);
    addLog("Sensor: moisture " + String(cur.moisture) + " %, " + String(cur.temp, 1) + " C, " +
           String(cur.lux) + " lx, " + String(cur.cond) + " uS/cm, battery " + String(cur.batt) +
           " %, BLE " + String(cur.rssi) + " dBm");
    publishReading();
  } else {
    bleFails++;
    lastError = bleMsg;
    addLog("Sensor read failed: " + lastError);
  }
}

// ------------------------------------------------------------------
//  MQTT + Home Assistant discovery
// ------------------------------------------------------------------
uint32_t expireSecs() { return (uint32_t)cfg.sleepMins * 120UL + 900UL; }   // two sleeps + 15 min

void addDevice(JsonDocument& d) {
  JsonObject dev = d["dev"].to<JsonObject>();
  dev["ids"][0] = cfg.deviceId;
  dev["name"]   = cfg.deviceName;
  dev["mdl"]    = "Xiaomi Flower Care BLE bridge";
  dev["mf"]     = "DIY";
  dev["sw"]     = FW_VERSION;
  if (WiFi.status() == WL_CONNECTED) dev["cu"] = "http://" + WiFi.localIP().toString() + "/";
}

void pubDiscovery(const char* comp, const char* key, JsonDocument& d) {
  addDevice(d);
  String t = String(cfg.discPrefix) + "/" + comp + "/" + cfg.deviceId + "/" + key + "/config";
  String payload; serializeJson(d, payload);
  mqtt.publish(t.c_str(), (const uint8_t*)payload.c_str(), payload.length(), true);
}

void discEntity(const char* comp, const char* key, const char* name, const char* tpl, const char* unit,
                const char* devClass, const char* stateClass, const char* icon, bool diag) {
  JsonDocument d;
  d["name"]    = name;
  d["uniq_id"] = String(cfg.deviceId) + "_" + key;
  d["stat_t"]  = topic("state");
  d["val_tpl"] = tpl;
  if (unit)       d["unit_of_meas"] = unit;
  if (devClass)   d["dev_cla"]      = devClass;
  if (stateClass) d["stat_cla"]     = stateClass;
  if (icon)       d["icon"]         = icon;
  if (diag)       d["ent_cat"]      = "diagnostic";
  d["exp_aft"] = expireSecs();
  pubDiscovery(comp, key, d);
}

void publishDiscovery() {
  discEntity("sensor", "moisture", "Moisture", "{{ value_json.moisture }}", "%", "moisture", "measurement", nullptr, false);
  discEntity("sensor", "temperature", "Temperature", "{{ value_json.temperature }}", "\xC2\xB0""C", "temperature", "measurement", nullptr, false);
  discEntity("sensor", "light", "Light", "{{ value_json.light }}", "lx", "illuminance", "measurement", nullptr, false);
  discEntity("sensor", "conductivity", "Fertility", "{{ value_json.conductivity }}", "\xC2\xB5S/cm", nullptr, "measurement", "mdi:flower", false);
  discEntity("sensor", "battery", "Battery", "{{ value_json.battery }}", "%", "battery", "measurement", nullptr, false);
  discEntity("binary_sensor", "needs_water", "Needs water", "{{ value_json.needs_water }}", nullptr, "problem", nullptr, "mdi:watering-can", false);
  discEntity("sensor", "sensor_fw", "Sensor firmware", "{{ value_json.firmware }}", nullptr, nullptr, nullptr, "mdi:chip", true);
  discEntity("sensor", "ble_rssi", "Bluetooth signal", "{{ value_json.ble_rssi }}", "dBm", "signal_strength", "measurement", nullptr, true);
  discEntity("sensor", "wifi_rssi", "WiFi signal", "{{ value_json.wifi_rssi }}", "dBm", "signal_strength", "measurement", nullptr, true);
  discEntity("sensor", "last_reading", "Last reading", "{{ value_json.time }}", nullptr, nullptr, nullptr, "mdi:clock-outline", true);
  {
    JsonDocument d;
    d["name"]    = "Maintenance mode (stay awake)";
    d["uniq_id"] = String(cfg.deviceId) + "_maintenance";
    d["cmd_t"]   = topic("maintenance/set");
    d["stat_t"]  = topic("maintenance");
    d["ret"]     = true;          // retained: a sleeping board gets it at its next wake
    d["icon"]    = "mdi:wrench-clock";
    d["ent_cat"] = "config";
    pubDiscovery("switch", "maintenance", d);
  }
}

// v7 published generic, retained topics with no unique_id - remove them once.
void cleanupOldEntities() {
  const char* const OLD[] = {"temperature", "moisture", "light", "conductivity", "battery", "firmware"};
  for (const char* k : OLD) {
    mqtt.publish((String("homeassistant/sensor/") + k + "/config").c_str(), "", true);
    mqtt.publish((String("homeassistant/sensor/") + k).c_str(), "", true);
  }
  cfg.oldCleaned = 1;
  saveConfig();
  addLog("Removed the old v7 Home Assistant entities");
}

void publishReading() {
  if (!mqtt.connected() || !cur.valid) return;
  JsonDocument d;
  d["temperature"]  = r1(cur.temp);
  d["moisture"]     = cur.moisture;
  d["light"]        = cur.lux;
  d["conductivity"] = cur.cond;
  if (cur.batt >= 0) d["battery"] = cur.batt; else d["battery"] = nullptr;
  d["firmware"]     = cur.fw;
  d["ble_rssi"]     = cur.rssi;
  d["wifi_rssi"]    = WiFi.RSSI();
  d["needs_water"]  = cur.moisture < cfg.moistureLow ? "ON" : "OFF";
  char ts[24]; fmtTime(cur.when, ts, sizeof(ts), "%Y-%m-%d %H:%M:%S");
  d["time"]         = ts;
  d["boot"]         = bootCount;
  String s; serializeJson(d, s);
  mqtt.publish(topic("state").c_str(), s.c_str(), true);
}

void publishMaintState() {
  if (mqtt.connected()) mqtt.publish(topic("maintenance").c_str(), maintenance ? "ON" : "OFF", true);
}

void setMaintenance(bool on, bool fromMqtt) {
  if (on == maintenance) { if (fromMqtt) publishMaintState(); return; }
  maintenance = on;
  addLog(on ? "Maintenance mode ON - staying awake" : "Maintenance mode OFF");
  if (!on) {                       // give at least a minute before sleeping
    unsigned long minUntil = millis() + 60000UL;
    if ((long)(awakeUntil - minUntil) < 0) awakeUntil = minUntil;
  }
  if (!fromMqtt) {
    if (mqtt.connected()) mqtt.publish(topic("maintenance/set").c_str(), on ? "ON" : "OFF", true);
    else maintLocal = true;        // push it to the broker on the next connect
  }
  publishMaintState();
}

void onMqtt(char* t, byte* payload, unsigned int len) {
  String msg; msg.reserve(len);
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim();
  if (String(t) == topic("maintenance/set")) {
    if (msg.equalsIgnoreCase("ON")) setMaintenance(true, true);
    else if (msg.equalsIgnoreCase("OFF")) setMaintenance(false, true);
  }
}

bool mqttConnect() {
  if (cfg.mqttHost[0] == 0) return false;
  mqtt.setServer(cfg.mqttHost, cfg.mqttPort);
  bool ok = mqtt.connect(cfg.deviceId,
                         cfg.mqttUser[0] ? cfg.mqttUser : nullptr,
                         cfg.mqttUser[0] ? cfg.mqttPass : nullptr);
  if (!ok) { addLog("MQTT connect failed, rc=" + String(mqtt.state())); return false; }
  addLog(String("MQTT connected to ") + cfg.mqttHost);
  if (maintLocal) {                // web page change made while disconnected wins
    mqtt.publish(topic("maintenance/set").c_str(), maintenance ? "ON" : "OFF", true);
    maintLocal = false;
  }
  mqtt.subscribe(topic("maintenance/set").c_str());
  // give the broker a moment to deliver a retained maintenance command
  unsigned long t0 = millis();
  while (millis() - t0 < 800) { mqtt.loop(); delay(10); }
  if (!cfg.oldCleaned) cleanupOldEntities();
  publishDiscovery();
  publishMaintState();
  publishReading();
  return true;
}

void handleMqtt() {
  if (cfg.mqttHost[0] == 0) return;
  if (mqtt.connected()) { mqtt.loop(); return; }
  if (millis() - lastMqttAttempt < 10000) return;
  lastMqttAttempt = millis();
  mqttConnect();
}

// ------------------------------------------------------------------
//  Deep sleep
// ------------------------------------------------------------------
void goToSleep(const char* why) {
  Serial.printf("Deep sleep for %u min (%s)\n", cfg.sleepMins, why);
  if (mqtt.connected()) { mqtt.loop(); mqtt.disconnect(); }
  delay(300);                      // let the last packets leave
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_sleep_enable_timer_wakeup((uint64_t)cfg.sleepMins * 60ULL * 1000000ULL);
  Serial.flush();
  esp_deep_sleep_start();
}

// Timer wake: read + publish, then back to sleep. Returns true only if
// maintenance mode was switched on (then the board stays awake).
bool wakeCycle() {
  Serial.println("Timer wake: reading sensor");
  WiFi.setHostname(cfg.hostname);
  WiFi.mode(WIFI_STA);
  WiFi.begin();                    // credentials saved by WiFiManager
  unsigned long t0 = millis();
  startBle(BLE_READ);              // BLE and WiFi connect at the same time
  while (bleState == BLE_BUSY && millis() - t0 < BLE_STUCK_MS) delay(50);
  if (bleState == BLE_BUSY) { Serial.println("Sensor read stuck"); return false; }
  processBle();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) delay(50);
  if (WiFi.status() != WL_CONNECTED) { Serial.println("WiFi did not connect"); return false; }
  configTzTime(cfg.tz, "pool.ntp.org", "time.google.com");
  if (mqttConnect()) mqtt.loop();
  return maintenance;
}

// ------------------------------------------------------------------
//  Web pages
// ------------------------------------------------------------------
const char CSS[] PROGMEM = R"CSS(
<meta name=viewport content="width=device-width,initial-scale=1">
<style>
:root{--bg:#0f1720;--card:#18222e;--fg:#e6edf3;--mut:#8b9bb0;--acc:#2f81f7;--ok:#3fb950;--bad:#f85149;--btn:#243244;--line:#33465c;--inset:#0b1118;--hdr:#132033;--hdrline:#22324a;--shadow:none;color-scheme:dark}
:root[data-theme=light]{--bg:#eef2f6;--card:#ffffff;--fg:#1b2430;--mut:#5b6878;--acc:#1f6feb;--ok:#1a7f37;--bad:#cf222e;--btn:#eef2f7;--line:#c8d1dc;--inset:#f5f7fa;--hdr:#ffffff;--hdrline:#d6dde6;--shadow:0 1px 3px rgba(20,30,45,.12);color-scheme:light}
@media (prefers-color-scheme:light){:root[data-theme=auto]{--bg:#eef2f6;--card:#ffffff;--fg:#1b2430;--mut:#5b6878;--acc:#1f6feb;--ok:#1a7f37;--bad:#cf222e;--btn:#eef2f7;--line:#c8d1dc;--inset:#f5f7fa;--hdr:#ffffff;--hdrline:#d6dde6;--shadow:0 1px 3px rgba(20,30,45,.12);color-scheme:light}}
body{margin:0;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif}
header.top{background:var(--hdr);border-bottom:1px solid var(--hdrline);margin-bottom:14px;text-align:center}
header.top .wrap{padding:18px 16px 14px}
.sub{color:var(--mut);font-size:14px;margin-top:6px}
nav.tabs{display:flex;flex-wrap:wrap;justify-content:center;gap:6px;margin:0 0 14px}
nav.tabs a{color:var(--fg);text-decoration:none;padding:7px 12px;border-radius:8px;background:var(--card);box-shadow:var(--shadow);font-size:14px}
nav.tabs a:hover{background:var(--btn)}nav.tabs a.on{background:var(--acc);color:#fff}
.wrap{max-width:760px;margin:auto;padding:0 16px 16px}
h1{font-size:28px;margin:0;letter-spacing:.01em}h2{font-size:15px;margin:0 0 10px;color:var(--mut);text-transform:uppercase;letter-spacing:.05em}
.card{background:var(--card);border-radius:10px;padding:14px;margin-bottom:14px;box-shadow:var(--shadow)}
.tiles{display:grid;grid-template-columns:repeat(auto-fill,minmax(130px,1fr));gap:8px;margin-bottom:10px}
.tile{background:var(--inset);border-radius:8px;padding:10px}.tl{color:var(--mut);font-size:12.5px}.tv{font-size:22px;margin-top:4px}
button{background:var(--btn);color:var(--fg);border:1px solid var(--line);border-radius:8px;padding:10px;font-size:14px;cursor:pointer}
button:hover{border-color:var(--acc)}button.pri{background:var(--acc);border-color:var(--acc);color:#fff}
.now{font-size:18px}
.row{display:flex;gap:10px;flex-wrap:wrap}.row>*{flex:1}
table{width:100%;border-collapse:collapse}td{padding:4px 0;vertical-align:top}td:first-child{color:var(--mut);width:40%}
.ht th,.ht td,.ht td:first-child{width:auto;padding:4px 12px 4px 0;white-space:nowrap;text-align:left;font-weight:normal}
.ht th{color:var(--mut)}.ht td{color:var(--fg)}
pre{background:var(--inset);border-radius:8px;padding:10px;font-size:12.5px;max-height:340px;overflow:auto;white-space:pre-wrap;margin:0}
input,select{width:100%;box-sizing:border-box;background:var(--inset);color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:8px;font-size:14px}
input[type=submit]{width:auto;background:var(--acc);border:1px solid var(--acc);color:#fff;border-radius:8px;padding:10px 16px;margin-top:14px;cursor:pointer}
input[type=file]{padding:6px}
label{display:block;margin:10px 0 4px;color:var(--mut);font-size:13px}
a{color:var(--acc)}
.ok{color:var(--ok)}.bad{color:var(--bad)}.note{color:var(--mut);font-size:13px}
</style>)CSS";

const char INDEX_HTML[] PROGMEM = R"HTML(
<div class=card><h2 id=pname>Plant</h2>
<div class=tiles>
<div class=tile><div class=tl>Moisture</div><div class=tv id=vm>-</div></div>
<div class=tile><div class=tl>Temperature</div><div class=tv id=vt>-</div></div>
<div class=tile><div class=tl>Light</div><div class=tv id=vl>-</div></div>
<div class=tile><div class=tl>Fertility</div><div class=tv id=vc>-</div></div>
<div class=tile><div class=tl>Battery</div><div class=tv id=vb>-</div></div>
</div><p class=note id=vage></p>
<div class=row><button class=pri onclick="api('/api/read')">Read sensor now</button></div></div>
<div class=card><h2>Power</h2><div class=now id=pw>-</div><p class=note id=pwn></p>
<div class=row><button onclick="api('/api/extend')">Stay awake longer</button>
<button id=mb onclick="api('/api/maint?on='+(maint?0:1))">Maintenance on</button>
<button onclick="if(confirm('Go to deep sleep now? The page stops responding until the board wakes in maintenance mode or is power-cycled.'))api('/api/sleep')">Sleep now</button></div>
<p class=note>Maintenance mode keeps the board awake (for OTA or settings) until it is turned off. Switched on from Home Assistant while the board sleeps, it takes effect at the next wake.</p></div>
<div class=card><h2>History</h2><div style="overflow:auto"><table class=ht id=hist></table></div></div>
<div class=card><h2>Status</h2><table id=st></table></div>
<div class=card><h2>Log</h2><pre id=log></pre></div>
<script>
let maint=false;
function e(t){return String(t).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}
function api(u){fetch(u,{method:'POST'}).then(load)}
function fmt(s){s=Math.max(0,s|0);const m=Math.floor(s/60),x=s%60;return m>=60?Math.floor(m/60)+' h '+(m%60)+' min':m+':'+String(x).padStart(2,'0')}
function load(){fetch('/api/status').then(r=>r.json()).then(s=>{
 pname.textContent=s.name;maint=s.maint;const c=s.cur;
 if(c){vm.innerHTML=c.moist+' %'+(c.low?' <span class=bad>water</span>':'');vt.textContent=c.temp.toFixed(1)+' \u00B0C';
  vl.textContent=c.lux+' lx';vc.textContent=c.cond+' \u00B5S/cm';vb.textContent=c.batt==null?'-':c.batt+' %';
  vage.textContent='Read at '+c.time+' ('+fmt(c.age)+' ago), Bluetooth '+c.rssi+' dBm, sensor firmware '+c.fwv}
 else vage.textContent='No reading yet';
 if(s.busy)vage.textContent+=' \u2013 reading sensor now...';
 pw.textContent=s.maint?'Maintenance mode \u2013 staying awake':'Deep sleep in '+fmt(s.sleep_in);
 pwn.textContent='Then sleeps '+fmt(s.sleep_mins*60)+' between readings.'+(s.busy?'':' Next reading while awake in '+fmt(s.next_read)+'.');
 mb.textContent=s.maint?'Maintenance off':'Maintenance on';mb.classList.toggle('pri',s.maint);
 hist.innerHTML='<tr><th>Time</th><th>Moisture</th><th>Temp</th><th>Light</th><th>Fertility</th><th>Battery</th></tr>'+
  (s.hist.length?s.hist.map(h=>`<tr><td>${h[0]}</td><td>${h[1]} %</td><td>${h[2]} \u00B0C</td><td>${h[3]} lx</td><td>${h[4]} \u00B5S/cm</td><td>${h[5]<0?'-':h[5]+' %'}</td></tr>`).join(''):'<tr><td colspan=6>No readings yet</td></tr>');
 const rows=[['Sensor',s.mac?e(s.mac):'<span class=bad>no MAC set - see Settings</span>'],['Readings OK / failed in a row',s.reads+' / '+(s.fails?'<span class=bad>'+s.fails+'</span>':0)],
  ['Last sensor error',s.err?'<span class=bad>'+e(s.err)+'</span>':'-'],
  ['MQTT',s.mqtt?'<span class=ok>connected</span>':(s.mqtt_host?'<span class=bad>disconnected</span>':'not configured')],
  ['WiFi',e(s.ssid)+' ('+s.rssi+' dBm)'],['IP',s.ip],['Uptime',s.uptime],['Started by',e(s.restart)],['Boots + wakes',s.boots],['Firmware','v'+s.fw]];
 st.innerHTML=rows.map(r=>`<tr><td>${r[0]}</td><td>${r[1]}</td></tr>`).join('');
 log.textContent=s.log.join('\n');
})}
load();setInterval(load,2000);
</script>)HTML";

const char SCAN_JS[] PROGMEM = R"HTML(
<script>
function scan(){sres.textContent='Scanning for 6 s...';
 fetch('/api/scan',{method:'POST'}).then(r=>{if(!r.ok){sres.textContent='Sensor is busy - try again in a few seconds';return}setTimeout(poll,6500)})}
function poll(){fetch('/api/scanresult').then(r=>r.json()).then(s=>{
 if(s.busy){setTimeout(poll,1000);return}
 if(!s.hits.length){sres.textContent='Nothing found - is the sensor within range?';return}
 s.hits.sort((a,b)=>(b.flora-a.flora)||(b.rssi-a.rssi));
 sres.innerHTML=s.hits.map(h=>`<div><a href=# onclick="fmac.value='${h.mac}';return false">${h.mac}</a> ${h.name.replace(/</g,'&lt;')} (${h.rssi} dBm)${h.flora?' <b>Flower Care</b>':''}</div>`).join('')+
  '<div>Click an address to use it, then Save.</div>'})}
</script>)HTML";

const char* const TABS[][2] = { {"/", "Plant"}, {"/settings", "Settings"}, {"/update", "Firmware"} };

String pageStart(const char* title, const char* activeTab) {
  static const char* const THEME_NAMES[] = {"auto", "light", "dark"};
  String h; h.reserve(5000);
  h += F("<!doctype html><html data-theme=");
  h += THEME_NAMES[cfg.theme > 2 ? 2 : cfg.theme];
  h += F("><head><meta charset=utf-8><title>");
  h += title;
  h += F(" - Flower Care</title>");
  h += FPSTR(CSS);
  h += F("</head><body><header class=top><div class=wrap><h1>Flower Care Sensor</h1><div class=sub>");
  h += htmlEsc(cfg.deviceName);
  h += F(" &middot; Bluetooth to MQTT bridge</div></div></header><div class=wrap><nav class=tabs>");
  for (auto& t : TABS)
    h += "<a href=\"" + String(t[0]) + "\"" + (strcmp(t[0], activeTab) == 0 ? " class=on" : "") + ">" + t[1] + "</a>";
  h += F("</nav>");
  return h;
}

String pageEnd() { return F("</div></body></html>"); }

void handleStatus() {
  JsonDocument d;
  d["name"]      = cfg.deviceName;
  d["fw"]        = FW_VERSION;
  d["mqtt"]      = mqtt.connected();
  d["mqtt_host"] = cfg.mqttHost;
  d["ssid"]      = WiFi.SSID();
  d["rssi"]      = WiFi.RSSI();
  d["ip"]        = WiFi.localIP().toString();
  d["uptime"]    = fmtUptime();
  d["boots"]     = bootCount;
  d["restart"]   = resetReason;
  d["mac"]       = cfg.floraMac;
  d["maint"]     = maintenance;
  long left = (long)(awakeUntil - millis());
  d["sleep_in"]  = maintenance ? -1L : (left > 0 ? left / 1000 : 0L);
  d["sleep_mins"] = cfg.sleepMins;
  long nr = (long)(nextReadMs - millis());
  d["next_read"] = nr > 0 ? nr / 1000 : 0L;
  d["busy"]      = bleState == BLE_BUSY && bleJob == BLE_READ;
  d["reads"]     = readOk;
  d["fails"]     = bleFails;
  d["err"]       = lastError;
  if (cur.valid) {
    JsonObject c = d["cur"].to<JsonObject>();
    c["temp"]  = r1(cur.temp);
    c["moist"] = cur.moisture;
    c["low"]   = cur.moisture < cfg.moistureLow;
    c["lux"]   = cur.lux;
    c["cond"]  = cur.cond;
    if (cur.batt >= 0) c["batt"] = cur.batt; else c["batt"] = nullptr;
    c["fwv"]   = cur.fw;
    c["rssi"]  = cur.rssi;
    c["age"]   = (millis() - cur.ms) / 1000;
    char ts[16]; fmtTime(cur.when, ts, sizeof(ts), "%H:%M:%S");
    c["time"]  = ts;
  }
  JsonArray h = d["hist"].to<JsonArray>();
  for (int i = 0; i < histCount; i++) {
    const Sample& s = hist[(histHead - 1 - i + HIST_N) % HIST_N];
    JsonArray row = h.add<JsonArray>();
    char ts[16]; fmtTime((time_t)s.t, ts, sizeof(ts), "%d/%m %H:%M");
    row.add(ts);
    row.add(s.moist);
    row.add(s.temp10 / 10.0f);
    row.add(s.lux);
    row.add(s.cond);
    row.add(s.batt);
  }
  JsonArray l = d["log"].to<JsonArray>();
  for (int i = 0; i < logCount; i++) l.add(logBuf[(logHead - 1 - i + LOG_SIZE) % LOG_SIZE]);
  String out; serializeJson(d, out);
  server.send(200, "application/json", out);
}

String field(const char* label, const char* name, const String& val, const char* type) {
  return String("<label>") + label + "</label><input type=" + type + " name=" + name +
         " value=\"" + htmlEsc(val) + "\">";
}

String pair(const String& a, const String& b) {
  return "<div class=row><div>" + a + "</div><div>" + b + "</div></div>";
}

void handleSettingsGet() {
  String h = pageStart("Settings", "/settings");
  h += F("<form method=post action=/settings>");

  h += F("<div class=card><h2>Device</h2>");
  h += field("Device name (shown in Home Assistant)", "deviceName", cfg.deviceName, "text");
  h += field("Hostname (for OTA / mDNS)", "hostname", cfg.hostname, "text");
  h += field("Timezone (POSIX string, e.g. AEST-10 for Queensland)", "tz", cfg.tz, "text");
  {
    static const char* const THEME_LABELS[] = {"Follow device (light/dark)", "Light", "Dark"};
    h += F("<label>Page theme</label><select name=theme onchange=\"fetch('/api/theme?t='+this.value,{method:'POST'});"
           "document.documentElement.dataset.theme=['auto','light','dark'][this.value]\">");
    for (int i = 0; i < 3; i++)
      h += "<option value=" + String(i) + (cfg.theme == i ? " selected>" : ">") + THEME_LABELS[i] + "</option>";
    h += F("</select><p class=note>The theme changes straight away and is saved - no restart needed.</p>");
  }
  h += field("Home Assistant device ID (keep the same when swapping boards)", "deviceId", cfg.deviceId, "text");
  h += F("</div>");

  h += F("<div class=card><h2>Flower Care sensor</h2>");
  h += F("<label>Sensor Bluetooth MAC address</label><input type=text name=floraMac id=fmac value=\"");
  h += htmlEsc(cfg.floraMac);
  h += F("\"><br><br><button type=button onclick=scan()>Scan for sensors</button><div id=sres class=note></div>");
  h += field("\"Needs water\" below this moisture (%)", "moistureLow", String(cfg.moistureLow), "number");
  h += F("</div>");

  h += F("<div class=card><h2>Sleep</h2>");
  h += pair(field("Awake after power-on (minutes)", "awakeMins", String(cfg.awakeMins), "number"),
            field("Deep sleep between readings (minutes)", "sleepMins", String(cfg.sleepMins), "number"));
  h += field("Read interval while awake (seconds)", "readSecs", String(cfg.readSecs), "number");
  h += F("<p class=note>After power-on the board stays awake for the first time, reading the sensor at the "
         "read interval, then wakes every \"deep sleep\" minutes, reads once and sleeps again. "
         "360 minutes = 6 hours, 1440 = a day.</p></div>");

  h += F("<div class=card><h2>MQTT</h2>");
  h += field("Broker host / IP (blank = MQTT off)", "mqttHost", cfg.mqttHost, "text");
  h += pair(field("Port", "mqttPort", String(cfg.mqttPort), "number"),
            field("Username", "mqttUser", cfg.mqttUser, "text"));
  h += field("Password (leave blank to keep the current one)", "mqttPass", "", "password");
  h += pair(field("Base topic", "baseTopic", cfg.baseTopic, "text"),
            field("Discovery prefix", "discPrefix", cfg.discPrefix, "text"));
  h += F("</div><button class=pri type=submit>Save &amp; restart</button></form><br>");

  h += F("<div class=card><h2>WiFi</h2><p class=note>Clears saved WiFi and restarts into the "
         "<b>FlowerCare-Setup</b> access point.</p>"
         "<form method=post action=/wifireset onsubmit=\"return confirm('Forget WiFi and restart?')\">"
         "<button>Reset WiFi</button></form></div>"
         "<form method=post action=/reboot><button>Restart device</button></form>");
  h += FPSTR(SCAN_JS);
  h += pageEnd();
  server.send(200, "text/html; charset=utf-8", h);
}

uint16_t argU16(const char* n, uint16_t lo, uint16_t hi, uint16_t def) {
  if (server.arg(n).length() == 0) return def;
  return (uint16_t)constrain(server.arg(n).toInt(), (long)lo, (long)hi);
}

void handleSettingsPost() {
  auto arg = [](const char* n) { return server.arg(n); };
  if (arg("deviceName").length()) copyStr(cfg.deviceName, sizeof(cfg.deviceName), arg("deviceName").c_str());
  if (arg("hostname").length())   copyStr(cfg.hostname,   sizeof(cfg.hostname),   arg("hostname").c_str());
  if (arg("tz").length())         copyStr(cfg.tz,         sizeof(cfg.tz),         arg("tz").c_str());
  if (arg("deviceId").length())   copyStr(cfg.deviceId,   sizeof(cfg.deviceId),   arg("deviceId").c_str());
  String mac = arg("floraMac"); mac.trim(); mac.toLowerCase();
  if (mac.length() == 17) copyStr(cfg.floraMac, sizeof(cfg.floraMac), mac.c_str());
  cfg.moistureLow = argU16("moistureLow", 0, 100, cfg.moistureLow);
  cfg.awakeMins   = argU16("awakeMins", 1, 240, cfg.awakeMins);
  cfg.sleepMins   = argU16("sleepMins", 1, 10080, cfg.sleepMins);
  cfg.readSecs    = argU16("readSecs", 30, 3600, cfg.readSecs);
  cfg.theme       = argU16("theme", 0, 2, cfg.theme);
  copyStr(cfg.mqttHost, sizeof(cfg.mqttHost), arg("mqttHost").c_str());
  cfg.mqttPort = argU16("mqttPort", 1, 65535, 1883);
  copyStr(cfg.mqttUser, sizeof(cfg.mqttUser), arg("mqttUser").c_str());
  if (arg("mqttPass").length())   copyStr(cfg.mqttPass,   sizeof(cfg.mqttPass),   arg("mqttPass").c_str());
  if (arg("baseTopic").length())  copyStr(cfg.baseTopic,  sizeof(cfg.baseTopic),  arg("baseTopic").c_str());
  if (arg("discPrefix").length()) copyStr(cfg.discPrefix, sizeof(cfg.discPrefix), arg("discPrefix").c_str());
  saveConfig();
  server.send(200, "text/html", F("<meta http-equiv=refresh content='10;url=/'>Saved. Restarting..."));
  delay(500);
  ESP.restart();
}

void setupWeb() {
  server.on("/", HTTP_GET, [] {
    String h = pageStart("Plant", "/");
    h += FPSTR(INDEX_HTML);
    h += pageEnd();
    server.send(200, "text/html; charset=utf-8", h);
  });
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/read", HTTP_POST, [] {
    if (startBle(BLE_READ)) { nextReadMs = millis() + cfg.readSecs * 1000UL; addLog("Reading sensor (web)"); }
    server.send(200, "text/plain", "OK");
  });
  server.on("/api/extend", HTTP_POST, [] {
    awakeUntil = millis() + cfg.awakeMins * 60000UL;
    addLog("Staying awake another " + String(cfg.awakeMins) + " min");
    server.send(200, "text/plain", "OK");
  });
  server.on("/api/maint", HTTP_POST, [] {
    setMaintenance(server.arg("on") == "1", false);
    server.send(200, "text/plain", "OK");
  });
  server.on("/api/sleep", HTTP_POST, [] {
    if (maintenance) setMaintenance(false, false);
    addLog("Sleep requested from the web page");
    sleepRequested = true; sleepReqMs = millis();
    server.send(200, "text/plain", "OK");
  });
  server.on("/api/scan", HTTP_POST, [] {
    if (!startBle(BLE_SCAN)) { server.send(409, "text/plain", "busy"); return; }
    addLog("BLE scan started");
    server.send(200, "text/plain", "OK");
  });
  server.on("/api/scanresult", HTTP_GET, [] {
    JsonDocument d;
    d["busy"] = bleState == BLE_BUSY;
    JsonArray a = d["hits"].to<JsonArray>();
    if (bleState != BLE_BUSY) {
      for (int i = 0; i < scanCount; i++) {
        JsonObject o = a.add<JsonObject>();
        o["mac"] = scanHits[i].mac; o["name"] = scanHits[i].name; o["rssi"] = scanHits[i].rssi;
        o["flora"] = strstr(scanHits[i].name, "lower care") != nullptr || strstr(scanHits[i].name, "Flower") != nullptr;
      }
    }
    String out; serializeJson(d, out);
    server.send(200, "application/json", out);
  });
  server.on("/api/theme", HTTP_POST, [] {
    cfg.theme = (uint8_t)constrain(server.arg("t").toInt(), 0L, 2L);
    saveConfig();
    server.send(200, "text/plain", "OK");
  });
  server.on("/settings", HTTP_GET, handleSettingsGet);
  server.on("/settings", HTTP_POST, handleSettingsPost);
  server.on("/reboot", HTTP_POST, [] {
    server.send(200, "text/html", F("<meta http-equiv=refresh content='10;url=/'>Restarting..."));
    delay(500); ESP.restart();
  });
  server.on("/wifireset", HTTP_POST, [] {
    server.send(200, "text/html", F("WiFi cleared. Connect to the FlowerCare-Setup access point."));
    delay(500);
    WiFiManager wm; wm.resetSettings();
    ESP.restart();
  });
  // Styled firmware page. Registered before the updater so this GET wins;
  // the upload (POST /update) is handled by httpUpdater.
  server.on("/update", HTTP_GET, [] {
    unsigned long minUntil = millis() + 10UL * 60000UL;     // don't fall asleep mid-update
    if ((long)(awakeUntil - minUntil) < 0) awakeUntil = minUntil;
    String h = pageStart("Firmware", "/update");
    h += F("<div class=card><h2>Firmware update</h2><p class=note>Running v" FW_VERSION
           ". In the Arduino IDE use Sketch &gt; Export Compiled Binary, then choose the .bin file here. "
           "The board restarts when the upload finishes (starting a new awake period); settings are kept.</p>"
           "<form method=POST action=/update enctype=multipart/form-data>"
           "<input type=file accept='.bin' name=firmware>"
           "<input type=submit value='Upload &amp; update'></form></div>");
    h += pageEnd();
    server.send(200, "text/html; charset=utf-8", h);
  });
  httpUpdater.setup(&server, "/update");
  server.onNotFound([] { server.send(404, "text/plain", "Not found"); });
  server.begin();
}

// ------------------------------------------------------------------
//  WiFi / OTA
// ------------------------------------------------------------------
bool wmSaveNeeded = false;

void setupWiFi() {
  WiFi.setHostname(cfg.hostname);
  WiFi.mode(WIFI_STA);
  // Note: never call WiFi.setSleep(false) here - with Bluetooth also running
  // the ESP32 requires WiFi modem sleep and will crash without it.

  WiFiManager wm;
  WiFiManagerParameter pHost("mqtt_host", "MQTT broker host / IP", cfg.mqttHost, sizeof(cfg.mqttHost) - 1);
  char portStr[6]; snprintf(portStr, sizeof(portStr), "%u", cfg.mqttPort);
  WiFiManagerParameter pPort("mqtt_port", "MQTT port", portStr, 5);
  WiFiManagerParameter pUser("mqtt_user", "MQTT username", cfg.mqttUser, sizeof(cfg.mqttUser) - 1);
  WiFiManagerParameter pPass("mqtt_pass", "MQTT password", cfg.mqttPass, sizeof(cfg.mqttPass) - 1);
  WiFiManagerParameter pMac("flora_mac", "Flower Care MAC (aa:bb:cc:dd:ee:ff)", cfg.floraMac, sizeof(cfg.floraMac) - 1);
  wm.addParameter(&pHost); wm.addParameter(&pPort);
  wm.addParameter(&pUser); wm.addParameter(&pPass);
  wm.addParameter(&pMac);
  wm.setSaveParamsCallback([] { wmSaveNeeded = true; });
  wm.setSaveConfigCallback([] { wmSaveNeeded = true; });
  wm.setConfigPortalTimeout(180);
  wm.setConnectTimeout(20);

  addLog("Connecting to WiFi...");
  if (!wm.autoConnect("FlowerCare-Setup")) {
    addLog("WiFi not connected (setup portal timed out) - carrying on offline");
    return;
  }
  if (wmSaveNeeded) {
    copyStr(cfg.mqttHost, sizeof(cfg.mqttHost), pHost.getValue());
    cfg.mqttPort = atoi(pPort.getValue()) > 0 ? atoi(pPort.getValue()) : 1883;
    copyStr(cfg.mqttUser, sizeof(cfg.mqttUser), pUser.getValue());
    copyStr(cfg.mqttPass, sizeof(cfg.mqttPass), pPass.getValue());
    String mac = pMac.getValue(); mac.trim(); mac.toLowerCase();
    if (mac.length() == 17) copyStr(cfg.floraMac, sizeof(cfg.floraMac), mac.c_str());
    saveConfig();
  }
}

void setupOTA() {
  ArduinoOTA.setHostname(cfg.hostname);
  // no OTA password: the Arduino IDE uploads over the network without asking
  ArduinoOTA.onStart([] { otaInProgress = true; Serial.println("OTA start"); });
  ArduinoOTA.onError([](ota_error_t e) { otaInProgress = false; Serial.printf("OTA error %u\n", e); });
  ArduinoOTA.begin();          // also starts mDNS: http://<hostname>.local/
  MDNS.addService("http", "tcp", 80);
}

bool wifiMaintain() {
  static unsigned long lastRetry = 0;
  if (WiFi.status() == WL_CONNECTED) return true;
  if (millis() - lastRetry > 30000UL) { lastRetry = millis(); WiFi.reconnect(); }
  return false;
}

// Full awake mode: web page, OTA, periodic reads, sleep when the time is up.
void startAwake() {
  if (WiFi.status() != WL_CONNECTED) setupWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    addLog("WiFi connected: " + WiFi.SSID() + "  IP " + WiFi.localIP().toString());
    configTzTime(cfg.tz, "pool.ntp.org", "time.google.com");
  }
  setupOTA();
  setupWeb();
  awakeUntil = millis() + cfg.awakeMins * 60000UL;
  nextReadMs = cur.valid ? millis() + cfg.readSecs * 1000UL : millis() + 2000UL;
  if (maintenance) addLog("Maintenance mode - staying awake");
  else addLog("Awake for " + String(cfg.awakeMins) + " min, then deep sleep for " + String(cfg.sleepMins) + " min");
  addLog("Web page: http://" + WiFi.localIP().toString() + "/  or http://" + cfg.hostname + ".local/");
}

// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);
  bootCount++;
  timerWake = (esp_reset_reason() == ESP_RST_DEEPSLEEP);
  resetReason = resetText();
  Serial.println();
  Serial.println("Flower Care Bridge v" FW_VERSION "  (" + resetReason + ", boot " + String(bootCount) + ")");

  if (!LittleFS.begin(true)) Serial.println("LittleFS mount failed");
  loadConfig();
  setenv("TZ", cfg.tz, 1); tzset();     // the clock keeps running through deep sleep

  BLEDevice::init("");
  pClient = BLEDevice::createClient();

  mqtt.setBufferSize(1024);
  mqtt.setSocketTimeout(3);
  mqtt.setCallback(onMqtt);

  if (timerWake && !maintenance) {
    if (!wakeCycle()) goToSleep("reading done");
    // maintenance mode was switched on while asleep: carry on into awake mode
  }
  startAwake();
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  processBle();
  if (wifiMaintain()) handleMqtt();

  if (bleState == BLE_IDLE && (long)(millis() - nextReadMs) >= 0) {
    nextReadMs = millis() + cfg.readSecs * 1000UL;
    startBle(BLE_READ);
  }
  if (sleepRequested && millis() - sleepReqMs > 800) goToSleep("Sleep now pressed");
  if (!maintenance && !otaInProgress && bleState == BLE_IDLE && (long)(millis() - awakeUntil) >= 0)
    goToSleep("awake time over");
  delay(2);
}
