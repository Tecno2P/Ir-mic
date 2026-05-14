// ============================================================
//  system_module.cpp  –  LED Real Implementation
// ============================================================
#include "system_module.h"
#include <WiFi.h>

SystemModule sysModule;

// ─────────────────────────────────────────────────────────────
void SystemModule::begin() {
    _loadConfig();
    _loadScheduleTasks();
    _ledCfg  = loadLedConfig();
    _initFastLED();
    Serial.println("[SYS] System module initialized");
}

void SystemModule::loop() {
    // LED tick every 20ms
    if (millis() - _ledTimer > 20) {
        _ledTimer = millis();
        ledTick();
    }
}

// ─────────────────────────────────────────────────────────────
void SystemModule::_initFastLED() {
    if (_ledCfg.numLeds == 0) return;
    uint8_t n = min((uint8_t)MAX_LEDS, _ledCfg.numLeds);

    // FastLED requires compile-time DATA_PIN template parameter.
    // GPIO2 is forbidden (boot strapping pin) - use GPIO13 instead.
    // GPIO13 is safe: not forbidden, not input-only, no other module uses it permanently.
    FastLED.clearData();
    FastLED.addLeds<WS2812B, 13, GRB>(_leds, n);
    FastLED.setBrightness(_ledCfg.brightness);
    FastLED.clear(true);
    _fastledInited = true;
    Serial.printf("[SYS] FastLED: %u LEDs GPIO13 brightness=%u\n",
                  n, _ledCfg.brightness);
}

void SystemModule::ledTick() {
    if (!_fastledInited) return;
    uint8_t n = min((uint8_t)MAX_LEDS, _ledCfg.numLeds);

    switch (_ledCfg.mode) {
        case LedMode::SOLID:
            for (int i = 0; i < n; i++)
                _leds[i] = CRGB(_ledCfg.r, _ledCfg.g, _ledCfg.b);
            FastLED.show();
            break;

        case LedMode::RAINBOW:
            for (int i = 0; i < n; i++)
                _leds[i] = CHSV(_ledHue + (i * 255 / n), 255, 200);
            _ledHue++;
            FastLED.show();
            break;

        case LedMode::RAVE:
            for (int i = 0; i < n; i++)
                _leds[i] = CHSV(random8(), 255, 200);
            FastLED.show();
            break;

        case LedMode::BLINK:
            _ledBlink = (_ledBlink + 1) & 0x1F;
            for (int i = 0; i < n; i++)
                _leds[i] = (_ledBlink < 16)
                    ? CRGB(_ledCfg.r, _ledCfg.g, _ledCfg.b)
                    : CRGB::Black;
            FastLED.show();
            break;

        case LedMode::PULSE: {
            uint8_t bright = (uint8_t)(128 + 127 * sin(_ledHue * 0.05f));
            for (int i = 0; i < n; i++)
                _leds[i] = CRGB(_ledCfg.r, _ledCfg.g, _ledCfg.b).nscale8(bright);
            _ledHue++;
            FastLED.show();
            break;
        }

        case LedMode::OFF:
        default:
            FastLED.clear(); FastLED.show();
            break;
    }
}

// ─────────────────────────────────────────────────────────────
void SystemModule::setLedMode(const LedConfig& cfg) {
    _ledCfg = cfg;
    saveLedConfig(cfg);
    if (!_fastledInited) _initFastLED();
    FastLED.setBrightness(cfg.brightness);
    _applyLed();
}

void SystemModule::_applyLed() {
    if (_ledCfg.mode == LedMode::OFF) {
        FastLED.clear(); FastLED.show();
    }
}

void SystemModule::saveLedConfig(const LedConfig& cfg) {
    File f = LittleFS.open(LED_CFG_FILE, "w");
    if (!f) return;
    JsonDocument doc;
    doc["type"]       = (int)cfg.type;
    doc["mode"]       = (int)cfg.mode;
    doc["dataPin"]    = cfg.dataPin;
    doc["numLeds"]    = cfg.numLeds;
    doc["r"]          = cfg.r;
    doc["g"]          = cfg.g;
    doc["b"]          = cfg.b;
    doc["brightness"] = cfg.brightness;
    serializeJson(doc, f);
    f.close();
}

LedConfig SystemModule::loadLedConfig() const {
    LedConfig cfg;
    if (!LittleFS.exists(LED_CFG_FILE)) return cfg;
    File f = LittleFS.open(LED_CFG_FILE, "r");
    if (!f) return cfg;
    JsonDocument doc;
    if (deserializeJson(doc, f) == DeserializationError::Ok) {
        cfg.type       = (LedType)(doc["type"]       | 0);
        cfg.mode       = (LedMode)(doc["mode"]       | 0);
        cfg.dataPin    = doc["dataPin"]    | (uint8_t)13;  // GPIO2 forbidden - default to GPIO13
        cfg.numLeds    = doc["numLeds"]    | (uint8_t)8;
        cfg.r          = doc["r"]          | (uint8_t)255;
        cfg.g          = doc["g"]          | (uint8_t)0;
        cfg.b          = doc["b"]          | (uint8_t)128;
        cfg.brightness = doc["brightness"] | (uint8_t)128;
    }
    f.close();
    return cfg;
}

// ─────────────────────────────────────────────────────────────
String SystemModule::getStatusJson() const {
    JsonDocument doc;
    doc["ok"]         = true;
    doc["heap"]       = ESP.getFreeHeap();
    doc["uptime"]     = millis() / 1000;
    doc["cpuMhz"]     = getCpuFrequencyMhz();
    doc["cpuFreq"]    = getCpuFrequencyMhz();
    doc["chip"]       = ESP.getChipModel();
    doc["chipModel"]  = ESP.getChipModel();
    doc["firmware"]   = FIRMWARE_VERSION;
    doc["flashSize"]  = (uint32_t)(ESP.getFlashChipSize() / 1024);
    // MAC address
    uint8_t macBytes[6]; WiFi.macAddress(macBytes);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             macBytes[0], macBytes[1], macBytes[2],
             macBytes[3], macBytes[4], macBytes[5]);
    doc["mac"] = String(macStr);
    // LED
    JsonObject led = doc["led"].to<JsonObject>();
    led["mode"]   = (int)_ledCfg.mode;
    led["active"] = isLedActive();
    led["numLeds"]= _ledCfg.numLeds;
    String out; serializeJson(doc, out);
    return out;
}

String SystemModule::hardwareStatusJson() const {
    char buf[48];
    snprintf(buf, sizeof(buf),
             "{\"led\":%s}",
             _fastledInited?"true":"false");
    return String(buf);
}

// ─────────────────────────────────────────────────────────────
void SystemModule::setGhostLink(bool en) {
    _ghostLink = en; _saveConfig();
}
void SystemModule::setTimezone(const String& tz) {
    _timezone = tz; _saveConfig();
}

void SystemModule::_loadConfig() {
    if (!LittleFS.exists(SYS_CFG_FILE)) return;
    File f = LittleFS.open(SYS_CFG_FILE, "r");
    if (!f) return;
    JsonDocument doc;
    if (deserializeJson(doc, f) == DeserializationError::Ok) {
        _ghostLink = doc["ghostLink"] | false;
        _timezone  = doc["timezone"]  | (const char*)"IST";
    }
    f.close();
}

void SystemModule::_saveConfig() const {
    File f = LittleFS.open(SYS_CFG_FILE, "w");
    if (!f) return;
    JsonDocument doc;
    doc["ghostLink"] = _ghostLink;
    doc["timezone"]  = _timezone;
    serializeJson(doc, f);
    f.close();
}

// ─────────────────────────────────────────────────────────────
uint32_t SystemModule::addScheduleTask(SysScheduleTask& task) {
    static uint32_t nextId = 1;
    task.id = nextId++;
    _schedTasks.push_back(task);
    _saveScheduleTasks();
    return task.id;
}

bool SystemModule::deleteScheduleTask(uint32_t id) {
    for (auto it = _schedTasks.begin(); it != _schedTasks.end(); ++it) {
        if (it->id == id) { _schedTasks.erase(it); _saveScheduleTasks(); return true; }
    }
    return false;
}

bool SystemModule::toggleScheduleTask(uint32_t id, bool en) {
    for (auto& t : _schedTasks) {
        if (t.id == id) { t.enabled = en; _saveScheduleTasks(); return true; }
    }
    return false;
}

String SystemModule::scheduleTasksToJson() const {
    String out = "{\"tasks\":[";
    for (size_t i = 0; i < _schedTasks.size(); i++) {
        if (i) out += ',';
        const auto& t = _schedTasks[i];
        out += "{\"id\":" + String(t.id)
             + ",\"name\":\"" + t.name
             + "\",\"time\":\"" + t.time
             + "\",\"action\":\"" + t.action
             + "\",\"enabled\":" + (t.enabled?"true":"false") + "}";
    }
    out += "]}";
    return out;
}

void SystemModule::_loadScheduleTasks() {
    _schedTasks.clear();
    if (!LittleFS.exists(SYS_SCHED_FILE)) return;
    File f = LittleFS.open(SYS_SCHED_FILE, "r");
    if (!f) return;
    JsonDocument doc;
    if (deserializeJson(doc, f) == DeserializationError::Ok) {
        for (JsonObject o : doc["tasks"].as<JsonArray>()) {
            SysScheduleTask t;
            t.id      = o["id"]      | (uint32_t)0;
            t.name    = o["name"]    | "";
            t.time    = o["time"]    | "";
            t.action  = o["action"]  | "";
            t.enabled = o["enabled"] | true;
            _schedTasks.push_back(t);
        }
    }
    f.close();
}

void SystemModule::_saveScheduleTasks() const {
    File f = LittleFS.open(SYS_SCHED_FILE, "w");
    if (!f) return;
    f.print("{\"tasks\":[");
    bool first = true;
    for (const auto& t : _schedTasks) {
        if (!first) f.print(',');
        first = false;
        f.printf("{\"id\":%u,\"name\":\"%s\",\"time\":\"%s\","
                 "\"action\":\"%s\",\"enabled\":%s}",
                 t.id, t.name.c_str(), t.time.c_str(),
                 t.action.c_str(), t.enabled?"true":"false");
    }
    f.print("]}");
    f.close();
}

String SystemModule::gpioOverviewJson() const {
    // Return used pins for GPIO conflict detection
    JsonDocument doc;
    JsonArray pins = doc["used"].to<JsonArray>();
    // LED pin
    if (_ledCfg.mode != LedMode::OFF) {
        JsonObject o = pins.add<JsonObject>();
        o["pin"] = _ledCfg.dataPin; o["module"] = "LED";
    }
    String out; serializeJson(doc, out);
    return out;
}
