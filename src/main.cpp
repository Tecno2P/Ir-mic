// ============================================================
//  main.cpp  –  IR Remote Web GUI  v5.1.0  |  ESP32-WROOM-32
//
//  v5.1.0 build:
//    FIX C-01: RFID writeCardAsync() - non-blocking write via dedicated task
//    FIX C-02: WDT ping task double-spawn guard (_pingActive flag)
//    FIX C-03: NRF24 channel scan chunked 10/tick (was 125/tick, 16ms block)
//    FIX C-04: Global VSPI bus mutex (g_spi_vspi_mutex) - SPI thread safety
//    FIX M-01: SD File* closeOnce lambda - no handle leak on keep-alive
//    FIX P-03: broadcastStatus() uses static char[] - no heap frag every 3s
//    FIX S-01: Default auth password derived from MAC (was hardcoded)
//    FIX S-02: OTA version check uses TLS CA cert (was setInsecure)
//    + 5 AC protocols: DAIKIN, MITSUBISHI_AC, WHYNTER, HAIER_AC, COOLIX
//    + IRButton icon + color fields
//    + Scheduler per-entry repeatCount / repeatDelay
//    + Internal LittleFS macros (MacroManager - no SD needed)
// ============================================================
#include <Arduino.h>
#include <LittleFS.h>
#include <esp_log.h>
#include "task_manager.h"     // FreeRTOS task architecture
#include "config.h"
#include "gpio_config.h"
#include "ota_manager.h"
#include "ir_database.h"
#include "ir_receiver.h"
#include "ir_transmitter.h"
#include "wifi_manager.h"
#include "web_server.h"
#include "group_manager.h"
#include "scheduler.h"
#include "sd_manager.h"
#include "macro_manager.h"
#include "nfc_module.h"
#include "rfid_module.h"
#include "subghz_module.h"
#include "nrf24_module.h"
#include "system_module.h"
#include "audit_manager.h"          // Batch 1: Audit Trail
#include "rule_manager.h"           // Batch 2: IF-THEN Automation
#include "auth_manager.h"           // Batch 3: Authentication
#include "watchdog_manager.h"       // Batch 3: Self-Healing Watchdog
#include "log_rotation.h"           // Batch 4: Log Rotation + CSV Export
#include "mic_module.h"             // Mic: I2S Streaming + SD Recording

static void initFilesystem();
static void ensureDefaultFiles();
static void onIRReceived(const IRButton& btn);
static void onScheduleFire(const ScheduleEntry& entry);
static void printBanner();

// FIX: removed static - web_server.cpp accesses this via extern for safe restart
portMUX_TYPE s_restartMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t s_restartAt = 0;

static unsigned long s_lastStatusBroadcast = 0;
#define STATUS_BROADCAST_INTERVAL_MS 3000

// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(300);
    printBanner();

    // Suppress VFS "does not exist" spam from ESPAsyncWebServer
    esp_log_level_set("vfs_api", ESP_LOG_ERROR);
    esp_log_level_set("vfs",     ESP_LOG_ERROR);

    // ── CPU frequency - lock at 240MHz for minimum IR timing jitter ─────
    // Without explicit setCpuFrequencyMhz(), ESP32 Arduino starts at 240MHz
    // but the watchdog's loadConfig() restores the saved perfMode later.
    // We set 240MHz here first so the WiFi/IR init path always runs at full
    // speed regardless of what perfMode was saved last session.
    setCpuFrequencyMhz(240);

    // ── LittleFS (always required) ────────────────────────────
    initFilesystem();

    // ── SD Card (optional - non-blocking if absent) ───────────
    // begin() returns immediately if no card is detected.
    // All existing features work unchanged without SD.
    sdMgr.begin();

    // ── IR Database ───────────────────────────────────────────
    irDB.begin();

    // ── Groups ────────────────────────────────────────────────
    groupMgr.begin();

    // ── Internal Macros (LittleFS - no SD needed) ─────────────
    macroMgr.onTransmit([](uint32_t buttonId) {
        IRButton btn = irDB.findById(buttonId);
        // FIX: use transmitAsync - macro steps call this from loop() context.
        // transmit() would block for the full TX duration per macro step.
        if (btn.id) irTransmitter.transmitAsync(btn);
    });
    macroMgr.begin();

    // ── New modules (Tasks 7-11) ─────────────────────────────
    nfcModule.begin();
    rfidModule.begin();
    subGhzModule.begin();
    nrf24Module.begin();
    sysModule.begin();
    micModule.begin();             // Mic: I2S streaming + SD recording

    // ── Audit Trail (Batch 1) ─────────────────────────────────
    auditMgr.begin();

    // ── Rule Engine (Batch 2) ─────────────────────────────────
    ruleMgr.onIrTransmit([](uint32_t buttonId) {
        IRButton btn = irDB.findById(buttonId);
        if (btn.id) {
            // FIX: transmitAsync - rule actions fire from loop() context.
            // Blocking transmit() here would stall all subsequent rule actions.
            irTransmitter.transmitAsync(btn);
            auditMgr.logIrTx(btn.name, btn.id);
        }
    });
    ruleMgr.onMacroRun([](const String& name) {
        macroMgr.run(name);
        auditMgr.logMacro(name, true);
    });
    ruleMgr.begin();
    ruleMgr.triggerBoot();

    // ── Auth (Batch 3) ────────────────────────────────────────
    authMgr.begin();

    // ── Watchdog (Batch 3) ────────────────────────────────────
    wdtMgr.begin();
    // Re-apply saved perfMode CPU freq now that loadConfig() has run inside begin().
    // setup() forced 240MHz earlier for reliable init; this restores the user's
    // chosen mode (TURBO=240, NORMAL=160, POWER_SAVE=80) without an extra reboot.
    wdtMgr.setPerfMode(wdtMgr.getPerfMode());
    // ── Log Rotation (Batch 4) ────────────────────────────────
    logRotMgr.begin();

    // ── WiFi (AP+STA dual-mode with event handler) ────────────
    wifiMgr.begin();

    // ── Load persisted IR pin config ──────────────────────────
    IrPinConfig irPins;
    wifiMgr.loadIrPins(irPins);

    // ── OTA callbacks ─────────────────────────────────────────
    otaMgr.onProgress([](size_t done, size_t total) {
        webUI.broadcastOtaProgress(done, total);
    });
    otaMgr.onEnd([](bool success, const String& msg) {
        webUI.broadcastOtaResult(success, msg);
        if (success) { taskENTER_CRITICAL(&s_restartMux); s_restartAt = (uint32_t)(millis() + 1200); taskEXIT_CRITICAL(&s_restartMux); }
    });

    // ── IR Receiver ───────────────────────────────────────────
    irReceiver.onReceive(onIRReceived);
    irReceiver.begin(irPins.recvPin);

    // ── IR Transmitter ────────────────────────────────────────
    irTransmitter.begin(irPins);

    // ── Web Server ────────────────────────────────────────────
    webUI.begin();
    webUI.startCaptivePortal();  // Batch 3: DNS redirect on AP mode

    // ── Scheduler ────────────────────────────────────────────
    scheduler.onFire(onScheduleFire);
    scheduler.begin();

    Serial.printf(DEBUG_TAG " Ready v%s  AP: http://%s\n",
                  FIRMWARE_VERSION, wifiMgr.apIP().c_str());
    Serial.printf(DEBUG_TAG " RX=GPIO%d  TX-active=%d  Groups=%u  Schedules=%u  Heap=%u\n",
                  irReceiver.activePin(),
                  irTransmitter.activeCount(),
                  (unsigned)groupMgr.size(),
                  (unsigned)scheduler.size(),
                  ESP.getFreeHeap());
    Serial.printf(DEBUG_TAG " SD: %s\n",
                  sdMgr.isAvailable() ? "MOUNTED" : "not present");

    if (sdMgr.isAvailable()) {
        SdStatus ss = sdMgr.status();
        Serial.printf(DEBUG_TAG " SD: %s  %lluMB total  %lluMB free\n",
                      ss.cardTypeStr.c_str(),
                      ss.totalBytes / (1024ULL * 1024ULL),
                      (ss.totalBytes - ss.usedBytes) / (1024ULL * 1024ULL));
    }

    // All modules initialised successfully - reset boot-failure counter.
    // This must be the LAST call in setup() so only a full clean boot clears it.
    wdtMgr.markBootSuccess();

    // ── Start FreeRTOS task architecture ─────────────────────
    // Must be AFTER all module begin() calls since tasks reference them.
    //   net_io (Core 0, priority 2): handles ALL blocking HTTPS + buzzer
    //   hw_poll (Core 1, priority 2): polls NFC/RFID/SubGHz/NRF24
    // loop() will no longer call those modules directly.
    taskMgr.begin();

    Serial.println(DEBUG_TAG " OTA: boot confirmation deferred until WiFi connects");
}

// ─────────────────────────────────────────────────────────────
void loop() {
    taskENTER_CRITICAL(&s_restartMux); uint32_t _ra = s_restartAt; taskEXIT_CRITICAL(&s_restartMux);
    if (_ra != 0 && millis() >= _ra) {  // fire when future timestamp reached
        taskENTER_CRITICAL(&s_restartMux); s_restartAt = 0; taskEXIT_CRITICAL(&s_restartMux);
        Serial.println(DEBUG_TAG " Restarting for OTA...");
        ESP.restart();
    }
    if (!otaMgr.isUpdating() && !otaMgr.restartPending()) {
        irReceiver.loop();
    }
    wifiMgr.loop();
    scheduler.loop();
    webUI.loop();
    irDB.loop();

    // SD loop: hot-plug probe, log flush, macro step tick
    sdMgr.loop();

    // Internal macro tick (LittleFS macros - no SD needed)
    macroMgr.loop();

    // ── Hardware polling moved to hw_poll task (Core 1, priority 2) ──────
    // nfcModule.loop(), rfidModule.loop(), subGhzModule.loop(),
    // nrf24Module.loop() - all removed from loop() and run in hw_poll task
    // at a fixed 20ms tick via vTaskDelayUntil(). This removes 4 SPI bus
    // polling calls from every loop() iteration, reducing worst-case loop
    // duration by ~15ms under heavy SPI activity.

    sysModule.loop();
    auditMgr.loop();   // Batch 1
    ruleMgr.loop();    // Batch 2
    authMgr.loop();    // Batch 3: expire sessions
    wdtMgr.loop();     // Batch 3: watchdog feed + health check
    micModule.loop();  // Mic: SD recording tick
    webUI.loopCaptivePortal(); // Batch 3: DNS captive portal
    logRotMgr.loop();  // Batch 4: auto log rotation

    // OTA watchdog: abort stale uploads if TCP connection was dropped
    otaMgr.tickWatchdog();

    if (millis() - s_lastStatusBroadcast >= STATUS_BROADCAST_INTERVAL_MS) {
        s_lastStatusBroadcast = millis();
        webUI.broadcastStatus();
    }
    yield();
}

// ─────────────────────────────────────────────────────────────
//  LittleFS init + default file creation
// ─────────────────────────────────────────────────────────────
static void initFilesystem() {
    if (!LittleFS.begin(true)) {
        Serial.println(DEBUG_TAG " FATAL: LittleFS mount+format failed.");
        return;
    }
    Serial.printf(DEBUG_TAG " LittleFS OK: total=%uKB  used=%uKB\n",
                  (unsigned)(LittleFS.totalBytes() / 1024),
                  (unsigned)(LittleFS.usedBytes()  / 1024));
    ensureDefaultFiles();
}

static void ensureFile(const char* path, const char* defaultContent) {
    if (LittleFS.exists(path)) return;
    File f = LittleFS.open(path, "w");
    if (!f) {
        Serial.printf(DEBUG_TAG " ERROR: Cannot create %s\n", path);
        return;
    }
    f.print(defaultContent);
    f.close();
    Serial.printf(DEBUG_TAG " Created default: %s\n", path);
}

static void ensureDefaultFiles() {
    ensureFile("/ir_database.json",  "{\"buttons\":[]}");
    ensureFile("/ir_pins.json",
        "{\"recvPin\":14,\"emitCount\":1,"
        "\"emit\":["
        "{\"pin\":27,\"enabled\":true},"
        "{\"pin\":26,\"enabled\":false},"
        "{\"pin\":25,\"enabled\":false},"
        "{\"pin\":33,\"enabled\":false}"
        "]}");
    ensureFile(CFG_FILE,
        "{\"apSSID\":\"IR-Remote\",\"apPass\":\"irremote123\","
        "\"apChannel\":1,\"apHidden\":false,"
        "\"staSSID\":\"\",\"staPass\":\"\",\"staEnabled\":false}");
    ensureFile("/groups.json",      "{\"groups\":[]}");
    ensureFile("/schedules.json",   "{\"schedules\":[]}");
    ensureFile("/ntp_config.json",  "{\"tzOffset\":19800,\"dstOffset\":0}");
    ensureFile(IR_AUTO_SAVE_FILE,   "{\"autoSave\":false}");
}

// ─────────────────────────────────────────────────────────────
static void onIRReceived(const IRButton& btn) {
    webUI.broadcastIREvent(btn);
    auditMgr.logIrRx(protocolName(btn.protocol), String(btn.code, HEX));  // Batch 1
    ruleMgr.triggerIrReceived(btn.id, protocolName(btn.protocol));         // Batch 2

    if (!irDB.autoSaveEnabled()) return;

    IRButton copy = btn;
    uint32_t savedId = irDB.autoSaveReceived(copy);

    if (savedId) {
        Serial.printf(DEBUG_TAG " [AutoSave] Saved '%s' as id=%u\n",
                      copy.name.c_str(), savedId);
        webUI.broadcastMessage(String("Auto-saved: ") + copy.name);

        // Mirror auto-saved signal to SD raw dump if SD available
        if (sdMgr.isAvailable() && btn.protocol == IRProtocol::RAW &&
            !btn.rawData.empty()) {
            sdMgr.saveRawDump(copy.name, btn.rawData.data(),
                              btn.rawData.size(), btn.freqKHz);
        }
    } else {
        if (btn.protocol != IRProtocol::RAW) {
            for (const auto& b : irDB.buttons()) {
                if (b.protocol == btn.protocol && b.code == btn.code) {
                    Serial.printf(DEBUG_TAG " [AutoSave] Duplicate skipped: '%s' (id=%u)\n",
                                  b.name.c_str(), b.id);
                    break;
                }
            }
        }
    }
}

static void onScheduleFire(const ScheduleEntry& entry) {
    IRButton copy = irDB.findById(entry.buttonId);
    if (!copy.id) {
        Serial.printf(DEBUG_TAG " Scheduler: button %u not found\n", entry.buttonId);
        return;
    }

    // Honour schedule-level repeatCount (overrides button default if > 1)
    uint8_t  fireCount = (entry.repeatCount > 1) ? entry.repeatCount : copy.repeatCount;
    uint16_t fireDelay = (entry.repeatDelay > 0) ? entry.repeatDelay : copy.repeatDelay;

    Serial.printf(DEBUG_TAG " Scheduler TX: btn=%u '%s'  fires=%u  delay=%ums\n",
                  copy.id, copy.name.c_str(), fireCount, fireDelay);

    // FIX: was calling irTransmitter.transmit() + delay() in loop() context.
    // With fireCount=3 and fireDelay=200ms this blocked loop() for ~650ms -
    // starving IR receiver, WebSocket flush, WDT feed, and all other modules.
    //
    // Fix: post each fire as a separate IrTxCommand to the TX queue.
    // The dedicated ir_tx FreeRTOS task (priority 5) executes them sequentially.
    // loop() returns immediately and continues servicing other modules.
    //
    // repeatDelay between fires: bake it into the button copy so the TX task
    // applies it between the outer repeat iterations inside doTransmit().
    copy.repeatCount = fireCount;
    copy.repeatDelay = fireDelay;
    irTransmitter.transmitAsync(copy);   // non-blocking - posts to ir_tx queue

    webUI.broadcastMessage(String("Scheduled TX: ") + copy.name);
    auditMgr.logScheduler(entry.name, entry.buttonId);
    if (sdMgr.isAvailable())
        sdMgr.log(String("Scheduler TX: ") + copy.name);
}

static void printBanner() {
    Serial.println(F("\n"
        "╔══════════════════════════════════════════════╗\n"
        "║   IR Remote Web GUI                          ║\n"
        "║   ESP32-WROOM-32  ·  Full Feature Build      ║\n"
        "║   NFC · RFID · SubGHz · NRF24 · System       ║\n"
        "╚══════════════════════════════════════════════╝"));
    Serial.printf("Version: %s\n", FIRMWARE_VERSION);
    Serial.printf("Chip: %s rev%d %uMHz  Flash:%uMB  Heap:%u\n",
                  ESP.getChipModel(), ESP.getChipRevision(),
                  getCpuFrequencyMhz(),
                  (unsigned)(ESP.getFlashChipSize() >> 20),
                  ESP.getFreeHeap());
}
