// ============================================================
//  switcher.ino  -  IR Remote ↔ RC Firmware Switcher
//  ESP32 | Arduino
//
//  Ye sketch Combo firmware ke saath milkar kaam karta hai.
//  Agar Combo chal raha ho aur aap GPIO0 (BOOT button) ko
//  2 second tak dabaayein → automatically ir_remote pe wapas.
//
//  FLASH KARNE KA TARIKA (ek baar):
//  Ye sketch SEEDHA flash nahi hota — ye ir_remote ka
//  ek PART hai. Alag kuch flash nahi karna.
//
//  KAAM KAISE KARTA HAI:
//  1. ESP32 boot hota hai Combo firmware (app1) pe
//  2. Combo apna kaam karta hai (BT/BLE/WiFi/RC)
//  3. Aap GPIO0 (BOOT) 2 sec dabaate ho
//  4. esp_ota_set_boot_partition(app0) → ir_remote
//  5. ESP32 restart → ir_remote wapas!
//
//  NOTE: Ye sketch ek ALAG .ino file hai jo Combo ke
//  saath flash hoti hai ek alag OTA slot mein NAHI —
//  balki ye standalone tiny firmware hai.
//
//  FLASH ADDRESS: 0x10000 (app0 slot temporarily)
//  Ya phir: Combo chal raha ho tab ye detect karta hai
// ============================================================

#include <Arduino.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"

// ── Config ────────────────────────────────────────────────────
#define BOOT_BTN_PIN      0        // GPIO0 = BOOT button (already on board)
#define HOLD_TIME_MS      2000     // 2 second dabaao
#define LED_PIN           2        // Onboard LED (GPIO2 on most ESP32 boards)
#define BLINK_INTERVAL_MS 200      // LED blink speed during countdown

// ── State ─────────────────────────────────────────────────────
static unsigned long btnPressStart = 0;
static bool          btnHeld       = false;

// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(300);

    pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println("\n[SWITCHER] Running — Hold GPIO0 (BOOT) 2 sec → IR Remote");

    // Show current boot partition info
    const esp_partition_t* running = esp_ota_get_running_partition();
    if (running) {
        Serial.printf("[SWITCHER] Current partition: %s (offset=0x%06lX)\n",
                      running->label, running->address);
    }
    const esp_partition_t* app0 = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (app0) {
        Serial.printf("[SWITCHER] app0 (ir_remote) at: 0x%06lX\n", app0->address);
    }
}

// ─────────────────────────────────────────────────────────────
void loop() {
    bool btnPressed = (digitalRead(BOOT_BTN_PIN) == LOW);

    if (btnPressed) {
        if (!btnHeld) {
            // Button just pressed
            btnHeld       = true;
            btnPressStart = millis();
            Serial.println("[SWITCHER] Button held... release ko 2 sec tak roko");
        }

        // Blink LED faster as countdown progresses
        unsigned long held = millis() - btnPressStart;
        unsigned long blinkSpeed = BLINK_INTERVAL_MS -
                                   (held * BLINK_INTERVAL_MS / HOLD_TIME_MS);
        if (blinkSpeed < 50) blinkSpeed = 50;
        digitalWrite(LED_PIN, (millis() / blinkSpeed) % 2);

        // Check if held long enough
        if (held >= HOLD_TIME_MS) {
            Serial.println("[SWITCHER] 2 sec hold detected! Switching to ir_remote...");
            switchToIrRemote();
        }

    } else {
        if (btnHeld) {
            // Button released before 2 sec
            Serial.println("[SWITCHER] Released too early. Phir koshish karo.");
            btnHeld = false;
            digitalWrite(LED_PIN, LOW);
        }
    }

    delay(10);
}

// ─────────────────────────────────────────────────────────────
//  switchToIrRemote
//  app0 partition ko boot partition set karke restart karo
// ─────────────────────────────────────────────────────────────
void switchToIrRemote() {
    // LED solid ON → switching
    digitalWrite(LED_PIN, HIGH);

    // Find app0 partition
    const esp_partition_t* app0 = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP,
        ESP_PARTITION_SUBTYPE_APP_OTA_0,
        NULL);

    if (!app0) {
        Serial.println("[SWITCHER] ERROR: app0 partition nahi mila!");
        // Blink rapidly to show error
        for (int i = 0; i < 10; i++) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
        return;
    }

    Serial.printf("[SWITCHER] Setting boot partition → %s\n", app0->label);

    esp_err_t err = esp_ota_set_boot_partition(app0);
    if (err != ESP_OK) {
        Serial.printf("[SWITCHER] ERROR: esp_ota_set_boot_partition: %s\n",
                      esp_err_to_name(err));
        // Blink error
        for (int i = 0; i < 10; i++) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
        return;
    }

    Serial.println("[SWITCHER] Boot partition set! Restarting → ir_remote...");
    delay(500);
    esp_restart();
}
