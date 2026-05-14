#pragma once
// ============================================================
//  ir_transmitter.h  –  Multi-emitter IR transmit wrapper
//
//  v1.3.0 fixes:
//    - FreeRTOS mutex replaces bare bool for thread-safety
//    - Non-blocking async transmit via queue (loop() stays free)
//    - transmitAsync() posts to irTxQueue; IR task does the work
//    - transmit() still available for callers that need result
// ============================================================
#include <Arduino.h>
#include <vector>
#include <IRsend.h>
#include "ir_button.h"
#include "config.h"
#include "gpio_config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// ── IR TX command (posted to queue for async dispatch) ────────
struct IrTxCommand {
    IRButton btn;
    bool     rawMode;   // true → use rawData directly
};

class IRTransmitter {
public:
    IRTransmitter();
    ~IRTransmitter();

    // Call once from setup() — also starts the IR TX task.
    void begin(const IrPinConfig& pins);

    // Reconfigure emitters at runtime (no reboot needed).
    void reconfigure(const IrPinConfig& pins);

    // Blocking transmit — waits for mutex, does TX, returns result.
    // Safe to call from any task. Blocks caller for TX duration.
    bool transmit(const IRButton& btn);

    // Non-blocking async transmit — posts to irTxQueue.
    // Returns false only if queue is full (caller is not blocked).
    // Use this from loop() / scheduler / rule engine.
    bool transmitAsync(const IRButton& btn);

    // Transmit on a specific emitter index only (0-based). Blocking.
    bool transmitOn(uint8_t emitterIdx, const IRButton& btn);

    // Raw transmit on all enabled emitters. Blocking.
    bool transmitRaw(const uint16_t* data, size_t len,
                     uint16_t freqKHz = IR_DEFAULT_FREQ_KHZ);

    // Number of currently active emitters.
    uint8_t activeCount() const;

    // Active GPIO pin numbers for all enabled slots.
    std::vector<uint8_t> activePins() const;

    // Active GPIO for emitter at index i (255 = disabled/invalid).
    uint8_t emitterPin(uint8_t idx) const;

    // Queue handle — exposed so main.cpp can create the IR task.
    static QueueHandle_t txQueue;

private:
    IRsend*          _senders[IR_MAX_EMITTERS];
    uint8_t          _pins   [IR_MAX_EMITTERS];
    uint8_t          _count;
    SemaphoreHandle_t _txMutex;   // FreeRTOS mutex — replaces bare bool

    void destroyAll();
    void createSender(uint8_t idx, uint8_t pin);
    bool doTransmit(IRsend* s, const IRButton& btn);

    // IR TX task body — static so it can be passed to xTaskCreate
    static void _txTask(void* param);
};

extern IRTransmitter irTransmitter;
