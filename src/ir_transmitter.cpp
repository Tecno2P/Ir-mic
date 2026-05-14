// ============================================================
//  ir_transmitter.cpp  –  Multi-emitter IR transmit
//
//  v1.3.0 fixes applied:
//    FIX-1: FreeRTOS mutex (_txMutex) replaces bare-bool guard.
//           xSemaphoreTake/Give is safe across tasks and ISR context.
//    FIX-2: Dedicated IR TX FreeRTOS task (_txTask) on Core 1.
//           transmitAsync() posts IRButton to txQueue without blocking
//           loop(). The task calls transmit() inside its own context,
//           so delay() inside doTransmit() only blocks that task.
//    FIX-3: transmitRaw() quiet-time was 5 ms; raised to 20 ms for
//           consistency with transmit(). Fixes sporadic raw TX failures.
// ============================================================
#include "ir_transmitter.h"
#include "ir_receiver.h"
#include <new>

IRTransmitter irTransmitter;

// Static queue handle — defined here, declared extern in header.
QueueHandle_t IRTransmitter::txQueue = nullptr;

// ── Constructor ───────────────────────────────────────────────
IRTransmitter::IRTransmitter() : _count(0), _txMutex(nullptr) {
    for (uint8_t i = 0; i < IR_MAX_EMITTERS; ++i) {
        _senders[i] = nullptr;
        _pins[i]    = 255;
    }
}

IRTransmitter::~IRTransmitter() { destroyAll(); }

// ── begin ─────────────────────────────────────────────────────
void IRTransmitter::begin(const IrPinConfig& pins) {
    // Create mutex once — idempotent if called again via reconfigure()
    if (!_txMutex) {
        _txMutex = xSemaphoreCreateMutex();
        if (!_txMutex) {
            Serial.println(DEBUG_TAG " FATAL: IR TX mutex creation failed");
            return;
        }
    }

    // Create async TX queue once (depth 8 — enough for burst scheduling)
    if (!txQueue) {
        txQueue = xQueueCreate(8, sizeof(IrTxCommand));
        if (!txQueue) {
            Serial.println(DEBUG_TAG " FATAL: IR TX queue creation failed");
            return;
        }
        // Pin IR TX task to Core 1 alongside loop() — IR timing is CPU-bound
        // and benefits from being on the same core as the Arduino task so
        // FreeRTOS cooperative scheduling keeps them interleaved cleanly.
        // Priority 5 > loop() priority (1) so IR fires promptly when queued.
        // Stack 6KB: 8 emitters × doTransmit frame + IRsend overhead is safe.
        xTaskCreatePinnedToCore(
            _txTask,        // task function
            "ir_tx",        // name
            6144,           // stack (8 emitters — bumped from 4096)
            this,           // param → IRTransmitter instance
            5,              // priority (higher than loop = 1)
            nullptr,        // handle not stored — task runs forever
            1               // Core 1 — same as Arduino loop()
        );
        Serial.println(DEBUG_TAG " IR TX task started on Core 1 (priority 5)");
    }

    destroyAll();
    _count = min((uint8_t)IR_MAX_EMITTERS, pins.emitCount);
    for (uint8_t i = 0; i < _count; ++i) {
        if (pins.emitEnabled[i]) createSender(i, pins.emitPin[i]);
    }
    Serial.printf(DEBUG_TAG " IR Transmitter: %d emitter(s) active\n", activeCount());
    for (uint8_t i = 0; i < _count; ++i) {
        if (_senders[i])
            Serial.printf(DEBUG_TAG "   Emitter[%d] GPIO%d\n", i, _pins[i]);
    }
}

// ── _txTask — IR TX FreeRTOS task ────────────────────────────
// Blocks on txQueue. When a command arrives, calls transmit() which
// takes _txMutex. delay() inside doTransmit() only blocks this task.
/*static*/
void IRTransmitter::_txTask(void* param) {
    IRTransmitter* self = static_cast<IRTransmitter*>(param);
    IrTxCommand cmd;
    for (;;) {
        // Block indefinitely until a command is posted
        if (xQueueReceive(IRTransmitter::txQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            if (cmd.rawMode && !cmd.btn.rawData.empty()) {
                self->transmitRaw(cmd.btn.rawData.data(),
                                  cmd.btn.rawData.size(),
                                  cmd.btn.freqKHz);
            } else {
                self->transmit(cmd.btn);
            }
        }
    }
}

// ── reconfigure ───────────────────────────────────────────────
void IRTransmitter::reconfigure(const IrPinConfig& pins) {
    Serial.println(DEBUG_TAG " Reconfiguring emitters...");
    // Take mutex so we don't reconfigure while a TX is in progress
    if (_txMutex && xSemaphoreTake(_txMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        destroyAll();
        _count = min((uint8_t)IR_MAX_EMITTERS, pins.emitCount);
        for (uint8_t i = 0; i < _count; ++i) {
            if (pins.emitEnabled[i]) createSender(i, pins.emitPin[i]);
        }
        xSemaphoreGive(_txMutex);
    } else {
        Serial.println(DEBUG_TAG " WARNING: reconfigure skipped — TX in progress");
    }
}

// ── createSender / destroyAll ─────────────────────────────────
void IRTransmitter::createSender(uint8_t idx, uint8_t pin) {
    if (idx >= IR_MAX_EMITTERS) return;
    if (_senders[idx]) { delete _senders[idx]; _senders[idx] = nullptr; }

    PinStatus st = validateTxPin(pin);
    if (st != PinStatus::OK) {
        Serial.printf(DEBUG_TAG " Emitter[%d] GPIO%d rejected: %s\n",
                      idx, pin, pinStatusMsg(st));
        _pins[idx] = 255;
        return;
    }
    // IRremoteESP8266 v2.8.6 constructor: (pin, inverted, use_modulation)
    // RMT channel is assigned automatically by the library per-pin.
    _senders[idx] = new (std::nothrow) IRsend(pin, false, true);
    if (!_senders[idx]) {
        Serial.printf(DEBUG_TAG " ERROR: IRsend[%d] allocation failed (OOM)\n", idx);
        _pins[idx] = 255;
        return;
    }
    _senders[idx]->begin();
    _pins[idx] = pin;
}

void IRTransmitter::destroyAll() {
    for (uint8_t i = 0; i < IR_MAX_EMITTERS; ++i) {
        if (_senders[i]) { delete _senders[i]; _senders[i] = nullptr; }
        _pins[i] = 255;
    }
    _count = 0;
}

// ── activeCount / emitterPin ──────────────────────────────────
uint8_t IRTransmitter::activeCount() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < IR_MAX_EMITTERS; ++i)
        if (_senders[i]) ++n;
    return n;
}

uint8_t IRTransmitter::emitterPin(uint8_t idx) const {
    return (idx < IR_MAX_EMITTERS && _senders[idx]) ? _pins[idx] : 255;
}

// ── transmitAsync — non-blocking post to TX task ──────────────
// Posts the IRButton to txQueue. Returns false if queue full.
// Loop/scheduler/rule-engine should call this instead of transmit().
bool IRTransmitter::transmitAsync(const IRButton& btn) {
    if (!txQueue) return false;
    if (!btn.isValid()) {
        Serial.println(DEBUG_TAG " transmitAsync: invalid button");
        return false;
    }
    if (activeCount() == 0) {
        Serial.println(DEBUG_TAG " transmitAsync: no active emitters");
        return false;
    }
    IrTxCommand cmd;
    cmd.btn     = btn;   // IRButton copy — safe, owned by cmd
    cmd.rawMode = false;
    BaseType_t sent = xQueueSend(txQueue, &cmd, 0);  // non-blocking
    if (sent != pdTRUE) {
        Serial.println(DEBUG_TAG " WARNING: IR TX queue full — command dropped");
        return false;
    }
    return true;
}

// ── transmit (blocking — protected by FreeRTOS mutex) ─────────
bool IRTransmitter::transmit(const IRButton& btn) {
    if (!btn.isValid()) { Serial.println(DEBUG_TAG " TX: invalid button"); return false; }
    if (activeCount() == 0) { Serial.println(DEBUG_TAG " TX: no active emitters"); return false; }

    // FIX: use FreeRTOS mutex instead of portENTER_CRITICAL / bare bool.
    // xSemaphoreTake is safe from any task and does not disable interrupts.
    if (!_txMutex) return false;
    if (xSemaphoreTake(_txMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Serial.println(DEBUG_TAG " TX: mutex timeout — another TX in progress");
        return false;
    }

    Serial.printf(DEBUG_TAG " TX %-10s 0x%llX  %db  reps=%d  count=%d  delay=%dms  emitters=%d  freq=%dkHz\n",
                  protocolName(btn.protocol), (unsigned long long)btn.code,
                  btn.bits, btn.repeats, btn.repeatCount, btn.repeatDelay, activeCount(), btn.freqKHz);

    irReceiver.pause();
    // Pre-TX quiet time — receiver must be silent before first modulated burst.
    // IR_PRE_TX_QUIET_MS (config.h) = 20ms. Stubborn devices may need 25-30ms.
    delay(IR_PRE_TX_QUIET_MS);

    bool ok = true;
    for (uint8_t i = 0; i < IR_MAX_EMITTERS; ++i) {
        if (_senders[i]) ok &= doTransmit(_senders[i], btn);
    }

    // Post-TX settle: wait for the signal to fully clear the target device
    // before we re-enable the receiver (avoids self-decoding our own burst).
    // For RAW signals, scale the wait to actual signal duration so we don't
    // cut off the trailing carrier on long AC state frames (>100ms).
    uint32_t waitMs = IR_POST_TX_SETTLE_MS;
    if (btn.protocol == IRProtocol::RAW && !btn.rawData.empty()) {
        uint32_t totalUs = 0;
        for (uint16_t v : btn.rawData) totalUs += v;
        // Add 20ms margin on top of the actual signal length.
        uint32_t dynWait = (totalUs / 1000u) + 20u;
        if (dynWait > waitMs) waitMs = dynWait;
        if (waitMs > 500u)    waitMs = 500u;   // hard cap — 500ms is plenty
    }
    delay(waitMs);
    irReceiver.resume();

    xSemaphoreGive(_txMutex);
    return ok;
}

// ── transmitOn (single emitter, blocking) ─────────────────────
bool IRTransmitter::transmitOn(uint8_t idx, const IRButton& btn) {
    if (idx >= IR_MAX_EMITTERS || !_senders[idx]) {
        Serial.printf(DEBUG_TAG " transmitOn: emitter[%d] not active\n", idx);
        return false;
    }
    if (!btn.isValid()) return false;
    if (!_txMutex) return false;
    if (xSemaphoreTake(_txMutex, pdMS_TO_TICKS(2000)) != pdTRUE) return false;

    irReceiver.pause();
    delay(IR_PRE_TX_QUIET_MS);
    bool ok = doTransmit(_senders[idx], btn);
    delay(IR_POST_TX_SETTLE_MS);
    irReceiver.resume();

    xSemaphoreGive(_txMutex);
    return ok;
}

// ── transmitRaw ───────────────────────────────────────────────
bool IRTransmitter::transmitRaw(const uint16_t* data, size_t len, uint16_t freqKHz) {
    if (!data || len < 4 || activeCount() == 0) return false;
    if (!_txMutex) return false;
    if (xSemaphoreTake(_txMutex, pdMS_TO_TICKS(2000)) != pdTRUE) return false;

    irReceiver.pause();
    delay(IR_PRE_TX_QUIET_MS);  // was 5ms then 20ms — now from config
    for (uint8_t i = 0; i < IR_MAX_EMITTERS; ++i)
        if (_senders[i]) _senders[i]->sendRaw(data, static_cast<uint16_t>(len), freqKHz);
    delay(IR_POST_TX_SETTLE_MS);
    irReceiver.resume();

    xSemaphoreGive(_txMutex);
    return true;
}

// ── doTransmit ───────────────────────────────────────────────
// Protocol-aware minimum repeats:
//   SONY      needs ≥3 frames (spec says transmit 3x; cheap receivers need it)
//   DISH      needs ≥4 frames (satellite protocol requirement)
//   RC5/RC6   needs ≥2 frames (toggle-bit protocol; single frame often ignored)
//   COOLIX    needs ≥2 frames (AC protocol; single frame unreliable)
//   All other protocols: honour btn.repeats (default IR_SEND_REPEATS = 2)
//
// Carrier frequency guard:
//   SONY = 40kHz, RC5/RC6 = 36kHz, everything else = 38kHz.
//   If btn.freqKHz is 0 or out of [33..56] range, we correct it here so
//   a misconfigured or legacy DB entry still transmits with correct carrier.
//
// "Large vs small device" signal reliability:
//   • Small/cheap receivers (e.g. old TVs, fans): benefit from more repeats
//   • Large AC units: use state-based protocols (DAIKIN etc.) or COOLIX/RAW —
//     these already encode full state in every frame; extra repeat frames help.
bool IRTransmitter::doTransmit(IRsend* s, const IRButton& btn) {
    uint8_t  total = (btn.repeatCount >= 1) ? btn.repeatCount : 1;
    uint16_t delMs = (btn.repeatDelay > 0) ? btn.repeatDelay : IR_DEFAULT_REPEAT_DELAY;

    uint8_t reps = btn.repeats;
    // minRep: ensures protocol-level repeat frames meet the protocol minimum.
    // btn.repeats (default = IR_SEND_REPEATS = 2) is used when it already
    // meets or exceeds the required minimum. This guarantees reliable
    // reception on both small (cheap) and large (AC) devices without
    // requiring the user to manually tune per-button repeat settings.
    auto minRep = [&](uint8_t minVal) -> uint8_t {
        return (reps >= minVal) ? reps : minVal;
    };

    // Carrier frequency guard — correct any legacy/zero value before TX.
    // Wrong carrier = device can't decode the signal even if timing is perfect.
    uint16_t txFreq = btn.freqKHz;
    if (txFreq < 33 || txFreq > 56) txFreq = 38;  // out-of-range → safe default
    switch (btn.protocol) {
        case IRProtocol::SONY:                       txFreq = 40; break;  // Sony spec
        case IRProtocol::RC5:  case IRProtocol::RC6: txFreq = 36; break;  // Philips spec
        default: if (btn.freqKHz >= 33 && btn.freqKHz <= 56) txFreq = btn.freqKHz; break;
    }

    for (uint8_t rep = 0; rep < total; ++rep) {
        switch (btn.protocol) {
            case IRProtocol::NEC:
            case IRProtocol::NEC_LIKE:
                s->sendNEC(btn.code, btn.bits, minRep(2)); break;   // 2 = NEC spec min for reliable decode
            case IRProtocol::SONY:
                s->sendSony(btn.code, btn.bits, minRep(3)); break;  // Sony spec: transmit 3x
            case IRProtocol::SAMSUNG:
                s->sendSAMSUNG(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::SAMSUNG36:
                s->sendSamsung36(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::LG:
                s->sendLG(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::PANASONIC: {
                uint16_t addr = (uint16_t)((btn.code >> 32) & 0xFFFF);
                uint32_t data = (uint32_t)(btn.code & 0xFFFFFFFF);
                s->sendPanasonic(addr, data,
                    btn.bits == 0 ? kPanasonicBits : btn.bits, minRep(2));
                break; }
            case IRProtocol::RC5:
                s->sendRC5(btn.code, btn.bits, minRep(2)); break;   // RC5 toggle: 2 frames needed
            case IRProtocol::RC6:
                s->sendRC6(btn.code, btn.bits, minRep(2)); break;   // RC6 toggle: same
            case IRProtocol::JVC:
                s->sendJVC(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::DISH:
                s->sendDISH(btn.code, btn.bits, minRep(4)); break;  // DISH protocol requires ≥4 frames
            case IRProtocol::SHARP: {
                uint8_t addr = (btn.code >> 8) & 0x1F;
                uint8_t cmd  = btn.code & 0xFF;
                s->sendSharp(addr, cmd, btn.bits, minRep(2)); break; }
            case IRProtocol::DENON:
                s->sendDenon(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::MITSUBISHI:
                s->sendMitsubishi(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::MITSUBISHI2:
                s->sendMitsubishi2(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::SANYO:
                s->sendSanyoLC7461(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::AIWA_RC_T501:
            case IRProtocol::AIWA_RC_T501_2:
                s->sendAiwaRCT501(btn.code, minRep(2)); break;
            case IRProtocol::NIKAI:
                s->sendNikai(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::RCMM:
                s->sendRCMM(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::LEGOPF:
                s->sendLegoPf(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::PIONEER:
                s->sendPioneer(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::EPSON:
                s->sendEpson(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::SYMPHONY:
                s->sendSymphony(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::BOSE:
                s->sendBose(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::METZ:
                s->sendMetz(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::DOSHISHA:
                s->sendDoshisha(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::GORENJE:
                s->sendGorenje(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::INAX:
                s->sendInax(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::LUTRON:
                s->sendLutron(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::ELITESCREENS:
                s->sendElitescreens(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::MILESTAG2:
                s->sendMilestag2(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::XMP:
                s->sendXmp(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::TRUMA:
                s->sendTruma(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::WOWWEE:
                s->sendWowwee(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::TECO:
                s->sendTeco(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::GOODWEATHER:
                s->sendGoodweather(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::MIDEA:
                s->sendMidea(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::MIDEA24:
                s->sendMidea24(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::COOLIX:
                s->sendCOOLIX(btn.code, btn.bits, minRep(2)); break;  // AC: 2 frames min
            case IRProtocol::COOLIX48:
                s->sendCoolix48(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::GICABLE:
                s->sendGICable(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::MAGIQUEST:
                s->sendMagiQuest(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::LASERTAG:
                s->sendLasertag(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::ARRIS:
                s->sendArris(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::MULTIBRACKETS:
                s->sendMultibrackets(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::ZEPEAL:
                s->sendZepeal(btn.code, btn.bits, minRep(2)); break;
            case IRProtocol::MWM:
                if (!btn.rawData.empty()) {
                    s->sendRaw(btn.rawData.data(),
                               static_cast<uint16_t>(btn.rawData.size()),
                               txFreq);
                } else {
                    uint8_t mwmData[3];
                    mwmData[0] = (btn.code >> 16) & 0xFF;
                    mwmData[1] = (btn.code >>  8) & 0xFF;
                    mwmData[2] =  btn.code        & 0xFF;
                    uint16_t nb = (btn.bits > 0) ? (btn.bits + 7) / 8 : 3;
                    if (nb > 3) nb = 3;
                    s->sendMWM(mwmData, nb, btn.repeats);
                }
                break;
            case IRProtocol::RAW:
            default:
                if (btn.rawData.empty()) return false;
                s->sendRaw(btn.rawData.data(),
                           static_cast<uint16_t>(btn.rawData.size()),
                           txFreq);
                break;
        }
        if (rep + 1 < total && delMs > 0) delay(delMs);
    }
    return true;
}

// ── activePins ───────────────────────────────────────────────
std::vector<uint8_t> IRTransmitter::activePins() const {
    std::vector<uint8_t> result;
    for (uint8_t i = 0; i < IR_MAX_EMITTERS; ++i) {
        if (_senders[i] && _pins[i] != 255)
            result.push_back(_pins[i]);
    }
    return result;
}
