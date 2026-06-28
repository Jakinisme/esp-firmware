/**
 * @file main.cpp
 * @brief ASV Thrust Controller — ESP32 Firmware
 *
 * Receives thrust commands from Raspberry Pi over serial JSON,
 * performs motor mixing for a 4-thruster push-pull catamaran,
 * and drives BLHeli_S ESCs via DShot300 (3D mode, bidirectional).
 *
 * State machine:
 *   INIT → ARMING → ARMED ↔ FAILSAFE
 *
 * LED patterns:
 *   INIT/ARMING : fast blink  (100 ms)
 *   ARMED       : solid ON
 *   FAILSAFE    : slow blink  (500 ms)
 *
 * @author  Jaki (auto-generated, review before flight)
 */

#include <Arduino.h>
#include "header/config.h"
#include "lib/motor_mixer.h"
#include "lib/serial_parser.h"

// =====================================================================
//  State machine
// =====================================================================

enum class State : uint8_t {
    INIT,
    ARMING,
    ARMED,
    FAILSAFE,
};

static const char* stateStr(State s)
{
    switch (s) {
        case State::INIT:     return "INIT";
        case State::ARMING:   return "ARMING";
        case State::ARMED:    return "ARMED";
        case State::FAILSAFE: return "FAILSAFE";
        default:              return "???";
    }
}

// =====================================================================
//  Globals
// =====================================================================

static MotorMixer   mixer;
static SerialParser parser;
static State        state = State::INIT;

static uint32_t lastCmdMs       = 0;   // millis() of last valid command
static uint32_t lastStatusMs    = 0;   // millis() of last status report
static uint32_t lastDshotUs     = 0;   // micros() of last DShot send
static uint32_t lastLedToggleMs = 0;
static bool     ledState        = false;

// Latest command cache (used for continuous DShot refresh)
static float cmdSurge = 0.0f;
static float cmdSway  = 0.0f;
static float cmdYaw   = 0.0f;

// =====================================================================
//  LED helper
// =====================================================================

static void ledBlink(uint32_t intervalMs)
{
    uint32_t now = millis();
    if (now - lastLedToggleMs >= intervalMs) {
        lastLedToggleMs = now;
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState ? HIGH : LOW);
    }
}

static inline void ledOn()  { digitalWrite(PIN_LED, HIGH); ledState = true; }
static inline void ledOff() { digitalWrite(PIN_LED, LOW);  ledState = false; }

// =====================================================================
//  Setup
// =====================================================================

void setup()
{
    // LED
    pinMode(PIN_LED, OUTPUT);
    ledOff();

    // Serial (RPi ↔ ESP32)
    parser.begin(SERIAL_BAUD);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  ASV Thrust Controller — ESP32");
    Serial.println("  DShot300 | BLHeli_S 3D | Bidirectional");
    Serial.println("========================================");
    Serial.printf( "  Motors : NW=%d  NE=%d  SW=%d  SE=%d\n",
                   PIN_MOTOR_NW, PIN_MOTOR_NE, PIN_MOTOR_SW, PIN_MOTOR_SE);
    Serial.printf( "  Failsafe timeout : %lu ms\n", FAILSAFE_TIMEOUT_MS);
    Serial.printf( "  Arming duration  : %lu ms\n", ARMING_DURATION_MS);
    Serial.println("========================================");

    // Transition to arming
    state = State::ARMING;
    Serial.println("[main] State → ARMING");

    // mixer.begin() blocks for ARMING_DURATION_MS while sending zero-throttle
    mixer.begin();

    // Move to armed (ESCs are now ready)
    state      = State::ARMED;
    lastCmdMs  = millis();
    Serial.println("[main] State → ARMED");
    ledOn();
}

// =====================================================================
//  Main loop
// =====================================================================

void loop()
{
    uint32_t nowMs = millis();
    uint32_t nowUs = micros();

    // ---- 1. Read serial commands ----
    if (parser.poll()) {
        const ThrustCommand& cmd = parser.command();
        if (cmd.valid) {
            cmdSurge = cmd.surge;
            cmdSway  = cmd.sway;
            cmdYaw   = cmd.yaw;
            lastCmdMs = nowMs;

            // If we were in failsafe, recover
            if (state == State::FAILSAFE) {
                state = State::ARMED;
                Serial.println("[main] Failsafe cleared → ARMED");
                ledOn();
            }
        }
    }

    // ---- 2. Failsafe check ----
    if (state == State::ARMED) {
        if (nowMs - lastCmdMs >= FAILSAFE_TIMEOUT_MS) {
            state = State::FAILSAFE;
            cmdSurge = cmdSway = cmdYaw = 0.0f;
            mixer.failsafe();
            Serial.println("[main] ⚠ FAILSAFE — no command received");
        }
    }

    // ---- 3. DShot output (high-frequency refresh) ----
    if (nowUs - lastDshotUs >= DSHOT_SEND_INTERVAL_US) {
        lastDshotUs = nowUs;

        switch (state) {
            case State::ARMED:
                mixer.update(cmdSurge, cmdSway, cmdYaw);
                break;

            case State::FAILSAFE:
                mixer.failsafe();
                break;

            default:
                break;
        }
    }

    // ---- 4. Status report to RPi ----
    if (nowMs - lastStatusMs >= STATUS_REPORT_MS) {
        lastStatusMs = nowMs;

        uint16_t thr[NUM_MOTORS];
        mixer.getThrottleValues(thr);

        bool isFailsafe = (state == State::FAILSAFE);
        parser.sendStatus(!isFailsafe, isFailsafe, thr);
    }

    // ---- 5. LED indicator ----
    switch (state) {
        case State::ARMING:   ledBlink(100);  break;
        case State::ARMED:    /* solid on */  break;
        case State::FAILSAFE: ledBlink(500);  break;
        default:              break;
    }
}
