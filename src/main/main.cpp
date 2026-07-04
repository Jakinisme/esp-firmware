/**
 * @file main.cpp
 * @brief ASV Thrust Controller — ESP32 Firmware
 *
 * Receives thrust commands from Raspberry Pi over serial JSON,
 * performs motor mixing for a 4-thruster push-pull catamaran,
 * and drives BLHeli_S ESCs via PWM 50Hz (3D mode, bidirectional).
 *
 * State machine:
 *   INIT → ARMING → ARMED ↔ FAILSAFE
 *
 * LED patterns:
 *   INIT/ARMING : fast blink  (100 ms)
 *   ARMED       : solid ON
 *   FAILSAFE    : slow blink  (500 ms)
 *
 * @author  Jaki
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
static uint32_t lastMotorMs     = 0;   // millis() of last 50 Hz PWM tick
static uint32_t lastLedToggleMs = 0;
static bool     ledState        = false;

// Latest command cache
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

/** Send JSON status to RPi (also used during blocking self-test in setup). */
static void reportStatus()
{
    uint16_t thr[NUM_MOTORS];
    mixer.getThrottleValues(thr);
    bool isFailsafe = (state == State::FAILSAFE);
    parser.sendStatus(!isFailsafe, isFailsafe, thr);
}

/**
 * Run one 50 Hz motor cycle: mix active commands or hold neutral when idle.
 * Slew-rate math assumes exactly PWM_UPDATE_MS between calls.
 */
static void runMotorCycle(uint32_t nowMs)
{
    const bool commandStale = (nowMs - lastCmdMs) >= IDLE_NEUTRAL_DELAY_MS;
    const bool commandIdle  = MotorMixer::isCommandIdle(cmdSurge, cmdSway, cmdYaw);

    // Drop stale cached thrust so idle never keeps driving after commands stop
    if (commandStale && state == State::ARMED) {
        cmdSurge = cmdSway = cmdYaw = 0.0f;
    }

    if (state == State::FAILSAFE || commandStale || commandIdle) {
        mixer.holdNeutral();
    } else if (state == State::ARMED) {
        mixer.update(cmdSurge, cmdSway, cmdYaw);
    }

    mixer.tick();
}

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
    Serial.println("  PWM 50Hz | BLHeli_S 3D Mode");
    Serial.println("========================================");
    Serial.printf( "  Motors : NW=%d  NE=%d  SW=%d  SE=%d\n",
                   PIN_MOTOR_NW, PIN_MOTOR_NE, PIN_MOTOR_SW, PIN_MOTOR_SE);
    Serial.printf( "  Failsafe timeout : %lu ms\n", FAILSAFE_TIMEOUT_MS);
    Serial.printf( "  Arming duration  : %lu + %lu ms\n",
                   ARMING_DURATION_MS, ARMING_LOW_DURATION_MS);
    Serial.println("========================================");

    // Transition to arming
    state = State::ARMING;
    Serial.println("[main] State → ARMING");

    // mixer.begin() blocks while running the 2-phase arming sequence
    mixer.begin();

    // Move to armed (ESCs are now ready)
    state      = State::ARMED;
    Serial.println("[main] State → ARMED");
    ledOn();

    // ---- Startup Motor Self-Test (once, guarded — never runs from loop()) ----
    if (ENABLE_STARTUP_SELF_TEST) {
        Serial.println("[main] Self-test: ramping ALL motors forward (30%) ...");
        uint32_t testStart = millis();
        const uint32_t RAMP_UP_MS   = 1000;
        const uint32_t HOLD_MS      = 1500;
        const uint32_t RAMP_DOWN_MS = 1000;
        const uint32_t TOTAL_MS     = RAMP_UP_MS + HOLD_MS + RAMP_DOWN_MS;

        while (millis() - testStart < TOTAL_MS) {
            uint32_t elapsed = millis() - testStart;

            if (elapsed < RAMP_UP_MS) {
                float t = (float)elapsed / RAMP_UP_MS;
                mixer.update(t * 0.3f, 0.0f, 0.0f);
            } else if (elapsed < RAMP_UP_MS + HOLD_MS) {
                mixer.update(0.3f, 0.0f, 0.0f);
            } else {
                float t = 1.0f - (float)(elapsed - RAMP_UP_MS - HOLD_MS) / RAMP_DOWN_MS;
                mixer.update(t * 0.3f, 0.0f, 0.0f);
            }

            mixer.tick();
            delay(PWM_UPDATE_MS);

            if (millis() - lastStatusMs >= STATUS_REPORT_MS) {
                lastStatusMs = millis();
                reportStatus();
            }
        }

        mixer.holdNeutral();
        for (uint8_t n = 0; n < 40; ++n) {
            mixer.tick();
            delay(PWM_UPDATE_MS);
        }
        Serial.println("[main] Self-test complete ✓");
    } else {
        Serial.println("[main] Self-test skipped (ENABLE_STARTUP_SELF_TEST=false)");
    }

    mixer.holdNeutral();
    mixer.tick();

    lastCmdMs   = millis();
    lastMotorMs = millis();
}

// =====================================================================
//  Main loop
// =====================================================================

void loop()
{
    uint32_t nowMs = millis();

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
            mixer.holdNeutral();
            Serial.println("[main] ⚠ FAILSAFE — no command received");
        }
    }

    // ---- 3. Motor output @ 50 Hz ----
    // Slew-rate limiter requires fixed PWM_UPDATE_MS cadence; do NOT call
    // mixer.update/tick faster than this or ESCs see near-instant jumps.
    if (nowMs - lastMotorMs >= PWM_UPDATE_MS) {
        lastMotorMs = nowMs;
        runMotorCycle(nowMs);
    }

    // ---- 4. Status report to RPi ----
    if (nowMs - lastStatusMs >= STATUS_REPORT_MS) {
        lastStatusMs = nowMs;
        reportStatus();
    }

    // ---- 5. LED indicator ----
    switch (state) {
        case State::ARMING:   ledBlink(100);  break;
        case State::ARMED:    /* solid on */  break;
        case State::FAILSAFE: ledBlink(500);  break;
        default:              break;
    }
}
