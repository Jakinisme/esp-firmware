/**
 * @file main.cpp
 * @brief ASV Thrust Controller — ESP32 Firmware
 *
 * Receives thrust commands from Raspberry Pi over serial JSON,
 * performs motor mixing for a 4-thruster push-pull catamaran,
 * and drives BLHeli_S ESCs via PWM 50Hz (3D mode, bidirectional).
 *
 * State machine:
 *   INIT → ARMING → ARMED ↔ COOLDOWN ↔ FAILSAFE
 *
 * COOLDOWN is a temporary soft-stop between active drive and true neutral hold
 * (see COOLDOWN_DURATION_MS in config.h). Distinct from FAILSAFE, which triggers
 * only on communication loss (no valid command for FAILSAFE_TIMEOUT_MS).
 *
 * LED patterns:
 *   INIT/ARMING : fast blink  (100 ms)
 *   ARMED/COOLDOWN : solid ON
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
    COOLDOWN,   // soft-stop after explicit stop/neutral command (not failsafe)
    FAILSAFE,
};

static const char* stateStr(State s)
{
    switch (s) {
        case State::INIT:     return "INIT";
        case State::ARMING:   return "ARMING";
        case State::ARMED:    return "ARMED";
        case State::COOLDOWN: return "COOLDOWN";
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
static uint32_t cooldownStartMs = 0;   // millis() when COOLDOWN began
static bool     commandWasActive = false; // previous cycle had non-idle thrust
static uint32_t lastStatusMs    = 0;   // millis() of last status report
static uint32_t lastMotorMs     = 0;   // millis() of last 50 Hz PWM tick
static uint32_t lastLedToggleMs = 0;
static bool     ledState        = false;

// Latest command cache
static float cmdSurge = 0.0f;
static float cmdSway  = 0.0f;
static float cmdYaw   = 0.0f;


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
 * Run one 50 Hz motor cycle.
 *
 * Four gated behaviors (never mixed):
 *   FAILSAFE       — ramp/hold neutral until a new valid command clears failsafe
 *   COOLDOWN       — decay last drive output toward neutral (soft-stop band-aid;
 *                    see COOLDOWN_DURATION_MS — not used for comm-loss failsafe)
 *   ARMED + idle   — after cooldown completes, hold true neutral (1500 µs)
 *   ARMED + active — track cached target via mixer.update(); hold at target
 *                    until a new command changes it or FAILSAFE_TIMEOUT_MS
 *                    expires (handled in loop(), not here)
 *
 * Slew-rate math assumes exactly PWM_UPDATE_MS between calls.
 */
static void runMotorCycle()
{
    const uint32_t nowMs = millis();

    if (state == State::FAILSAFE) {
        mixer.holdNeutral();
    } else if (state == State::COOLDOWN) {
        const uint32_t elapsed = nowMs - cooldownStartMs;
        if (elapsed >= COOLDOWN_DURATION_MS) {
            state = State::ARMED;
            mixer.holdNeutral();
            Serial.println("[main] Cooldown complete → neutral hold");
        } else {
            mixer.tickCooldown(elapsed, COOLDOWN_DURATION_MS);
        }
    } else if (state == State::ARMED) {
        if (MotorMixer::isCommandIdle(cmdSurge, cmdSway, cmdYaw)) {
            mixer.holdNeutral();
        } else {
            mixer.update(cmdSurge, cmdSway, cmdYaw);
        }
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

    // Serial (USB/UART0) — debug log ke PC. Tanpa ini Serial.println() tidak keluar.
    Serial.begin(SERIAL_BAUD);
    delay(300);   // tunggu USB CDC siap

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
    Serial.printf( "  Cooldown duration: %lu ms (ESC soft-stop band-aid)\n",
                   COOLDOWN_DURATION_MS);
    Serial.printf( "  Arming sequence  : center %lu ms → fwd %lu ms → center %lu ms"
                   " → rev %lu ms → center\n",
                   ARMING_DURATION_MS, ARMING_SWING_HOLD_MS,
                   ARMING_CENTER_HOLD_MS, ARMING_SWING_HOLD_MS);
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

    if (ENABLE_STARTUP_SELF_TEST) {
        Serial.println("[main] Self-test: ramping ALL motors forward (30%) ...");
        uint32_t testStart = millis();
        const uint32_t RAMP_UP_MS   = 10000;
        const uint32_t HOLD_MS      = 15000;
        const uint32_t RAMP_DOWN_MS = 10000;
        const uint32_t TOTAL_MS     = RAMP_UP_MS + HOLD_MS + RAMP_DOWN_MS;

        while (millis() - testStart < TOTAL_MS) {
            uint32_t elapsed = millis() - testStart;

            if (elapsed < RAMP_UP_MS) {
                float t = (float)elapsed / RAMP_UP_MS;
                mixer.update(t * 0.6f, 0.0f, 0.0f);
            } else if (elapsed < RAMP_UP_MS + HOLD_MS) {
                mixer.update(0.6f, 0.0f, 0.0f);
            } else {
                float t = 1.0f - (float)(elapsed - RAMP_UP_MS - HOLD_MS) / RAMP_DOWN_MS;
                mixer.update(t * 0.6f, 0.0f, 0.0f);
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

            const bool nowActive = !MotorMixer::isCommandIdle(cmdSurge, cmdSway, cmdYaw);

            // Explicit stop/neutral command (active → idle): soft-stop, not failsafe.
            if (state == State::ARMED && commandWasActive && !nowActive) {
                state = State::COOLDOWN;
                cooldownStartMs = nowMs;
                mixer.beginCooldown();
                Serial.println("[main] Active command ended → COOLDOWN (ESC soft-stop)");
            } else if (state == State::COOLDOWN && nowActive) {
                state = State::ARMED;
                Serial.println("[main] Cooldown cancelled → active command");
            }

            commandWasActive = nowActive;

            // If we were in failsafe, recover
            if (state == State::FAILSAFE) {
                state = State::ARMED;
                commandWasActive = nowActive;
                Serial.println("[main] Failsafe cleared → ARMED");
                ledOn();
            }
        }
    }

    // ---- 2. Failsafe check (communication loss only — not explicit stop) ----
    if (state == State::ARMED || state == State::COOLDOWN) {
        if (nowMs - lastCmdMs >= FAILSAFE_TIMEOUT_MS) {
            state = State::FAILSAFE;
            commandWasActive = false;
            cmdSurge = cmdSway = cmdYaw = 0.0f;
            mixer.holdNeutral();
            Serial.println("[main] ⚠ FAILSAFE — no command received");
        }
    }

    if (nowMs - lastMotorMs >= PWM_UPDATE_MS) {
        lastMotorMs = nowMs;
        runMotorCycle();
    }

    // ---- 4. Status report to RPi ----
    if (nowMs - lastStatusMs >= STATUS_REPORT_MS) {
        lastStatusMs = nowMs;
        reportStatus();
    }

    // ---- 5. LED indicator ----
    switch (state) {
        case State::ARMING:   ledBlink(100);  break;
        case State::ARMED:
        case State::COOLDOWN: /* solid on */  break;
        case State::FAILSAFE: ledBlink(500);  break;
        default:              break;
    }
}
