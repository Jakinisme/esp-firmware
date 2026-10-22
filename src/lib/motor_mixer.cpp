/**
 * @file motor_mixer.cpp
 * @brief Motor mixing implementation for push-pull catamaran (X-config)
 *
 * Uses ESP32 LEDC peripheral to generate 50 Hz PWM for BLHeli_S ESCs
 * in 3D mode.  PWM range: PWM_MIN_US–PWM_MAX_US, center PWM_CENTER_US.
 */

#include "lib/motor_mixer.h"
#include <Arduino.h>
#include <cmath>

MotorMixer::MotorMixer()
    : armed_(false)
{
    // Pin table — order matches MotorID enum (NW, NE, SW, SE)
    pins_[M_NW] = PIN_MOTOR_NW;
    pins_[M_NE] = PIN_MOTOR_NE;
    pins_[M_SW] = PIN_MOTOR_SW;
    pins_[M_SE] = PIN_MOTOR_SE;

    // LEDC channel table
    ledcChannels_[M_NW] = LEDC_CH_NW;
    ledcChannels_[M_NE] = LEDC_CH_NE;
    ledcChannels_[M_SW] = LEDC_CH_SW;
    ledcChannels_[M_SE] = LEDC_CH_SE;

    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        pulseUs_[i]       = PWM_CENTER_US;
        targetPulseUs_[i] = PWM_CENTER_US;
        mixed_[i]         = 0.0f;
    }
}

/**
 * Convert microseconds to LEDC duty value.
 * At 50 Hz (20 ms period) with 16-bit resolution (65536 steps):
 *   duty = pulseUs * 65535 / 20000
 */
void MotorMixer::writePulse(uint8_t motorIdx, uint16_t pulseUs)
{
    pulseUs_[motorIdx] = pulseUs;
    targetPulseUs_[motorIdx] = pulseUs;   // direct write — sync target too
    uint32_t duty = (uint32_t)pulseUs * ((1 << PWM_RESOLUTION) - 1) / 20000;
    ledcWrite(ledcChannels_[motorIdx], duty);
}

/**
 * Slew-rate-limited pulse writer.
 * Moves pulseUs_ toward targetPulseUs_ by at most MOTOR_SLEW_RATE_US
 * per call, then writes the result to the LEDC hardware.
 *
 * Called once per update cycle (every PWM_UPDATE_MS ≈ 20 ms).
 */
void MotorMixer::writeRampedPulse(uint8_t motorIdx)
{
    uint16_t current = pulseUs_[motorIdx];
    uint16_t target  = targetPulseUs_[motorIdx];

    if (current == target) return;

    // Snap to exact center when idle — prevents hunting at ESC deadband edge
    if (target == PWM_CENTER_US
        && abs((int16_t)current - (int16_t)PWM_CENTER_US) <= (int16_t)MOTOR_NEUTRAL_BAND_US) {
        pulseUs_[motorIdx] = PWM_CENTER_US;
        uint32_t duty = (uint32_t)PWM_CENTER_US * ((1 << PWM_RESOLUTION) - 1) / 20000;
        ledcWrite(ledcChannels_[motorIdx], duty);
        return;
    }

    int16_t diff = (int16_t)target - (int16_t)current;

    // Slower slew only when LEAVING center (not when returning to it)
    uint16_t maxStep = MOTOR_SLEW_RATE_US;
    if (target != PWM_CENTER_US
        && abs((int16_t)current - (int16_t)PWM_CENTER_US) <= (int16_t)MOTOR_NEUTRAL_BAND_US) {
        maxStep = MOTOR_NEUTRAL_EXIT_SLEW_US;
    }

    if (diff > (int16_t)maxStep) {
        current += maxStep;
    } else if (diff < -(int16_t)maxStep) {
        current -= maxStep;
    } else {
        current = target;
    }

    pulseUs_[motorIdx] = current;
    uint32_t duty = (uint32_t)current * ((1 << PWM_RESOLUTION) - 1) / 20000;
    ledcWrite(ledcChannels_[motorIdx], duty);
}

void MotorMixer::rampAllBlocking(uint16_t targetUs, uint16_t stepUs, uint32_t intervalMs)
{
    bool done = false;
    while (!done) {
        done = true;
        for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
            uint16_t current = pulseUs_[i];
            if (current == targetUs) continue;

            done = false;
            int16_t diff = (int16_t)targetUs - (int16_t)current;
            if (diff > (int16_t)stepUs) {
                current += stepUs;
            } else if (diff < -(int16_t)stepUs) {
                current -= stepUs;
            } else {
                current = targetUs;
            }

            pulseUs_[i]       = current;
            targetPulseUs_[i] = current;
            uint32_t duty = (uint32_t)current * ((1 << PWM_RESOLUTION) - 1) / 20000;
            ledcWrite(ledcChannels_[i], duty);
        }
        if (!done) {
            delay(intervalMs);
        }
    }
}

void MotorMixer::tick()
{
    if (!armed_) return;
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        writeRampedPulse(i);
    }
}

void MotorMixer::begin()
{
    static const char* names[NUM_MOTORS] = { "NW", "NE", "SW", "SE" };

    // ---- Setup LEDC channels ----
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        ledcSetup(ledcChannels_[i], PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(pins_[i], ledcChannels_[i]);
        Serial.printf("[mixer] Motor %s (GPIO %d, LEDC ch%d) — PWM OK\n",
                      names[i], pins_[i], ledcChannels_[i]);
    }

    // Phase 1: Send 3D center (1500 µs) — ESC recognises valid signal
    Serial.printf("[mixer] Arming phase 1 — center (%u µs) for %lu ms ...\n",
                  PWM_CENTER_US, ARMING_DURATION_MS);
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        writePulse(i, PWM_CENTER_US);
    }
    delay(ARMING_DURATION_MS);

    // Phase 2: Send min throttle (PWM_MIN_US) — required for ESC arm confirmation.
    // Must match the true low endpoint used in thrustToPulse3D() below, or the
    // ESC may never see the full swing it needs to complete arming.
    Serial.printf("[mixer] Arming phase 2 — min (%u µs) for %lu ms ...\n",
                  PWM_MIN_US, ARMING_LOW_DURATION_MS);
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        writePulse(i, PWM_MIN_US);
    }
    delay(ARMING_LOW_DURATION_MS);

    // Ramp back to center — instant 1000→1500 jump disarms BLHeli_S 3D ESCs
    Serial.printf("[mixer] Arming return — ramp min→center (%u µs) over %lu ms ...\n",
                  PWM_CENTER_US, ARM_RETURN_RAMP_MS);
    rampAllBlocking(PWM_CENTER_US, MOTOR_NEUTRAL_EXIT_SLEW_US, PWM_UPDATE_MS);

    armed_ = true;
    Serial.println("[mixer] ESCs armed ✓");
}


void MotorMixer::update(float surge, float sway, float yaw)
{
    if (!armed_) return;

    // ---- 1. Apply mixing matrix ----
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        float raw = MIX_MATRIX[i][0] * surge
                  + MIX_MATRIX[i][1] * sway
                  + MIX_MATRIX[i][2] * yaw;

        // Clamp to [-1, 1]
        mixed_[i] = fmaxf(-1.0f, fminf(1.0f, raw));
    }

    // ---- 2. Set target PWM (actual output applied in tick()) ----
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        targetPulseUs_[i] = thrustToPulse3D(mixed_[i], i);
    }
}

void MotorMixer::failsafe()
{
    // Ramp to center — instant snap causes ESC signal-loss / re-init
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        targetPulseUs_[i] = PWM_CENTER_US;
        mixed_[i] = 0.0f;
    }
}

void MotorMixer::sendIdle()
{
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        targetPulseUs_[i] = PWM_CENTER_US;
        mixed_[i] = 0.0f;
    }
}

bool MotorMixer::isCommandIdle(float surge, float sway, float yaw)
{
    return fabsf(surge) <= MOTOR_DEADZONE
        && fabsf(sway)  <= MOTOR_DEADZONE
        && fabsf(yaw)   <= MOTOR_DEADZONE;
}

void MotorMixer::beginCooldown()
{
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        cooldownStartPulseUs_[i] = pulseUs_[i];
    }
}

void MotorMixer::tickCooldown(uint32_t elapsedMs, uint32_t durationMs)
{
    if (durationMs == 0) {
        durationMs = 1;
    }

    float t = (float)elapsedMs / (float)durationMs;
    if (t > 1.0f) {
        t = 1.0f;
    }

    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        const uint16_t start = cooldownStartPulseUs_[i];
        const int16_t  diff  = (int16_t)PWM_CENTER_US - (int16_t)start;
        targetPulseUs_[i] = (uint16_t)((int16_t)start + (int16_t)(diff * t));
        mixed_[i] = 0.0f;
    }
}

void MotorMixer::holdNeutral()
{
    bool allInBand = true;

    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        targetPulseUs_[i] = PWM_CENTER_US;
        mixed_[i] = 0.0f;
        if (abs((int16_t)pulseUs_[i] - (int16_t)PWM_CENTER_US) > (int16_t)MOTOR_NEUTRAL_BAND_US) {
            allInBand = false;
        }
    }

    // Lock exact center once settled — prevents ESC deadband hunting at ~1520/1480
    if (allInBand) {
        for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
            writePulse(i, PWM_CENTER_US);
        }
    }
}

// ---------------------------------------------------------------------------
//  Telemetry helpers
// ---------------------------------------------------------------------------

void MotorMixer::getThrottleValues(uint16_t out[NUM_MOTORS]) const
{
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) out[i] = pulseUs_[i];
}

void MotorMixer::getMixedValues(float out[NUM_MOTORS]) const
{
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) out[i] = mixed_[i];
}

// ---------------------------------------------------------------------------
//  3D-mode PWM conversion
// ---------------------------------------------------------------------------

uint16_t MotorMixer::thrustToPulse3D(float thrust, uint8_t motorIdx)
{
    // Deadzone → center (neutral, no spin). Use <= so ±deadzone never maps to ±15 µs.
    if (fabsf(thrust) <= MOTOR_DEADZONE) {
        return PWM_CENTER_US;
    }

    // Clamp
    thrust = fmaxf(-1.0f, fminf(1.0f, thrust));

    // Backwards thruster: mirror around center so forward → below center,
    // reverse → above center.
    if (MOTOR_INVERT[motorIdx]) {
        thrust = -thrust;
    }

    // Linear map, now consistent with the arming sequence above:
    //   thrust  0 → PWM_CENTER_US (center)
    //   thrust +1 → PWM_MAX_US    (full forward)
    //   thrust -1 → PWM_MIN_US    (full reverse)
    if (thrust > 0.0f) {
        return (uint16_t)(PWM_CENTER_US + thrust * (PWM_MAX_US - PWM_CENTER_US));
    } else {
        return (uint16_t)(PWM_CENTER_US + thrust * (PWM_CENTER_US - PWM_MIN_US));
    }
}