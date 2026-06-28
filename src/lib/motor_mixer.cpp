/**
 * @file motor_mixer.cpp
 * @brief Motor mixing implementation for push-pull catamaran (X-config)
 */

#include "lib/motor_mixer.h"
#include <Arduino.h>
#include <cmath>

// ---------------------------------------------------------------------------
//  Construction
// ---------------------------------------------------------------------------

MotorMixer::MotorMixer()
    : armed_(false)
{
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        motors_[i]    = nullptr;
        throttle_[i]  = 0;
        mixed_[i]     = 0.0f;
    }
}

// ---------------------------------------------------------------------------
//  Initialisation & arming
// ---------------------------------------------------------------------------

void MotorMixer::begin()
{
    // GPIO pin table — order must match MotorID enum (NW, NE, SW, SE)
    static constexpr gpio_num_t pins[NUM_MOTORS] = {
        PIN_MOTOR_NW, PIN_MOTOR_NE, PIN_MOTOR_SW, PIN_MOTOR_SE
    };
    static const char* names[NUM_MOTORS] = { "NW", "NE", "SW", "SE" };

    // ---- Create & initialise DShotRMT objects ----
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        motors_[i] = new DShotRMT(pins[i], DSHOT_MODE, DSHOT_BIDIR);
        dshot_result_t res = motors_[i]->begin();

        if (res.success) {
            Serial.printf("[mixer] Motor %s (GPIO %d) — DShot OK\n",
                          names[i], pins[i]);
        } else {
            Serial.printf("[mixer] Motor %s (GPIO %d) — DShot FAIL (%d)\n",
                          names[i], pins[i], res.result_code);
        }
    }

    // ---- Arming sequence: send motor-stop for ARMING_DURATION_MS ----
    Serial.printf("[mixer] Arming ESCs (%lu ms) ...\n", ARMING_DURATION_MS);
    uint32_t start = millis();
    while (millis() - start < ARMING_DURATION_MS) {
        for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
            motors_[i]->sendThrottle(0);   // DShot MOTOR_STOP command
        }
        delayMicroseconds(DSHOT_SEND_INTERVAL_US);
    }

    armed_ = true;
    Serial.println("[mixer] ESCs armed ✓");
}

// ---------------------------------------------------------------------------
//  Mixing & output
// ---------------------------------------------------------------------------

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

    // ---- 2. Convert to DShot 3D-mode throttle & send ----
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        throttle_[i] = thrustToThrottle3D(mixed_[i]);
        motors_[i]->sendThrottle(throttle_[i]);
    }
}

void MotorMixer::failsafe()
{
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        motors_[i]->sendThrottle(0);   // MOTOR_STOP
        throttle_[i] = 0;
        mixed_[i]    = 0.0f;
    }
}

void MotorMixer::sendIdle()
{
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        motors_[i]->sendThrottle(THROT_3D_CENTER);
        throttle_[i] = THROT_3D_CENTER;
        mixed_[i]    = 0.0f;
    }
}

// ---------------------------------------------------------------------------
//  Telemetry helpers
// ---------------------------------------------------------------------------

void MotorMixer::getThrottleValues(uint16_t out[NUM_MOTORS]) const
{
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) out[i] = throttle_[i];
}

void MotorMixer::getMixedValues(float out[NUM_MOTORS]) const
{
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) out[i] = mixed_[i];
}

// ---------------------------------------------------------------------------
//  3D-mode throttle conversion
// ---------------------------------------------------------------------------

uint16_t MotorMixer::thrustToThrottle3D(float thrust)
{
    // Deadzone → idle
    if (fabsf(thrust) < MOTOR_DEADZONE) {
        return 0;   // motor stop (not spinning)
    }

    // Clamp just in case
    thrust = fmaxf(-1.0f, fminf(1.0f, thrust));

    if (thrust > 0.0f) {
        // Forward: map (0, 1] → [THROT_3D_FWD_MIN, THROT_3D_FWD_MAX]
        //   thrust=small → 1049,  thrust=1.0 → 2047
        uint16_t range = THROT_3D_FWD_MAX - THROT_3D_FWD_MIN;   // 998
        return static_cast<uint16_t>(THROT_3D_FWD_MIN + thrust * range);
    } else {
        // Reverse: map [-1, 0) → [THROT_3D_REV_MAX, THROT_3D_REV_MIN]
        //   thrust=-small → 1047,  thrust=-1.0 → 48
        //   Formula: 1047 + thrust * (1047 - 48)  = 1047 + thrust * 999
        uint16_t range = THROT_3D_REV_MIN - THROT_3D_REV_MAX;   // 999
        return static_cast<uint16_t>(THROT_3D_REV_MIN + thrust * range);
    }
}
