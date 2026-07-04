/**
 * @file motor_mixer.h
 * @brief Motor mixing for 4-thruster push-pull catamaran (X-config)
 *
 * Receives high-level thrust commands (surge, sway, yaw) in [-1, 1]
 * and produces PWM signals for each ESC via BLHeli_S 3D-mode mapping.
 *
 * Uses ESP32 LEDC peripheral for precise PWM generation at 50 Hz.
 */

#pragma once

#include <Arduino.h>
#include "header/config.h"

class MotorMixer {
public:
    MotorMixer();

    /** Initialise four LEDC PWM channels and run the arming sequence. */
    void begin();

    /**
     * Mix surge / sway / yaw and send the result to all four ESCs.
     *
     * @param surge  Forward (+) / backward (-)  [-1, 1]
     * @param sway   Right   (+) / left     (-)  [-1, 1]
     * @param yaw    CW      (+) / CCW      (-)  [-1, 1]
     */
    void update(float surge, float sway, float yaw);

    /** Failsafe — ramp targets to center/neutral (no spin). */
    void failsafe();

    /** Send idle (center) to keep ESCs alive without spinning. */
    void sendIdle();

    /**
     * Hold exact neutral — snap when within deadband, otherwise ramp via tick().
     * Use whenever all thrust commands are in the idle deadzone.
     */
    void holdNeutral();

    /** True when all three axes are within MOTOR_DEADZONE. */
    static bool isCommandIdle(float surge, float sway, float yaw);

    /**
     * Apply slew-rate limiting and write PWM hardware.
     * Must be called at PWM_UPDATE_MS (50 Hz) from the main loop.
     */
    void tick();

    /** True once the arming sequence has completed. */
    bool isArmed() const { return armed_; }

    /** Copy the current PWM pulse widths in µs (for telemetry). */
    void getThrottleValues(uint16_t out[NUM_MOTORS]) const;

    /** Read the mixed thrust in [-1,1] per motor (for debug). */
    void getMixedValues(float out[NUM_MOTORS]) const;

private:
    uint8_t    ledcChannels_[NUM_MOTORS];
    uint8_t    pins_[NUM_MOTORS];
    uint16_t   pulseUs_[NUM_MOTORS];       // current PWM pulse width (µs)
    uint16_t   targetPulseUs_[NUM_MOTORS]; // target PWM (ramping toward this)
    float      mixed_[NUM_MOTORS];         // post-mix values [-1,1]
    bool       armed_;

    /** Write a PWM pulse width (µs) directly (no ramping). Used for arming. */
    void writePulse(uint8_t motorIdx, uint16_t pulseUs);

    /**
     * Move pulseUs_[motorIdx] toward targetPulseUs_[motorIdx] by at most
     * MOTOR_SLEW_RATE_US per call, then write the result to hardware.
     */
    void writeRampedPulse(uint8_t motorIdx);

    /** Blocking ramp for all motors (setup / arming only). */
    void rampAllBlocking(uint16_t targetUs, uint16_t stepUs, uint32_t intervalMs);

    /**
     * Convert a normalised thrust value [-1, 1] into a PWM pulse
     * width for BLHeli_S 3D mode.
     *
     *  0        → 1500 µs  (center, no spin)
     *  (0, +1]  → [1500, 2000] µs  forward
     *  [-1, 0)  → [1500, 1000] µs  reverse
     */
    static uint16_t thrustToPulse3D(float thrust);
};
