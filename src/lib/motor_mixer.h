/**
 * @file motor_mixer.h
 * @brief Motor mixing for 4-thruster push-pull catamaran (X-config)
 *
 * Receives high-level thrust commands (surge, sway, yaw) in [-1, 1]
 * and produces DShot throttle values for each motor via 3D-mode mapping.
 */

#pragma once

#include <DShotRMT.h>
#include "header/config.h"

class MotorMixer {
public:
    MotorMixer();

    /** Initialise four DShotRMT channels and run the arming sequence. */
    void begin();

    /**
     * Mix surge / sway / yaw and send the result to all four ESCs.
     *
     * @param surge  Forward (+) / backward (-)  [-1, 1]
     * @param sway   Right   (+) / left     (-)  [-1, 1]
     * @param yaw    CW      (+) / CCW      (-)  [-1, 1]
     */
    void update(float surge, float sway, float yaw);

    /** Emergency stop — sends motor-stop command to all ESCs. */
    void failsafe();

    /** Send DShot idle (3D-center) to keep ESCs alive without spinning. */
    void sendIdle();

    /** True once the arming sequence has completed. */
    bool isArmed() const { return armed_; }

    /** Copy the current DShot throttle values (for telemetry). */
    void getThrottleValues(uint16_t out[NUM_MOTORS]) const;

    /** Read the mixed thrust in [-1,1] per motor (for debug). */
    void getMixedValues(float out[NUM_MOTORS]) const;

private:
    DShotRMT*  motors_[NUM_MOTORS];
    uint16_t   throttle_[NUM_MOTORS];   // current DShot values
    float      mixed_[NUM_MOTORS];      // post-mix values [-1,1]
    bool       armed_;

    /**
     * Convert a normalised thrust value [-1, 1] into a DShot 3D-mode
     * throttle value.
     *
     *  0        → MOTOR_STOP (0)
     *  (0, +1]  → [1049, 2047]  forward
     *  [-1, 0)  → [1047, 48]   reverse
     */
    static uint16_t thrustToThrottle3D(float thrust);
};
