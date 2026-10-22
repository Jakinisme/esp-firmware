/**
 * @file config.h
 * @brief ASV Thrust Controller Configuration
 *
 * ESP32 + BLHeli_S (3D Mode) + PWM 50Hz
 * Push-pull catamaran X-configuration
 *
 * Thruster layout (top view, bow = up):
 *
 *        ──── hull kiri ────
 *       [NW]              [SW]
 *       front-left        rear-left
 *
 *              [CENTER]
 *
 *       [NE]              [SE]
 *       front-right       rear-right
 *        ──── hull kanan ────
 *
 * Mixing (push-pull catamaran):
 *   T_NW =  +surge  +sway  +yaw    front-left
 *   T_NE =  +surge  -sway  -yaw    front-right
 *   T_SW =  +surge  -sway  +yaw    rear-left
 *   T_SE =  +surge  +sway  -yaw    rear-right
 */

#pragma once

#include <Arduino.h>

// =====================================================================
//  Motor GPIO Pins — adjust to match your wiring
// =====================================================================
static constexpr uint8_t PIN_MOTOR_NW = 33;   // front-left  (kiri depan)
static constexpr uint8_t PIN_MOTOR_NE = 25;   // front-right (kanan depan)
static constexpr uint8_t PIN_MOTOR_SW = 26;   // rear-left   (kiri belakang)
static constexpr uint8_t PIN_MOTOR_SE = 27;   // rear-right  (kanan belakang)

static constexpr uint8_t NUM_MOTORS = 4;

// Motor index order (matches arrays everywhere)
enum MotorID : uint8_t { M_NW = 0, M_NE, M_SW, M_SE };

// =====================================================================
//  PWM Configuration (50 Hz servo-style)
// =====================================================================
static constexpr uint32_t PWM_FREQ       = 50;     // 50 Hz standard ESC
static constexpr uint8_t  PWM_RESOLUTION = 16;     // 16-bit (0–65535)

// LEDC channels — one per motor
static constexpr uint8_t LEDC_CH_NW = 0;
static constexpr uint8_t LEDC_CH_NE = 1;
static constexpr uint8_t LEDC_CH_SW = 2;
static constexpr uint8_t LEDC_CH_SE = 3;

// =====================================================================
//  BLHeli_S 3D-Mode PWM Ranges (microseconds)
//
//  1000 µs  → Full reverse
//  1500 µs  → Center / neutral (motor idle, no spin)
//  2000 µs  → Full forward
//
//  Motor starts spinning at ~1520 µs (forward) or ~1480 µs (reverse)
//  based on ESC deadband setting.
// =====================================================================
static constexpr uint16_t PWM_MIN_US     = 1200;   // full reverse
static constexpr uint16_t PWM_CENTER_US  = 1500;   // neutral (3D center)
static constexpr uint16_t PWM_MAX_US     = 1800;   // full forward

// =====================================================================
//  Motor Mixing Matrix
//
//  Row = motor,  Columns = [surge, sway, yaw]
//  T_NW =  +surge  +sway  +yaw
//  T_NE =  +surge  -sway  -yaw
//  T_SW =  +surge  -sway  +yaw
//  T_SE =  +surge  +sway  -yaw
// =====================================================================
static constexpr float MIX_MATRIX[NUM_MOTORS][3] = {
    /* NW */ { +1.0f, +1.0f, +1.0f },
    /* NE */ { +1.0f, -1.0f, -1.0f },
    /* SW */ { +1.0f, -1.0f, +1.0f },
    /* SE */ { +1.0f, +1.0f, -1.0f },
};

// =====================================================================
//  Per-motor direction inversion
//
//  true = flip forward/reverse in software (thruster mounted/ESC wired the
//  other way round).  Inverted motor: forward → pulse BELOW PWM_CENTER_US,
//  reverse → pulse ABOVE it.  Mixing, arming ramp and center snapping are
//  unaffected — only the sign fed to thrustToPulse3D() flips.
// =====================================================================
static constexpr bool MOTOR_INVERT[NUM_MOTORS] = {
    /* NW */ false,
    /* NE */ true,    // kanan atas (kanan depan) — putaran terbalik
    /* SW */ false,
    /* SE */ false,
};

// =====================================================================
//  Deadzone & Limits
// =====================================================================
static constexpr float MOTOR_DEADZONE = 0.03f;  // |thrust| < this → idle

// Slew rate limiter — max µs change per update cycle (called at 50 Hz)
// 40 µs/cycle × 50 Hz = 2000 µs/sec → full range (500µs) in 0.25 sec
static constexpr uint16_t MOTOR_SLEW_RATE_US = 40;

// Slower slew when leaving 3D neutral — BLHeli_S rejects abrupt deadband exits
// 12 µs/cycle × 50 Hz = 600 µs/sec → 1500→1650 in ~250 ms (matches working test)
static constexpr uint16_t MOTOR_NEUTRAL_EXIT_SLEW_US = 12;
static constexpr uint16_t MOTOR_NEUTRAL_BAND_US      = 30;   // ±30 µs around center

// Post-arm ramp: min → center after phase-2 (avoid instant 1000→1500 jump) {"s":0.2,"sw":0,"y":0}{"s":0.2,"sw":0,"y":0}
static constexpr uint32_t ARM_RETURN_RAMP_MS = 1000;

// Run forward ramp self-test once in setup() after arming (disable for production)
static constexpr bool ENABLE_STARTUP_SELF_TEST = false;

// =====================================================================
//  Serial (RPi ↔ ESP32)
// =====================================================================
static constexpr unsigned long SERIAL_BAUD = 115200;

// =====================================================================
//  Timing
// =====================================================================
static constexpr uint32_t FAILSAFE_TIMEOUT_MS   = 2000;   // no cmd → idle motors (RPi @ 10 Hz)

// Temporary mitigation for BLHeli_S "Brake on Stop" oscillation when snapping to
// 1500 µs while the prop still has spin-down momentum. Holds/decays the last
// drive output before true neutral. Permanent fix: BLHeliSuite → Brake on Stop /
// Non-Damped Mode. Tune 400–800 ms on the bench.
static constexpr uint32_t COOLDOWN_DURATION_MS   = 1000;

static constexpr uint32_t ARMING_DURATION_MS     = 3000;   // center-throttle at boot
static constexpr uint32_t ARMING_LOW_DURATION_MS = 3000;   // low-throttle (range learn)
static constexpr uint32_t STATUS_REPORT_MS       = 100;    // status to RPi @ 10 Hz
static constexpr uint32_t PWM_UPDATE_MS          = 20;     // 50 Hz PWM update rate

// =====================================================================
//  Status LED
// =====================================================================
static constexpr uint8_t PIN_LED = 2;   // built-in LED on ESP32 DevKit
