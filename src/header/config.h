 /**
 * @file config.h
 * @brief ASV Thrust Controller Configuration
 *
 * ESP32 + BLHeli_S (3D Mode) + DShot150 Bidirectional
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
 *   T_NW =  surge - sway + yaw    front-left
 *   T_NE =  surge + sway - yaw    front-right
 *   T_SW = -surge + sway - yaw    rear-left  (surge negated)
 *   T_SE = -surge - sway + yaw    rear-right (surge negated)
 */

#pragma once

#include <DShotRMT.h>

// =====================================================================
//  Motor GPIO Pins — adjust to match your wiring
// =====================================================================
static constexpr gpio_num_t PIN_MOTOR_NW = GPIO_NUM_25;   // front-left
static constexpr gpio_num_t PIN_MOTOR_NE = GPIO_NUM_26;   // front-right
static constexpr gpio_num_t PIN_MOTOR_SW = GPIO_NUM_27;   // rear-left
static constexpr gpio_num_t PIN_MOTOR_SE = GPIO_NUM_14;   // rear-right

static constexpr uint8_t NUM_MOTORS = 4;

// Motor index order (matches arrays everywhere)
enum MotorID : uint8_t { M_NW = 0, M_NE, M_SW, M_SE };

// =====================================================================
//  DShot Configuration
// =====================================================================
static constexpr dshot_mode_t DSHOT_MODE       = DSHOT150;
static constexpr bool         DSHOT_BIDIR      = true;   // telemetry enabled

// =====================================================================
//  BLHeli_S 3D-Mode Throttle Ranges
//
//  0          → Motor Stop / disarm command
//  48  – 1047 → Reverse  (48 = max-rev, 1047 = min-rev)
//  1048       → Neutral  (idle, motor doesn't spin)
//  1049 – 2047→ Forward  (1049 = min-fwd, 2047 = max-fwd)
// =====================================================================
static constexpr uint16_t THROT_3D_REV_MAX  = 48;    // fastest reverse
static constexpr uint16_t THROT_3D_REV_MIN  = 1047;  // slowest reverse
static constexpr uint16_t THROT_3D_CENTER   = 1048;   // neutral
static constexpr uint16_t THROT_3D_FWD_MIN  = 1049;  // slowest forward
static constexpr uint16_t THROT_3D_FWD_MAX  = 2047;  // fastest forward

// =====================================================================
//  Motor Mixing Matrix
//
//  Row = motor,  Columns = [surge, sway, yaw]
//  T_NW =  +surge  -sway  +yaw
//  T_NE =  +surge  +sway  -yaw
//  T_SW =  -surge  +sway  -yaw
//  T_SE =  -surge  -sway  +yaw
// =====================================================================
static constexpr float MIX_MATRIX[NUM_MOTORS][3] = {
    /* NW */ { +1.0f, -1.0f, +1.0f },
    /* NE */ { +1.0f, +1.0f, -1.0f },
    /* SW */ { -1.0f, +1.0f, -1.0f },
    /* SE */ { -1.0f, -1.0f, +1.0f },
};

// =====================================================================
//  Deadzone & Limits
// =====================================================================
static constexpr float MOTOR_DEADZONE = 0.03f;  // |thrust| < this → idle

// =====================================================================
//  Serial (RPi ↔ ESP32)
// =====================================================================
static constexpr unsigned long SERIAL_BAUD = 115200;

// =====================================================================
//  Timing
// =====================================================================
static constexpr uint32_t FAILSAFE_TIMEOUT_MS   = 500;    // no cmd → kill motors
static constexpr uint32_t ARMING_DURATION_MS     = 3000;   // zero-throttle at boot
static constexpr uint32_t STATUS_REPORT_MS       = 100;    // status to RPi @ 10 Hz
static constexpr uint32_t DSHOT_SEND_INTERVAL_US = 500;    // ~2 kHz DShot refresh

// =====================================================================
//  Status LED
// =====================================================================
static constexpr uint8_t PIN_LED = 2;   // built-in LED on ESP32 DevKit
