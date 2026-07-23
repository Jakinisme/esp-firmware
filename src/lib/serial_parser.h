/**
 * @file serial_parser.h
 * @brief Lightweight JSON parser for RPi → ESP32 thrust commands
 *
 * Runs over UART2 (GPIO16=RX2, GPIO17=TX2) — NOT the default Serial/UART0,
 * so UART0 (GPIO1/3, shared with onboard USB) stays free for debug prints
 * and USB flashing/monitoring even while deployed and wired to the Pi.
 *
 * Expected frame (from pi-control esp32_link.py):
 *   {"s":0.400,"sw":-0.200,"y":0.100,"ts":123456.789}\n
 *
 * Keys:
 *   s   = surge  [-1, 1]
 *   sw  = sway   [-1, 1]
 *   y   = yaw    [-1, 1]
 *   ts  = timestamp (float, optional)
 */

#pragma once

#include <Arduino.h>
#include "header/config.h"

struct ThrustCommand {
    float    surge     = 0.0f;
    float    sway      = 0.0f;
    float    yaw       = 0.0f;
    float    timestamp = 0.0f;
    bool     valid     = false;
};

class SerialParser {
public:
    SerialParser();

    /** Call once in setup(). Starts UART2 on GPIO16(RX2)/GPIO17(TX2). */
    void begin(unsigned long baud = SERIAL_BAUD);

    /**
     * Non-blocking read from Serial2 (link to RPi).
     * @return true when a complete, valid command has been received.
     */
    bool poll();

    /** Latest parsed command (valid flag set when parse succeeds). */
    const ThrustCommand& command() const { return cmd_; }

    /**
     * Send JSON status back to RPi over UART2.
     *
     * @param ok         true if system is operating normally
     * @param failsafe   true if failsafe is active
     * @param throttle   current PWM pulse width per motor [4]
     */
    void sendStatus(bool ok, bool failsafe, const uint16_t throttle[NUM_MOTORS]);

private:
    static constexpr size_t BUF_SIZE = 256;
    char          buf_[BUF_SIZE];
    size_t        pos_;
    ThrustCommand cmd_;

    /**
     * Parse a complete JSON line into cmd_.
     * Uses strstr() for each key — no external JSON library required.
     */
    bool parseJSON(const char* json);
};