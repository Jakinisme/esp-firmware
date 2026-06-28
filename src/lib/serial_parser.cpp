/**
 * @file serial_parser.cpp
 * @brief Lightweight JSON parser — reads newline-delimited JSON from RPi
 */

#include "lib/serial_parser.h"
#include <cstring>
#include <cstdlib>

// ---------------------------------------------------------------------------
//  Helpers — find a JSON key and extract its float value
// ---------------------------------------------------------------------------

/**
 * Find `"key":` inside @p json and return the float value that follows.
 * Returns true on success, storing the value in @p out.
 */
static bool extractFloat(const char* json, const char* key, float& out)
{
    const char* p = strstr(json, key);
    if (!p) return false;
    p += strlen(key);                       // skip past the key string
    while (*p == ' ' || *p == '\t') ++p;    // skip whitespace
    char* end = nullptr;
    float v = strtof(p, &end);
    if (end == p) return false;             // no conversion happened
    out = v;
    return true;
}

// ---------------------------------------------------------------------------
//  SerialParser
// ---------------------------------------------------------------------------

SerialParser::SerialParser()
    : pos_(0)
{
    memset(buf_, 0, BUF_SIZE);
}

void SerialParser::begin(unsigned long baud)
{
    Serial.begin(baud);
    Serial.setTimeout(5);   // short timeout for readStringUntil (unused here)
}

bool SerialParser::poll()
{
    bool gotCommand = false;

    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());

        if (c == '\n' || c == '\r') {
            // End of line → try to parse
            if (pos_ > 0) {
                buf_[pos_] = '\0';
                gotCommand = parseJSON(buf_);
            }
            pos_ = 0;
        } else {
            if (pos_ < BUF_SIZE - 1) {
                buf_[pos_++] = c;
            } else {
                // Buffer overflow — discard line
                pos_ = 0;
            }
        }
    }

    return gotCommand;
}

bool SerialParser::parseJSON(const char* json)
{
    // Sanity check: must contain opening brace
    if (json[0] != '{') {
        cmd_.valid = false;
        return false;
    }

    // Parse each field.  Keys are quoted and followed by colon.
    //   "s":      → surge
    //   "sw":     → sway
    //   "y":      → yaw
    //   "ts":     → timestamp (optional)
    //
    // IMPORTANT: search for "sw": BEFORE "s": to avoid false match.
    // Instead we use the full quoted key patterns which are unambiguous.

    float s = 0.0f, sw = 0.0f, y = 0.0f, ts = 0.0f;
    bool hasS  = extractFloat(json, "\"s\":",  s);
    bool hasSW = extractFloat(json, "\"sw\":", sw);
    bool hasY  = extractFloat(json, "\"y\":",  y);
    /*bool hasTS =*/ extractFloat(json, "\"ts\":", ts);

    if (!hasS || !hasSW || !hasY) {
        cmd_.valid = false;
        return false;
    }

    cmd_.surge     = constrain(s,  -1.0f, 1.0f);
    cmd_.sway      = constrain(sw, -1.0f, 1.0f);
    cmd_.yaw       = constrain(y,  -1.0f, 1.0f);
    cmd_.timestamp = ts;
    cmd_.valid     = true;
    return true;
}

void SerialParser::sendStatus(bool ok, bool failsafe,
                              const uint16_t throttle[NUM_MOTORS])
{
    // Compact JSON — matches what esp32_link._read_response() expects
    Serial.printf(
        "{\"ok\":%d,\"fs\":%d,\"m\":[%u,%u,%u,%u]}\n",
        ok ? 1 : 0,
        failsafe ? 1 : 0,
        throttle[M_NW], throttle[M_NE],
        throttle[M_SW], throttle[M_SE]
    );
}
