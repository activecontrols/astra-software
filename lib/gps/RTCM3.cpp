#include "RTCM3.h"

/*
 * RTCM3 frame layout
 * ┌──────────┬──────────────────────────┬──────────────────────┬──────────────┐
 * │ 0xD3 (1B)│ 6 reserved + 10-bit      │  payload (0–1023 B)  │ CRC-24Q (3B) │
 * │ preamble │ length field (2B)        │                      │              │
 * └──────────┴──────────────────────────┴──────────────────────┴──────────────┘
 *
 * The first 12 bits of the payload are the message number (type).
 * buf[] stores the raw frame from the preamble onward so the CRC can be
 * verified over header + payload in one pass.
 */

RTCM3::RTCM3() {
    msg_cbk = nullptr;
    reset();
}

void RTCM3::reset() {
    state       = IDLE;
    pos         = 0;
    payload_len = 0;
}

void RTCM3::encode(uint8_t x) {
    switch (state) {

    // ── waiting for preamble ──────────────────────────────────────────────────
    case IDLE:
        if (x == RTCM3_PREAMBLE) {
            pos     = 0;
            buf[pos++] = x;
            state   = HEADER;
        }
        break;

    // ── 2 header bytes after preamble ────────────────────────────────────────
    case HEADER:
        buf[pos++] = x;
        if (pos == 3) {
            // Upper 6 bits of buf[1] are reserved and must be zero.
            if (buf[1] & 0xFC) {
                reset();
                // The byte we just stored might itself be a preamble.
                if (x == RTCM3_PREAMBLE) {
                    buf[pos++] = x;
                    state = HEADER;
                }
                break;
            }
            payload_len = ((uint16_t)(buf[1] & 0x03) << 8) | buf[2];
            state = (payload_len == 0) ? CRC : PAYLOAD;
        }
        break;

    // ── payload bytes ─────────────────────────────────────────────────────────
    case PAYLOAD:
        if (pos >= (int)(3 + RTCM3_MAX_PAYLOAD)) {
            // Overflow — this can't be a valid frame.
            reset();
            break;
        }
        buf[pos++] = x;
        if (pos == 3 + payload_len)
            state = CRC;
        break;

    // ── 3 CRC bytes then verify ───────────────────────────────────────────────
    case CRC:
        buf[pos++] = x;
        if (pos == 3 + payload_len + 3) {
            if (crc_ok() && msg_cbk) {
                const uint8_t *payload = &buf[3];
                // Message type is the first 12 bits of the payload (big-endian).
                uint16_t msg_type = payload_len >= 2
                    ? (((uint16_t)payload[0] << 4) | (payload[1] >> 4))
                    : 0;
                msg_cbk(msg_type, payload, payload_len);
            }
            reset();
        }
        break;
    }
}

// CRC-24Q — the polynomial mandated by the RTCM3 standard.
uint32_t RTCM3::crc24q(const uint8_t *data, size_t len) {
    uint32_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint32_t)data[i] << 16;
        for (int j = 0; j < 8; ++j) {
            crc <<= 1;
            if (crc & 0x1000000)
                crc ^= 0x1864CFB; // CRC-24Q polynomial
        }
    }
    return crc & 0xFFFFFF;
}

bool RTCM3::crc_ok() const {
    // CRC covers everything up to (but not including) the 3 CRC bytes.
    size_t   covered  = 3 + payload_len;
    uint32_t computed = crc24q(buf, covered);
    uint32_t received =
        ((uint32_t)buf[covered    ] << 16) |
        ((uint32_t)buf[covered + 1] <<  8) |
         (uint32_t)buf[covered + 2];
    return computed == received;
}
