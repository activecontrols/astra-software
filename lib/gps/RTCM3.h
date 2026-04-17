#pragma once
#include <stdint.h>
#include <stddef.h>

#define RTCM3_PREAMBLE    0xD3
#define RTCM3_MAX_PAYLOAD 1023
// Full frame: 3-byte header + payload + 3-byte CRC
#define RTCM3_BUF_LEN     (3 + RTCM3_MAX_PAYLOAD + 3)

class RTCM3 {
public:
    RTCM3();

    // Feed one byte from the stream.
    void encode(uint8_t x);

    // Fired once per valid frame.
    // msg_type : 12-bit RTCM3 message number
    // payload  : pointer to raw payload bytes (valid only during the call)
    // length   : payload length in bytes
    void (*msg_cbk)(uint16_t msg_type, const uint8_t *payload, uint16_t length);

private:
    enum State : uint8_t {
        IDLE,       // waiting for 0xD3 preamble
        HEADER,     // collecting the 2 header bytes
        PAYLOAD,    // collecting payload bytes
        CRC         // collecting 3 CRC bytes
    };

    State    state;
    uint8_t  buf[RTCM3_BUF_LEN];
    int      pos;
    uint16_t payload_len;

    void reset();
    bool crc_ok() const;

    static uint32_t crc24q(const uint8_t *buf, size_t len);
};
