#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

// can change checksum type for different length reliability tradeoffs.

template <typename T, typename CsumT = int, uint8_t DS = 'S', uint8_t DE = 'E', uint8_t ESC = '\\', uint8_t R = 0x5A> struct PacketCodec {
  static constexpr size_t payload_size() {
    return sizeof(T);
  }
  static constexpr size_t csum_packet_size() {
    return sizeof(T) + sizeof(CsumT);
  }
  static constexpr size_t max_framed_size() {
    return 2 * csum_packet_size() + 2;
  }

  // checksum stuff

  static inline void csum_encode(const T &data, uint8_t *raw) {
    std::memcpy(raw, &data, sizeof(T));

    CsumT csum = 0;
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&data);
    for (size_t i = 0; i < sizeof(T); i++)
      csum = static_cast<CsumT>(csum + bytes[i]);

    std::memcpy(raw + sizeof(T), &csum, sizeof(CsumT));
  }

  static inline bool csum_decode(const uint8_t *raw, T &out) {
    std::memcpy(&out, raw, sizeof(T));

    CsumT received;
    std::memcpy(&received, raw + sizeof(T), sizeof(CsumT));

    CsumT csum = 0;
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&out);
    for (size_t i = 0; i < sizeof(T); i++)
      csum = static_cast<CsumT>(csum + bytes[i]);

    return received == csum;
  }

  // packet framing stuff

  static inline size_t escape_delimit(const uint8_t *raw, uint8_t *framed) {
    size_t out = 0;
    framed[out++] = DS;

    for (size_t i = 0; i < csum_packet_size(); i++) {
      uint8_t b = raw[i];
      if (b == DS || b == DE || b == ESC) {
        framed[out++] = ESC;
        framed[out++] = static_cast<uint8_t>(b ^ R);
      } else {
        framed[out++] = b;
      }
    }

    framed[out++] = DE;
    return out;
  }

  static inline size_t encode_framed(const T &data, uint8_t *framed) {
    uint8_t raw[csum_packet_size()];
    csum_encode(data, raw);
    return escape_delimit(raw, framed);
  }

  // decode for when full framed packet is available.
  // framed[0] == DS and framed[-1] == DE required
  static inline bool decode_framed(const uint8_t *framed, size_t framed_len, T &out) {
    if (framed_len < 2)
      return false;
    if (framed[0] != DS)
      return false;
    if (framed[framed_len - 1] != DE)
      return false;

    uint8_t raw[csum_packet_size()];
    size_t raw_i = 0;

    for (size_t i = 1; i + 1 < framed_len; i++) {
      uint8_t b = framed[i];

      if (b == ESC) {
        if ((i + 1) + 1 >= framed_len)
          return false; // dangling escape

        b = framed[++i] ^ R; // next byte xored.
      } else if (b == DS || b == DE) {
        return false; // DS and DE should never appear unescaped in a valid packet
      }

      if (raw_i >= csum_packet_size())
        return false;
      raw[raw_i++] = b;
    }

    if (raw_i != csum_packet_size())
      return false;
    return csum_decode(raw, out);
  }

  // decode when we have one byte at a time (like in reality)
  struct Decoder {
    enum class state : uint8_t { hunt, in_frame, escaped };

    uint8_t raw[csum_packet_size()];
    size_t raw_i = 0;
    state st = state::hunt;

    void reset() {
      st = state::hunt;
      raw_i = 0;
    }

    void start_frame() {
      st = state::in_frame;
      raw_i = 0;
    }

    // push one raw byte into buffer; overflow => reset. returns true on successful push.
    bool push_or_reset(uint8_t b) {
      if (raw_i >= csum_packet_size()) {
        reset();
        return false;
      }
      raw[raw_i++] = b;
      return true;
    }

    // feed one byte. returns true if a full valid packet was decoded into out.
    bool feed(uint8_t b, T &out) {
      switch (st) {

      case state::hunt:
        if (b == DS)
          start_frame();
        return false;

      case state::in_frame:
        if (b == DS) { // DS cannot appear unescaped, let's pretend this is the new start.
          start_frame();
          return false;
        }
        if (b == DE) { // end of frame
          bool ok = (raw_i == csum_packet_size()) && csum_decode(raw, out);
          reset();
          return ok;
        }
        if (b == ESC) { // next byte is escaped
          st = state::escaped;
          return false;
        }
        push_or_reset(b);
        return false;

      case state::escaped:
        if (push_or_reset(b ^ R)) // decode escaped byte
          st = state::in_frame;   // if successfully pushed (no reset), return to in_frame.
        return false;
      }
      return false; // unreachable, but keeps compilers happy
    }
  };
};