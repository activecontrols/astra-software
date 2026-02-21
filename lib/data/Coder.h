#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

template <uint16_t MAX_LENGTH, typename CsumT = int32_t, uint8_t DS = 'S', uint8_t DE = 'E', uint8_t ESC = '\\'> struct VarPacketCodec {

  static constexpr uint16_t max_framed_size() {
    // 2 * (length + payload + checksum) + start + end
    return 2 * (MAX_LENGTH + sizeof(CsumT) + sizeof(uint16_t)) + 2; // worst case, everything needs to be escaped.
  }

  // encode into dest given bytes and len, returning encoded length.
  static inline uint16_t encode(const uint8_t *src, uint16_t src_len, uint8_t *dest) {
    if (src_len > MAX_LENGTH)
      return 0;

    CsumT csum = 0;
    uint16_t out_i = 0;

    auto push_escaped = [&](uint8_t b) {
      if (b == DS || b == DE || b == ESC) {
        dest[out_i++] = ESC;
      }
      dest[out_i++] = b;
    };

    dest[out_i++] = DS; // start byte.

    // add two byte length.
    push_escaped((uint8_t)src_len);
    push_escaped((uint8_t)(src_len >> 8));

    // add escaped bytes while calculating checksum.
    for (uint16_t i = 0; i < src_len; i++) {
      uint8_t b = src[i];
      csum += b;
      push_escaped(b);
    }

    uint8_t *csum_bytes = (uint8_t *)(&csum);
    for (uint16_t i = 0; i < sizeof(CsumT); i++)
      push_escaped(csum_bytes[i]);

    dest[out_i++] = DE; // end byte.
    return out_i;
  }

  struct Decoder {
    enum class State : uint8_t { HUNT, LEN_0, LEN_1, DATA, AWAIT_END };

    State state = State::HUNT;
    bool escaped = false;
    uint16_t expected_len = 0; // payload length only
    // uint16_t expected_total = 0; // payload + checksum
    uint8_t buffer[MAX_LENGTH + sizeof(CsumT)];
    uint16_t buf_i = 0;

    // void reset() {
    //   state = State::HUNT;
    //   escaped = false;
    //   buf_i = 0;
    // }

    uint16_t feed(uint8_t b, uint8_t *dest) {
      if (b == DS && !escaped) { // unescaped ds is start, regardless of state.
        state = State::LEN_0;
        buf_i = 0;
        return 0;
      }
      if (b == ESC && !escaped) {
        escaped = true;
        return 0;
      }

      bool was_escaped = escaped; // useful when checking DE (since it must be unescaped)
      escaped = false;

      switch (state) {

      case State::HUNT: // we start when we see unescaped DS
        break;

      case State::LEN_0: // lower byte
        expected_len = b;
        state = State::LEN_1;
        break;

      case State::LEN_1: // upper byte
        expected_len |= ((uint16_t)b) << 8;
        if (expected_len > MAX_LENGTH) {
          state = State::HUNT;
        } else {
          buf_i = 0;
          state = State::DATA;
        }
        break;

      case State::DATA: // captures payload and checksum.
                        // todo: we should maybe check for unescaped DE here in case length field is garbled.
                        // however, we already check for unescaped DS so maybe its ok.
        buffer[buf_i++] = b;
        if (buf_i == expected_len + sizeof(CsumT))
          state = State::AWAIT_END;
        break;

      case State::AWAIT_END:
        state = State::HUNT;
        if (b == DE && !was_escaped) // unescaped DE
          return validate(dest);
        break;
      }
      return 0;
    }

  private:
    uint16_t validate(uint8_t *dest) {
      CsumT calculated = 0;
      for (uint16_t i = 0; i < expected_len; i++)
        calculated += buffer[i];

      CsumT received;
      memcpy(&received, buffer + expected_len, sizeof(CsumT));

      if (calculated != received)
        return 0;

      memcpy(dest, buffer, expected_len);
      return expected_len;
    }
  };
};

template <typename T, typename CsumT = int32_t, uint8_t DS = 'S', uint8_t DE = 'E', uint8_t ESC = '\\'> struct PacketCodec {
  using Codec = VarPacketCodec<sizeof(T), CsumT, DS, DE, ESC>;

  static constexpr uint16_t max_framed_size() {
    return Codec::max_framed_size();
  }

  static inline uint16_t encode(const T &src, uint8_t *dest) {
    return Codec::encode((const uint8_t)(&src), sizeof(T), dest);
  }

  struct Decoder {
    bool feed(uint8_t b, T &out_obj) {
      uint16_t decoded_len = Codec::Decoder.feed(b, reinterpret_cast<uint8_t *>(&out_obj));
      return decoded_len == sizeof(T); // go until length matches.
    }
  };
};

// LLM tests:

/**
 * @brief Runs protocol self-tests and returns the total number of successful tests.
 * @return int: Number of passing tests (Target: 5)
 */
inline int run_protocol_tests() {
  using Codec = VarPacketCodec<128, int32_t, 'S', 'E', '\\'>;
  Codec::Decoder decoder;

  uint8_t encode_buf[Codec::max_framed_size()];
  uint8_t decode_buf[128];
  int passed = 0;

  // Helper lambda to run a standard encode/decode cycle
  auto run_single_test = [&](const uint8_t *payload, uint16_t len) -> bool {
    decoder = {}; // Reset decoder state for clean test isolation

    uint16_t enc_len = Codec::encode(payload, len, encode_buf);
    if (enc_len == 0)
      return false;

    uint16_t result_len = 0;
    for (uint16_t i = 0; i < enc_len; i++) {
      result_len = decoder.feed(encode_buf[i], decode_buf);
      if (result_len > 0)
        break;
    }

    return (result_len == len && memcmp(payload, decode_buf, len) == 0);
  };

  // --- TEST 1: Basic String ---
  if (run_single_test((uint8_t *)"hello", 5))
    passed++;

  // --- TEST 2: Escaping Gauntlet ---
  uint8_t escape_payload[] = {'S', '\\', 'E', 'S', 'E', '\\'};
  if (run_single_test(escape_payload, sizeof(escape_payload)))
    passed++;

  // --- TEST 3: Checksum Corruption ---
  {
    decoder = {};
    uint16_t enc_len = Codec::encode((uint8_t *)"valid", 5, encode_buf);
    encode_buf[enc_len - 3] ^= 0xFF; // Corrupt the checksum

    uint16_t result_len = 0;
    for (uint16_t i = 0; i < enc_len; i++) {
      result_len = decoder.feed(encode_buf[i], decode_buf);
    }
    // Success if the packet was REJECTED (result_len == 0)
    if (result_len == 0)
      passed++;
  }

  // --- TEST 4: Recovery from Desync ---
  {
    decoder = {};
    // Send a partial "junk" packet first
    Codec::encode((uint8_t *)"ignored", 7, encode_buf);
    for (uint16_t i = 0; i < 5; i++)
      decoder.feed(encode_buf[i], decode_buf);

    // Immediately follow with a valid packet (without a reset)
    // This tests if the 'S' byte correctly forces a state machine reset
    uint16_t enc_len_valid = Codec::encode((uint8_t *)"recovered", 9, encode_buf);
    uint16_t final_len = 0;
    for (uint16_t i = 0; i < enc_len_valid; i++) {
      final_len = decoder.feed(encode_buf[i], decode_buf);
    }
    if (final_len == 9)
      passed++;
  }

  // --- TEST 5: Maximum Length ---
  uint8_t max_p[128];
  memset(max_p, 0xAA, 128);
  if (run_single_test(max_p, 128))
    passed++;

  return passed;
}