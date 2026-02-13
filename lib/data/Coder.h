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

  // thank u llm.
  // ------------------------------------------------------------
  // Self-test: exercises encode/decode and streaming decoder.
  // Returns true iff all checks pass.
  // ------------------------------------------------------------
  static inline bool self_test() { // confirmed to return true on feb 12 8 pm EST.
    auto memeq = [](const uint8_t *a, const uint8_t *b, size_t n) -> bool {
      for (size_t i = 0; i < n; i++)
        if (a[i] != b[i])
          return false;
      return true;
    };

    // 1) Roundtrip (full-frame decode) for several patterns
    const uint8_t patterns[][3] = {
        {0x00, 0x00, 0x00}, {0xFF, 0xFF, 0xFF}, {DS, DE, ESC}, // force escaping
        {ESC, DS, DE},      {0x12, 0x34, 0x56}, {0xA5, 0x5A, 0xC3},
    };

    for (size_t p = 0; p < sizeof(patterns) / sizeof(patterns[0]); p++) {
      T in{};
      uint8_t *in_bytes = reinterpret_cast<uint8_t *>(&in);
      // Fill struct bytes deterministically; repeats the 3-byte pattern.
      for (size_t i = 0; i < sizeof(T); i++)
        in_bytes[i] = patterns[p][i % 3];

      uint8_t framed[max_framed_size()];
      size_t n = encode_framed(in, framed);

      if (n < 2)
        return false;
      if (framed[0] != DS)
        return false;
      if (framed[n - 1] != DE)
        return false;

      T out{};
      if (!decode_framed(framed, n, out))
        return false;
      if (!memeq(reinterpret_cast<const uint8_t *>(&in), reinterpret_cast<const uint8_t *>(&out), sizeof(T)))
        return false;
    }

    // 2) Streaming decoder: same framed bytes, fed byte-by-byte, should yield one packet.
    {
      T in{};
      uint8_t *in_bytes = reinterpret_cast<uint8_t *>(&in);
      for (size_t i = 0; i < sizeof(T); i++)
        in_bytes[i] = (uint8_t)(i * 131u + 7u); // deterministic

      uint8_t framed[max_framed_size()];
      size_t n = encode_framed(in, framed);

      Decoder dec;
      T out{};
      bool got = false;

      for (size_t i = 0; i < n; i++) {
        if (dec.feed(framed[i], out)) {
          if (got)
            return false; // should not produce twice for one frame
          got = true;
        }
      }
      if (!got)
        return false;
      if (!memeq(reinterpret_cast<const uint8_t *>(&in), reinterpret_cast<const uint8_t *>(&out), sizeof(T)))
        return false;
    }

    // 3) Corruption test: flip one interior byte => checksum should fail (both decode paths).
    {
      T in{};
      uint8_t *in_bytes = reinterpret_cast<uint8_t *>(&in);
      for (size_t i = 0; i < sizeof(T); i++)
        in_bytes[i] = (uint8_t)(0x3C + i);

      uint8_t framed[max_framed_size()];
      size_t n = encode_framed(in, framed);

      if (n < 4)
        return false;    // need an interior byte to flip
      framed[1] ^= 0x01; // flip a bit (may land on ESC sometimes; still should fail or desync)

      T out{};
      if (decode_framed(framed, n, out))
        return false;

      Decoder dec;
      bool got = false;
      for (size_t i = 0; i < n; i++) {
        if (dec.feed(framed[i], out))
          got = true;
      }
      if (got)
        return false;
    }

    // 4) Resync test: noise + DS mid-stream should restart cleanly and decode the later good frame.
    {
      T in{};
      uint8_t *in_bytes = reinterpret_cast<uint8_t *>(&in);
      for (size_t i = 0; i < sizeof(T); i++)
        in_bytes[i] = (uint8_t)(0x90 ^ (uint8_t)i);

      uint8_t framed[max_framed_size()];
      size_t n = encode_framed(in, framed);

      Decoder dec;
      T out{};
      bool got = false;

      // feed some garbage (including an unescaped DS to force a restart)
      const uint8_t noise[] = {0x11, 0x22, DS, 0x33, 0x44, 0x55};
      for (size_t i = 0; i < sizeof(noise); i++)
        dec.feed(noise[i], out);

      // then feed a clean frame
      for (size_t i = 0; i < n; i++) {
        if (dec.feed(framed[i], out)) {
          got = true;
          break;
        }
      }

      if (!got)
        return false;
      if (!memeq(reinterpret_cast<const uint8_t *>(&in), reinterpret_cast<const uint8_t *>(&out), sizeof(T)))
        return false;
    }

    // 5) Dangling escape in framed buffer should be rejected by decode_framed.
    {
      T in{};
      uint8_t *in_bytes = reinterpret_cast<uint8_t *>(&in);
      for (size_t i = 0; i < sizeof(T); i++)
        in_bytes[i] = (uint8_t)(0x42 + i);

      uint8_t framed[max_framed_size()];
      size_t n = encode_framed(in, framed);

      // Force "dangling escape": make the last interior byte ESC.
      // framed[n-1] is DE, so framed[n-2] is last interior byte.
      framed[n - 2] = ESC;

      T out{};
      if (decode_framed(framed, n, out))
        return false;
    }

    return true;
  }
};