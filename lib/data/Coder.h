#pragma once

#include <cstring> // for memcpy

typedef struct {
  float f1;
  float f2;
  float f3;
  char c1;
} placeholder;

// easy to change which struct is being encoded/decoded
#define CODE_T placeholder

// can change to char, short, int, long etc for different reliability/length tradeoffs
#define CSUM_T int

namespace Coder {
int get_packet_size() { // todo: when i tried size_t for ret type it told me i needed to include stddef.h
  return sizeof(CODE_T) + sizeof(CSUM_T);
}

// encodes into buffer. buffer should be get_packet_size() bytes long.
void encode_packet(const CODE_T &data, char *buffer) {
  memcpy(buffer, &data, sizeof(CODE_T));

  CSUM_T csum = 0;
  const char *data_bytes = (const char *)&data;
  for (size_t i = 0; i < sizeof(CODE_T); i++) {
    csum += data_bytes[i];
  }

  memcpy(buffer + sizeof(CODE_T), &csum, sizeof(CSUM_T));
}

// decodes into data. buffer should be get_packet_size() bytes long.
bool decode_packet(const char *buffer, CODE_T &data) {
  CSUM_T received_csum;
  memcpy(&received_csum, buffer + sizeof(CODE_T), sizeof(CSUM_T));

  memcpy(&data, buffer, sizeof(CODE_T));

  CSUM_T csum = 0;
  const char *data_bytes = (const char *)&data;
  for (size_t i = 0; i < sizeof(CODE_T); i++) {
    csum += data_bytes[i];
  }

  return received_csum == csum;
}

bool test() {
  CODE_T original;
  original.f1 = 1.23f;
  original.f2 = 4.56f;
  original.f3 = 7.89f;
  original.c1 = 'A';

  const int packet_size = get_packet_size();
  char *buffer = new char[packet_size];

  encode_packet(original, buffer);

  CODE_T decoded;
  bool success = decode_packet(buffer, decoded) && (original.f1 == decoded.f1) && (original.f2 == decoded.f2) && (original.f3 == decoded.f3) && (original.c1 == decoded.c1);

  buffer[3]++; // corrupt.
  bool fail_detected = !decode_packet(buffer, decoded);
  return success && fail_detected;
}

} // namespace Coder