#include <stdio.h>

#define MAX_PACKET_SIZE 256
char compressed_data[MAX_PACKET_SIZE];

template <typename S> void print_compressed(const char *id, S data) {
  printf("%s", id);

  // need an extra byte for every 7 bits
  int extra_bytes_needed = (sizeof(data) + 6) / 7; // equivalent to ceil(size / 7)
  char *raw_data = (char *)&data;                  // allows [] index

  for (size_t i = 0; i < sizeof(data); i += 7) {
    compressed_data[sizeof(data) + i / 7] = 0x80; // wipe all the extra bytes
  }

  for (size_t i = 0; i < sizeof(data); i++) {
    compressed_data[i] = raw_data[i] | 0x80; // set all the upper bits
    if (raw_data[i] & 0x80) {
      compressed_data[sizeof(data) + i / 7] |= 1 << (6 - (i % 7));
    }
  }

  for (size_t i = 0; i < sizeof(data) + extra_bytes_needed; i++) {
    printf("%02x ", (unsigned char)compressed_data[i]);
  }

  printf("\n");
}

struct my_sample_packet {
  float a;
  float b;
  int c;
  char msg[3];
};

int main() {
  my_sample_packet tx = {}; // ensure any padding bytes get init to 0
  tx.a = 5.3;
  tx.b = 6.3;
  tx.c = 7;
  tx.msg[0] = 'a';
  tx.msg[1] = 'b';
  tx.msg[2] = '\0';
  print_compressed("#a>", tx);
}