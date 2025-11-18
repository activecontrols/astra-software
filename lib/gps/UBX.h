#pragma once

#include "UBX_DEFS.h"
#include <stdint.h>

class UBX;

template <typename t> struct GPS_Packet {
  friend class UBX;

public:
  t *data{0};
  bool updated = false;

  // this is kind of a dummy check that just checks if the first packet has been received
  bool is_valid()
  {
    return valid;
  }

private:
  bool valid = false;
  t a;
  t b;
  t *new_data = &a;
  void swap() {
    data = new_data;

    new_data = new_data == &a ? &b : &a;

    valid = true;
    updated = true;
  }
};

class UBX {

public:
  void encode(uint8_t);

  GPS_Packet<UBX_NAV_PVT> pvt_solution;
  GPS_Packet<UBX_NAV_COV> cov;

  UBX();

private:
  void reset_state();

  enum States {
    PRE_PAYLOAD, // is in pre-payload or waiting for preamble
    PAYLOAD,     // is processing payload and is not in the middle of a term
    ENCODE_TERM, // is in a term (in payload)
    CHECKSUM     // is past the payload and needs to verify checksum
  };
  States state;
  // encoder state variables

  int frame_position;

  uint8_t CK_A; // checksum a
  uint8_t CK_B; // checksum b

  uint8_t msg_CK_A; // CK_A to compare with (from message)
  uint8_t msg_CK_B; // CK_B to compare with (from message)

  uint8_t message_class;
  uint8_t message_id;
  uint8_t payload_length;

  // destination and size of the term currently being processed
  void *term_location;
  unsigned int term_size;
  unsigned int term_index;

  void encode_pre_payload(uint8_t);
  void encode_payload(uint8_t);
  void encode_term(uint8_t);
  void checksum(uint8_t);
};