#include "UBX.h"
#include <cstring>

UBX::UBX() {
  reset_state();
  updated = false;
  valid = false;
}

bool UBX::isValid() {
  return valid;
}

void UBX::reset_state() {
  state = PRE_PAYLOAD;
  frame_position = 0;
  CK_A = 0;
  CK_B = 0;
}

void UBX::encode_pre_payload(uint8_t x) {

  switch (frame_position) {
  case 0:
    if (x != 0xb5) {
      reset_state();
      frame_position = -1;
      return;
    }
    break;
  case 1:
    if (x != 0x62) {
      reset_state();
      if (x != 0xb5) {
        frame_position = -1;
      }
      return;
    }
    break;
  case 2:
    message_class = x;
    break;
  case 3:
    message_id = x;
    break;
  case 4:
    payload_length = x;
    break;
  case 5:
    payload_length |= x << 8;
    state = PAYLOAD;
    break;
  }
}

void UBX::encode_payload(uint8_t x) {
  int payload_position = frame_position - 6; // the first 6 bytes of the frame are pre-payload

  if (message_class == 0x01 && message_id == 0x07) // UBX-NAV-PVT
  {
    state = ENCODE_TERM;
    term_index = 0;

    switch (payload_position) {
    case 23: // numSV
      term_location = &new_pvt_solution.numSV;
      term_size = sizeof(new_pvt_solution.numSV);
      break;
    case 24: // longitude
      term_location = &new_pvt_solution.lon;
      term_size = sizeof(new_pvt_solution.lon);
      break;
    case 28: // latitude
      term_location = &new_pvt_solution.lat;
      term_size = sizeof(new_pvt_solution.lat);
      break;
    case 36:
      term_location = &new_pvt_solution.hMSL;
      term_size = sizeof(new_pvt_solution.hMSL);
      break;
    case 48: // velN
      term_location = &new_pvt_solution.velN;
      term_size = sizeof(new_pvt_solution.velN);
      break;
    case 52: // velE
      term_location = &new_pvt_solution.velE;
      term_size = sizeof(new_pvt_solution.velE);
      break;
    case 56: // velD
      term_location = &new_pvt_solution.velD;
      term_size = sizeof(new_pvt_solution.velD);
      break;
    default:
      state = PAYLOAD;
      // we will ignore this term
      return;
    }
    encode_term(x);
  }
}

void UBX::encode_term(uint8_t x) {
  *((uint8_t *)term_location + term_index) = x;

  ++term_index;

  if (term_index >= term_size) {
    state = PAYLOAD;
    return;
  }
}

void UBX::checksum(uint8_t x) {
  if (frame_position == 6 + payload_length) {
    msg_CK_A = x;
    return;
  }

  msg_CK_B = x;

  // compare checksum
  if (!(msg_CK_A == CK_A && msg_CK_B == CK_B)) {
    // invalid checksum; "throw out" the data and wait for another packet
    reset_state();
    return;
  }
  if (message_class == 0x01 && message_id == 0x07) // UBX-NAV-PVT
  {
    // copy values over
    memcpy(&pvt_solution, &new_pvt_solution, sizeof(UBX_NAV_PVT));
    // set flags
    updated = true;
    valid = true;
  }

  reset_state();
  frame_position = -1;
}

void UBX::encode(uint8_t x) {
  // checksum operation
  if (frame_position > 1 && (state == PRE_PAYLOAD || frame_position < 6 + payload_length)) {
    CK_A += x;
    CK_B += CK_A;
  }

  switch (state) {
  case PRE_PAYLOAD:
    encode_pre_payload(x);
    break;
  case PAYLOAD:
    encode_payload(x);
    break;
  case ENCODE_TERM:
    encode_term(x);
    break;
  case CHECKSUM:
    checksum(x);
    break;
  }

  ++frame_position;

  if (state != PRE_PAYLOAD && frame_position >= payload_length + 6) {
    // end of payload has been reached
    state = CHECKSUM;
    return;
  }
}