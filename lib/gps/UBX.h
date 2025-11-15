#include <stdint.h>

struct UBX_NAV_PVT {
  uint8_t numSV; // number of satellites
  int32_t lon;   // longitude (degrees 1e-7)
  int32_t lat;   // latitude (degrees 1e-7)
  int32_t hMSL;  // height above mean sea level (mm)
  int32_t velN;  // velocity north (mm/s)
  int32_t velE;  // velocity east (mm/s)
  int32_t velD;  // velocity down (mm/s)
};

class UBX {

public:
  UBX_NAV_PVT pvt_solution{0};

  void encode(uint8_t);
  bool isValid();

  bool updated;

  UBX();

private:
  void reset_state();

  bool valid;

  UBX_NAV_PVT new_pvt_solution{0};

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