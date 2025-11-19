#pragma once
#include <stdint.h>

#define CLASS_NAV 0x01
#define CLASS_INF 0x04

// class NAV
#define ID_PVT 0x07
#define ID_COV 0x36

// class INF
#define ID_DEBUG 0x04
#define ID_ERROR 0x00
#define ID_NOTICE 0x02
#define ID_TEST 0x03
#define ID_WARNING 0x01

struct __attribute__((packed)) UBX_NAV_PVT {
  uint32_t iTOW; // ms
  uint16_t year; // y
  uint8_t month; // month
  uint8_t day;   // d
  uint8_t hour;  // h
  uint8_t min;   // min
  uint8_t sec;   // s
  uint8_t valid;
  uint32_t tAcc; // ns
  int32_t nano;  // ns
  uint8_t fixType;
  uint8_t flags;
  uint8_t flags2;
  uint8_t numSV;
  int32_t lon;      // deg (1e-7)
  int32_t lat;      // deg (1e-7)
  int32_t height;   // mm
  int32_t hMSL;     // mm
  uint32_t hAcc;    // mm
  uint32_t vAcc;    // mm
  int32_t velN;     // mm/s
  int32_t velE;     // mm/s
  int32_t velD;     // mm/s
  int32_t gSpeed;   // mm/s
  int32_t headMot;  // deg (1e-5)
  uint32_t sAcc;    // mm/s
  uint32_t headAcc; // deg (1e-5)
  uint16_t pDOP;
  uint16_t flags3;
  uint8_t reserved0[4];
  int32_t headVeh; // deg (1e-5)
  int16_t magDec;  // deg (1e-2)
  uint16_t magAcc; // deg (1e-2)
};
static_assert(sizeof(UBX_NAV_PVT) == 92U, "UBX_NAV_PVT packet payload must be 92 bytes.");

struct __attribute__((packed)) UBX_NAV_COV {
  uint32_t iTOW; // ms
  uint8_t version;
  uint8_t posCovValid;
  uint8_t velCovValid;
  uint8_t reserved0[9];
  float posCovNN; // m^2
  float posCovNE; // m^2
  float posCovND; // m^2
  float posCovEE; // m^2
  float posCovED; // m^2
  float posCovDD; // m^2
  float velCovNN; // m^2/s^2
  float velCovNE; // m^2/s^2
  float velCovND; // m^2/s^2
  float velCovEE; // m^2/s^2
  float velCovED; // m^2/s^2
  float velCovDD; // m^2/s^2
};
static_assert(sizeof(UBX_NAV_COV) == 64, "UBX_NAV_COV packet payload must be 64 bytes.");