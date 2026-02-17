#pragma once

#define CMD_EQIO 0x35
#define CMD_RDID 0x9F
#define CMD_QPIID 0xAF
#define CMD_RES 0xAB
#define CMD_WREN 0x06
#define CMD_WRDI 0x04
#define CMD_RDSR 0x05

struct flash_status_reg {
  uint8_t WIP : 1;
  uint8_t WEL : 1;
  uint8_t BP0 : 1;
  uint8_t BP1 : 1;
  uint8_t BP2 : 1;
  uint8_t BP3 : 1;
  uint8_t QE : 1;
  uint8_t SRWD : 1;
};


struct flash_config_reg 
{
    uint8_t ODS : 3;
    uint8_t TB : 1;
    uint8_t RESERVED : 2;
    uint8_t DC : 2;
};

struct flash_security_reg
{
    uint8_t factory_otp_indicator : 1;
    uint8_t user_otp_indicator : 1;
    uint8_t PSB : 1;
    uint8_t ESB : 1;
    uint8_t RESERVED : 1;
    uint8_t P_FAIL : 1;
    uint8_t E_FAIL : 1;
    uint8_t WPSEL : 1;
};