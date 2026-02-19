#include <Arduino.h>
#include <stm32h747xx.h>

#include "Router.h"
#include "flash.h"

#include "flash_defs.h"

#define FREQ_MAX 50'000'000

// 1ms
#define HAL_TIMEOUT 1

// proper practice here is to always keep WEL disabled (write enable latch, which should be kept low to prevent accidental writes or erases)
// each function that modifies data on the flash should first enable WEL, write, then disable WEL before returning

void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi) {
  __HAL_RCC_QSPI_CLK_ENABLE();

  __HAL_RCC_QSPI_FORCE_RESET();

  __HAL_RCC_QSPI_RELEASE_RESET();

  __HAL_RCC_GPIOG_CLK_ENABLE();

  __HAL_RCC_GPIOD_CLK_ENABLE();

  __HAL_RCC_GPIOF_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init;

  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  gpio_init.Pin = GPIO_PIN_7 | GPIO_PIN_10;
  gpio_init.Alternate = GPIO_AF9_QUADSPI;

  HAL_GPIO_Init(GPIOF, &gpio_init);

  gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  gpio_init.Alternate = GPIO_AF9_QUADSPI; // or AF10 depending on part
  HAL_GPIO_Init(GPIOD, &gpio_init);

  gpio_init.Pin = GPIO_PIN_6;
  gpio_init.Alternate = GPIO_AF10_QUADSPI;

  HAL_GPIO_Init(GPIOG, &gpio_init);
}

namespace Flash {
QSPI_HandleTypeDef hqspi;

bool command(uint8_t instruction, unsigned int data_len = 0, uint32_t addr = static_cast<uint32_t>(-1), unsigned int dummy = 0) {
  QSPI_CommandTypeDef cmd{};

  cmd.Instruction = instruction;
  cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;
  cmd.DummyCycles = dummy;

  if (data_len) {
    cmd.NbData = data_len;
    cmd.DataMode = QSPI_DATA_4_LINES;
  }

  if (addr != static_cast<uint32_t>(-1)) {
    cmd.Address = addr;
    cmd.AddressSize = QSPI_ADDRESS_24_BITS;
    cmd.AddressMode = QSPI_ADDRESS_4_LINES;
  }

  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT) != HAL_OK)
    return false;

  return true;
}

bool read(uint8_t instruction, void *data, unsigned int len, uint32_t addr = static_cast<uint32_t>(-1), unsigned int dummy = 0) {
  if (!command(instruction, len, addr, dummy))
    return false;

  return HAL_QSPI_Receive(&hqspi, static_cast<uint8_t *>(data), HAL_TIMEOUT) == HAL_OK;
}

// note that this function WILL NOT wait for the write operation to complete; you need to check WIP bit separately
bool write(uint8_t instruction, void *data, unsigned int len, uint32_t addr = static_cast<uint32_t>(-1), unsigned int dummy = 0) {
  if (!command(instruction, len, addr, dummy))
    return false;

  return HAL_QSPI_Transmit(&hqspi, static_cast<uint8_t *>(data), HAL_TIMEOUT) == HAL_OK;
}

// returns true if peripheral did not error
bool enable_qspi() {
  QSPI_CommandTypeDef cmd{};

  cmd.Instruction = CMD_EQIO;                        // enable QSPI command - EQIO
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;     // InstructionMode 1 corresponds to single serial mode - use 1 pin for output and 1 pin for input
  cmd.AddressMode = QSPI_ADDRESS_NONE;               // skip sending address
  cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE; // skip alternate bytes
  cmd.DummyCycles = 0;
  cmd.DataMode = QSPI_DATA_NONE;
  cmd.NbData = 0;
  cmd.DdrMode = 0;

  // when sending a command without data the command initiates when HAL_QSPI_Command is called
  return HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT) == HAL_OK;
}

// read electronic id over quadspi
// returns true if peripheral did not error
inline bool read_qpiid(uint8_t out[3]) {
  return read(CMD_QPIID, out, sizeof(out), -1, 6);
}
// read status register (RDSR)
inline bool read_status(flash_status_reg *out) {
  return read(CMD_RDSR, out, sizeof(*out));
}

// read security register (RDSCUR)

// wait for write-in-progress bit to be unset
bool wait_for_wip(unsigned long max_delay = 50) {
  flash_status_reg reg;

  unsigned long start_micros = micros();
  do {
    unsigned long now_micros = micros();

    // deal with unlikely overflow error (that can occur if fc is running for about 70 minutes)
    if (now_micros < start_micros && (static_cast<unsigned int>(-1) - start_micros + now_micros > max_delay))
      return false;
    if (now_micros - start_micros > max_delay)
      return false;

    read_status(&reg);
    delayMicroseconds(10);
  } while (reg.WIP);
}

bool is_write_enable() {
  flash_status_reg reg;

  if (!read_status(&reg))
    return false;

  return reg.WEL;
}

bool is_write_in_progress() {
  flash_status_reg reg;

  if (!read_status(&reg))
    return true;

  return reg.WIP;
}

bool write_enable() {
  if (!command(CMD_WREN))
    return false;

  wait_for_wip();
  return true;
}

// note: this function waits for WIP to be unset before returning
bool write_disable() {
  QSPI_CommandTypeDef cmd{};

  cmd.Instruction = CMD_WRDI;
  cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;

  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT) != HAL_OK)
    return false;

  wait_for_wip();
  return true;
}

// !!! EXERCISE CAUTION: THIS COMMAND WILL ERASE THE ENTIRE CHIP !!!
// this function blocks until the erase is complete
bool chip_erase() {
  wait_for_wip();
  write_enable();

  // check that write enable bit is set
  if (!is_write_enable())
    return false;

  if (command(CMD_CE) != HAL_OK)
    return false;

  wait_for_wip();  // wait for chip erase operation to complete (takes about 60 seconds)
  write_disable(); // disable write to protect against accidental data corruption

  // check the write enable bit is unset - even though the erase operation probably succeeded, still return false to indicate something is wrong
  if (is_write_enable())
    return false;

  return true;
}

// erase a 4KB sector
bool sector_erase(uint32_t addr) {
  wait_for_wip();
  write_enable();

  // check that write enable bit is set
  if (!is_write_enable())
    return false;

  if (!command(CMD_SE, 0, addr))
    return false;

  wait_for_wip();  // wait for erase operation to complete
  write_disable(); // disable write to protect against accidental data corruption

  // check the write enable bit is unset - even though the erase operation probably succeeded, still return false to indicate something is wrong
  if (is_write_enable())
    return false;

  return true;
}

// as per DS - only last 256 bytes are actually written, and the write address wraps back around the the beginning of the 256-byte page when necessary
// chain function calls together to write data across multiple pages
bool page_program(uint8_t addr, uint8_t *data, uint32_t len) {
  if (!write(CMD_4PP, data, len, addr))
    return false;
}

bool _initialize() {
  uint32_t f_hclk = HAL_RCC_GetHCLKFreq();

  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = f_hclk / FREQ_MAX;

  if (!(f_hclk % FREQ_MAX)) // "round up" when needed
  {
    --hqspi.Init.ClockPrescaler;
  }
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_8_CYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;

  HAL_StatusTypeDef status = HAL_QSPI_Init(&hqspi);

  if (status != HAL_OK) {
    Router::print("HAL_QSPI_Init failed.\n");
    return false;
  }

  Router::print("QSPI Peripheral Initialized.\n");
  Router::printf("HCLK Frequency: %d\n", f_hclk);
  Router::printf("QSPI Prescaler: %d\n", hqspi.Init.ClockPrescaler);
  Router::print("QSPI Frequency: ");
  Router::print(f_hclk / (hqspi.Init.ClockPrescaler + 1) * 1e-6, 2);
  Router::print(" MHz\n");

  if (!enable_qspi()) {
    Router::print("Flash enable QPIO command (EQIO) failed.\n");
    return false;
  }

  HAL_QSPI_StateTypeDef state = HAL_QSPI_GetState(&hqspi);

  Router::print("QSPI Peripheral State: ");
  Router::print(state, HEX);
  Router::print('\n');

  uint8_t qpiid[3];
  const uint8_t correct_qpiid[3] = {0xC2, 0x20, 0x18};
  if (!read_qpiid(qpiid)) {
    Router::print("Read QPIID failed.\n");
    return false;
  }

  Router::print("QPIID: ");
  for (int i = 0; i < sizeof(qpiid); ++i) {
    Router::print(qpiid[i], HEX);
    Router::print(' ');
  }
  Router::print('\n');

  if (memcmp(qpiid, correct_qpiid, sizeof(correct_qpiid)) != 0) {
    Router::print("QPIID response invalid.\n");
    return false;
  }

  return true;
}

void write_test() {
  Router::print("Write test commencing.\n");

  if (!sector_erase(0)) {
    Router::print("Sector erase command failed.\n");
    return;
  }
  Router::print("Sector erased and WIP unset.\n");

  // program sector
  char program_data[256];
  char str[] = "Hello, world!";
  memset(program_data, 0, sizeof(program_data));
  memcpy(program_data, str, sizeof(str));

  Router::print("Programming page 0\n");
  unsigned long start_micros = micros();
  if (!page_program(0, reinterpret_cast<uint8_t *>(program_data), sizeof(program_data))) {
    Router::print("Page program command Failed.\n");
    return;
  }

  // wait for write operation to finish
  wait_for_wip();

  unsigned long delta_t = micros() - start_micros;
  Router::print("Page program command succeeded.\n");
  Router::print("WIP Unset.\n");
  Router::printf("Single block write time (us): %d\n", delta_t);

  Router::print("Reading page 0\n");

  char data_read[sizeof(program_data)];
  if (!read(CMD_4READ, data_read, sizeof(data_read), 0, 6)) {
    Router::print("4READ command failed\n");
    return;
  }

  if (memcmp(data_read, program_data, sizeof(program_data)) != 0) {
    Router::print("Error: 4READ data does not match program data.\n");
  }

  Router::print(data_read);
  Router::print('\n');
}

void begin() {

  if (!_initialize()) {
    while (1) {
      Router::print("QSPI Init Failed\n");
      delay(1000);
    }
  }

  Router::print("Flash driver ready.\n");
  Router::add({write_test, "write_test"});

  return;
}
}; // namespace Flash