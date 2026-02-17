#include <Arduino.h>
#include <stm32h747xx.h>

#include "Router.h"
#include "flash.h"

#include "flash_defs.h"

#define FREQ_MAX (100'000'000)

#define HAL_TIMEOUT 1000

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
bool read_qpiid(uint8_t out[3]) {
  QSPI_CommandTypeDef cmd{};

  cmd.Instruction = CMD_QPIID;
  cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;
  cmd.AddressMode = QSPI_ADDRESS_NONE;
  cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  cmd.DummyCycles = 6; // 6 dummy cycles because this command (and this command only, for some reason) always returns the first 4 bits all high even when they shouldn't be, so we skip until the flash
                       // repeats the data
  cmd.DataMode = QSPI_DATA_4_LINES;
  cmd.NbData = 3;
  cmd.DdrMode = 0;

  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT) != HAL_OK) {
    return false;
  }

  return HAL_QSPI_Receive(&hqspi, out, HAL_TIMEOUT) == HAL_OK;
}

// read status register (RDSR)
bool read_status(flash_status_reg *out) {
  QSPI_CommandTypeDef cmd{};

  cmd.Instruction = CMD_RDSR;
  cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;
  cmd.DataMode = QSPI_DATA_4_LINES;
  cmd.NbData = 1;

  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT) != HAL_OK) {
    return false;
  }

  return HAL_QSPI_Receive(&hqspi, (uint8_t *)out, HAL_TIMEOUT) == HAL_OK;
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
  QSPI_CommandTypeDef cmd{};

  cmd.Instruction = CMD_WREN;
  cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;

  return HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT) == HAL_OK;
}

bool write_disable()
{
  QSPI_CommandTypeDef cmd{};

  cmd.Instruction = CMD_WRDI;
  cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;

  return HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT) == HAL_OK;
}

// !!! EXERCISE CAUTION: THIS COMMAND WILL ERASE THE ENTIRE CHIP !!!
// this function blocks until the erase is complete
bool erase() {
  while (!is_write_enable()) {
    write_enable();
    delay(1);
  }

  while (is_write_in_progress()) {
    delay(1);
  }

  


  while (is_write_in_progress()) {
    delay(1);
  }

  while (is_write_enable())
  {
    write_disable();
    delay(1);
  }
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

void begin() {

  if (!_initialize()) {
    while (1) {
      Router::print("QSPI Init Failed\n");
      delay(1000);
    }
  }

  Router::print("Flash driver ready.\n");

  return;
}
}; // namespace Flash