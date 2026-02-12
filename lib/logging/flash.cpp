#include <Arduino.h>
#include <stm32h747xx.h>

#include "Router.h"
#include "flash.h"

#include "flash_defs.h"

#define FREQ_MAX (50'000000)

#define HAL_TIMEOUT 1000

void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi) {
  __HAL_RCC_QSPI_CLK_ENABLE();

  __HAL_RCC_QSPI_FORCE_RESET();

  __HAL_RCC_QSPI_RELEASE_RESET();

  __HAL_RCC_GPIOG_CLK_ENABLE();

  __HAL_RCC_GPIOD_CLK_ENABLE();

  __HAL_RCC_GPIOF_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init{};

  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;

  gpio_init.Pin = GPIO_PIN_7 | GPIO_PIN_10;
  gpio_init.Alternate = GPIO_AF9_QUADSPI;

  HAL_GPIO_Init(GPIOF, &gpio_init);

  gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  gpio_init.Alternate = GPIO_AF9_QUADSPI;
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
  cmd.DataMode = QSPI_DATA_NONE; // data on only a single line
  cmd.NbData = 0;
  cmd.DdrMode = 0;

  // when sending a command without data the command initiates when HAL_QSPI_Command is called
  return HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT) == HAL_OK;
}


// read electronic id over quadspi
// returns true if peripheral did not error
bool read_qpiid(uint8_t out[3])
{
  QSPI_CommandTypeDef cmd{};

  cmd.Instruction = CMD_QPIID;
  cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;
  cmd.AddressMode = QSPI_ADDRESS_NONE;
  cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  cmd.DummyCycles = 0;
  cmd.DataMode = QSPI_DATA_4_LINES;
  cmd.NbData = 3;
  cmd.DdrMode = 0;

  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT) != HAL_OK)
  {
    return false;
  }

  return HAL_QSPI_Receive(&hqspi, out, HAL_TIMEOUT) == HAL_OK;
}

void begin() {

  uint32_t f_hclk = HAL_RCC_GetHCLKFreq();

  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = f_hclk / FREQ_MAX - 1;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE; // my testing showed that some bit errors occured when I didn't use half cycle
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_4_CYCLE;
  hqspi.Init.FlashSize = 24; // corresponds with the number of bits required to address the entire memory region
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;

  HAL_StatusTypeDef status = HAL_QSPI_Init(&hqspi);

  if (status != HAL_OK) {
    Router::print("HAL_QSPI_Init failed.\n");
    return;
  }

  Router::print("QSPI Peripheral Initialized.\n");
  Router::print("QSPI Frequency: ");
  Router::print(f_hclk / (hqspi.Init.ClockPrescaler + 1) * 1e-6, 2);
  Router::print(" MHz\n");

  if (!enable_qspi())
  {
    Router::print("Flash enable QPIO command (EQIO) failed.\n");
    return;
  }

  uint8_t qpiid[3];
  const uint8_t correct_qpiid[3] = {0xC2, 0x20, 0x18};
  if (!read_qpiid(qpiid))
  {
    Router::print("Read QPIID failed.\n");
    return;
  }

  Router::print("QPIID: ");
  Router::print(qpiid[0], HEX);
  Router::print(' ');
  Router::print(qpiid[1], HEX);
  Router::print(' ');
  Router::print(qpiid[2], HEX);
  Router::print('\n');

  if (memcmp(qpiid, correct_qpiid, sizeof(qpiid)) != 0)
  {
    Router::print("QPIID response invalid.\n");
    return;
  }

  Router::print("Flash driver ready.\n");

  return;
}
}; // namespace Flash