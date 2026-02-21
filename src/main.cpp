#include "GPS.h"
#include "IMU.h"
#include "Mag.h"
#include "Prop.h"
#include "Router.h"
#include "TrajectoryFollower.h"
#include "TrajectoryLoader.h"
#include "gimbal_servos.h"
#include <Arduino.h>

// TODO - printf not supporting floats / doubles

// TODO - this should be included - don't know why I had to write it manually
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SPI123;
  PeriphClkInit.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

void ping(const char *args) {
  Router::println("pong");
  Router::print("args: ");
  Router::println(args == nullptr ? "null" : args);
}

void setup() {
  delay(3000);
  fc_spi.begin(); // spi is a shared interface, so we always begin here
  Router::begin();
  Router::println("Controller started.");

  digitalWrite(PB7, HIGH);
  digitalWrite(PC12, HIGH);

  digitalWrite(PD7, HIGH); // MAG CS 1
  digitalWrite(PE7, HIGH); // MAG CS 2
  digitalWrite(PE4, HIGH); // MAG CS 3

  digitalWrite(PB3, HIGH); // IMU CS 3
  digitalWrite(PB4, HIGH); // IMU CS 2
  digitalWrite(PB5, HIGH); // IMU CS 1

  // TODO - configure CS somewhere else!
  pinMode(PB7, OUTPUT);
  pinMode(PC12, OUTPUT);
  pinMode(PE7, OUTPUT);
  pinMode(PE4, OUTPUT);

  pinMode(PB3, OUTPUT);
  pinMode(PB4, OUTPUT);
  pinMode(PB5, OUTPUT);
  pinMode(PD7, OUTPUT);

  pinMode(PB6, OUTPUT);

  digitalWrite(IMU_CS_4_EX, HIGH);

  while (1) {
    for (int i = 0; i < IMU::IMU_COUNT; ++i) {
      uint32_t test_cs = IMU_CS[i];
      digitalWrite(test_cs, LOW);
      digitalWrite(IMU_CS_4_EX, LOW);
      fc_spi.beginTransaction(SPISettings(1'000'000, MSBFIRST, SPI_MODE0));

      // fc_spi.transfer(0x2F | (1 << 7)); // read MAG digital id - should respond with 0x30
      fc_spi.transfer(0x75 | (1 << 7)); // read WHOAMI - IMU should respond with 0x3B
      uint8_t response = fc_spi.transfer(0x00);

      fc_spi.endTransaction();
      digitalWrite(IMU_CS_4_EX, HIGH);
      digitalWrite(test_cs, HIGH);

      Router::printf("IMU %d: ", i + 1);
      Router::println(response, HEX);

      delay(1000);
    }

    for (int i = 0; i < Mag::MAG_COUNT; ++i) {
      uint32_t test_cs = MAG_CS[i];
      digitalWrite(test_cs, LOW);
      digitalWrite(IMU_CS_4_EX, LOW);
      fc_spi.beginTransaction(SPISettings(1'000'000, MSBFIRST, SPI_MODE0));

      fc_spi.transfer(0x2F | (1 << 7)); // read MAG digital id - should respond with 0x30
      // fc_spi.transfer(0x75 | (1 << 7)); // read WHOAMI - IMU should respond with 0x3B
      uint8_t response = fc_spi.transfer(0x00);

      fc_spi.endTransaction();
      digitalWrite(IMU_CS_4_EX, HIGH);
      digitalWrite(test_cs, HIGH);

      Router::printf("MAG %d: ", i + 1);
      Router::println(response, HEX);

      delay(1000);
    }
  }

  // Prop::begin();
  // Mag::begin();
  // GPS::begin();
  // IMU::begin();
  // GimbalServos::begin();
  // TrajectoryLoader::begin();
  // TrajectoryFollower::begin();

  Router::add({ping, "ping"}); // example registration
  Router::add({Router::print_all_cmds, "help"});
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}