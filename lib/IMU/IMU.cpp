#include <Arduino.h>
#include <SPI.h>

#include "IMU.h"

#include "./Invn/Drivers/Icm406xx/Icm406xxDriver_HL.h"
#include "./Invn/Drivers/Icm406xx/Icm406xxTransport.h"
#include "./Invn/EmbUtils/ErrorHelper.h"

#include "Router.h"
#include "SDCard.h"
#include "portenta_pins.h"

#define WRITE_FLAG (0)
#define READ_FLAG (1 << 7)

// Unit: Hz
// !!! DO NOT EXCEED 24MHz !!!
#define SPI_RATE (1 * 1000000)

#define G_TO_MS2 9.80145

// g / LSB (this is for +-8g fsr)
constexpr double ACCEL_RESOLUTION = 1.0 / 4096.0;

// dps / LSB (this is for +- 2000dps fsr)
constexpr double GYRO_RESOLUTION = 1.0 / 16.4;

#define SPI_SETTINGS SPISettings(SPI_RATE, MSBFIRST, SPI_MODE0)

using namespace IMU;

Sensor IMU::IMUs[IMU_COUNT] = {Sensor(IMU_CS, &SPI)};

int read_reg(void *context, uint8_t reg, uint8_t *buf, uint32_t len);
int write_reg(void *context, uint8_t reg, const uint8_t *buf, uint32_t len);

void begin_transaction(SPI_Interface *my_spi);
void end_transaction(SPI_Interface *my_spi);

Sensor::Sensor(int cs, arduino::MbedSPI *spi) {
  this->spi_interface.cs = cs;
  this->spi_interface.spi = spi;

  this->clear_calib();

  this->inv_icm = malloc(sizeof(inv_icm406xx));
}

Sensor::~Sensor() {
  free(this->inv_icm);
}

int Sensor::init() {

  int status;

  inv_icm406xx_serif serif;

  digitalWrite(this->spi_interface.cs, HIGH); // write before setting mode to output
  pinMode(this->spi_interface.cs, OUTPUT);

  // configure the serial interface (serif) structure that is used by TDK's driver
  serif.context = &this->spi_interface;
  serif.read_reg = &read_reg;
  serif.write_reg = &write_reg;
  serif.max_read = 1024 * 32;
  serif.max_write = 1024 * 32;
  serif.serif_type = ICM406XX_UI_SPI4;

  status = inv_icm406xx_init((inv_icm406xx *)this->inv_icm, &serif, nullptr);

  if (status) {
    const char *error_message = inv_error_str(status);
    Router::printf("Error occurred while initializing IMU: %s\n", error_message);
    return status;
  }

  // tell IMU to report data in little endian !!!DO NOT REMOVE THIS LINE!!!
  status = inv_icm406xx_wr_intf_config0_data_endian((inv_icm406xx *)this->inv_icm, ICM406XX_INTF_CONFIG0_DATA_LITTLE_ENDIAN);

  if (status) {
    const char *error_message = inv_error_str(status);
    Router::printf("Error occurred while enabling little endian on IMU: %s\n", error_message);
    return status;
  }

  /* =========================================================================================
      ---- enable the anti-alias filter (this must be done while accel/gyro is disabled) ----
     =========================================================================================*/
  // switch to user bank 1
  status = inv_icm406xx_wr_reg_bank_sel((inv_icm406xx *)this->inv_icm, 1);

  // enable anti-alias filter on gyro
  status |= this->write_reg_mask(MPUREG_GYRO_CONFIG_STATIC2_B1, (1 << 1), 0);

  // switch to user bank 2
  inv_icm406xx_wr_reg_bank_sel((inv_icm406xx *)this->inv_icm, 2);

  // enable anti-alias filter on accelerometer
  status |= this->write_reg_mask(MPU_ACCEL_CONFIG_STATIC2_B2, (1 << 0), 0);

  // switch back to user bank 0 for normal operation
  inv_icm406xx_wr_reg_bank_sel((inv_icm406xx *)this->inv_icm, 0);

  /* =========================================================================================
      ---- done enabling anti-alias filter ----
     =========================================================================================*/

  if (status) {
    const char *error_message = inv_error_str(status);
    Router::printf("Error occurred while enabling anti-alias filter on IMU: %s\n", error_message);
    return status;
  }
  return status;
}

// loads three doubles from serial and stores them in dest
int load_calibration_helper(double *dest) {
  char buf[3][100];
  String line = Router::read(100);
  line.trim();
  Router::println(line);
  int read = sscanf(line.c_str(), "%99s %99s %99s", buf[0], buf[1], buf[2]);
  if (read != 3) {
    return read;
  }
  for (int i = 0; i < 3; ++i) {
    dest[i] = atof(buf[i]);
  }
  return read;
}

void Sensor::load_custom_calib() {
  Calib new_calib;

  const String prompts[] = {"Gyroscope bias", "Accelerometer bias", "Accelerometer gain"};
  double *destinations[] = {new_calib.gyro_bias, new_calib.accel_correction_bias, new_calib.accel_correction_gain};

  for (int i = 0; i < 3; ++i) {
    Router::printf("Enter %s (X Y Z) separated by spaces: ", prompts[i].c_str());
    int read = load_calibration_helper(destinations[i]);
    if (read != 3) {
      Router::printf("Only %d inputs read. Not saving calibration. Quitting.\n", read);
      return;
    }
  }

  memcpy(&this->calib, &new_calib, sizeof(Sensor::calib));
  Router::printf("Calibration written in memory. Remember to save calibration data to SD Card!!!");
}

int Sensor::read_accel_config(uint8_t *value) {
  return inv_icm406xx_read_reg((inv_icm406xx *)this->inv_icm, MPUREG_ACCEL_CONFIG0, 1, value);
}

void Sensor::calibrate_gyro() {
  int64_t sums[3];
  int16_t raw[3];
  memset(sums, 0, sizeof(sums));
  int count = 0;

  unsigned long start_time = millis();

  while (millis() - start_time < 20000) {
    this->read_latest_gyro_raw(raw);
    for (unsigned int i = 0; i < sizeof(sums) / sizeof(double); ++i) {
      sums[i] += raw[i];
    }

    ++count;
    delay(1);
  }

  for (unsigned int i = 0; i < sizeof(sums) / sizeof(double); ++i) {
    int16_t div = sums[i] / count;
    double frac = (double)(sums[i] % count) / count;
    this->calib.gyro_bias[i] = (div + frac) * GYRO_RESOLUTION;
  }

  return;
}

// output in LSBs
void Sensor::read_latest_gyro_raw(int16_t *out) {
  begin_transaction(&this->spi_interface);

  this->spi_interface.spi->transfer(READ_FLAG | MPUREG_GYRO_DATA_X1_UI);

  for (int i = 0; i < 3; ++i) {
    uint8_t lower = this->spi_interface.spi->transfer(0x00);
    uint8_t upper = this->spi_interface.spi->transfer(0x00);
    out[i] = ((int16_t)(upper << 8) | lower);
  }

  end_transaction(&this->spi_interface);
}

// output in LSBs
void Sensor::read_latest_accel_raw(int16_t *out) {
  begin_transaction(&this->spi_interface);

  this->spi_interface.spi->transfer(READ_FLAG | MPUREG_ACCEL_DATA_X1_UI);

  for (int i = 0; i < 3; ++i) {
    uint8_t lower = this->spi_interface.spi->transfer(0x00);
    uint8_t upper = this->spi_interface.spi->transfer(0x00);
    out[i] = ((int16_t)(upper << 8) | lower);
  }

  end_transaction(&this->spi_interface);
}

// output in g's and in deg/s
void Sensor::read_latest_no_calib(Data *output) {
  begin_transaction(&this->spi_interface);

  this->spi_interface.spi->transfer(READ_FLAG | MPUREG_ACCEL_DATA_X1_UI);
  // status = inv_icm406xx_read_reg((inv_icm406xx *)this->inv_icm, MPUREG_ACCEL_DATA_X1_UI, 12, (uint8_t*)temp);

  for (int i = 0; i < 3; ++i) {
    uint8_t lower = this->spi_interface.spi->transfer(0x00);
    uint8_t upper = this->spi_interface.spi->transfer(0x00);
    output->acc[i] = ((int16_t)(upper << 8) | lower) * ACCEL_RESOLUTION;
  }

  for (int i = 0; i < 3; ++i) {
    uint8_t lower = this->spi_interface.spi->transfer(0x00);
    uint8_t upper = this->spi_interface.spi->transfer(0x00);
    output->gyro[i] = ((int16_t)(upper << 8) | lower) * GYRO_RESOLUTION;
  }

  end_transaction(&this->spi_interface);
}

// output in m/s^2 and rad/s
void Sensor::read_latest(Data *output) {

  this->read_latest_no_calib(output);

  // apply bias to both accelerometer and gyroscope data
  for (int i = 0; i < 3; ++i) {
    output->acc[i] = (output->acc[i] - this->calib.accel_correction_bias[i]) / this->calib.accel_correction_gain[i] * G_TO_MS2;
  }
  for (int i = 0; i < 3; ++i) {
    output->gyro[i] = (output->gyro[i] - this->calib.gyro_bias[i]) * DEG_TO_RAD;
  }

  return;
}

int Sensor::enable_accel() {
  return inv_icm406xx_enable_accel_low_noise_mode((inv_icm406xx *)this->inv_icm);
}

int Sensor::enable_gyro() {
  return inv_icm406xx_enable_gyro_low_noise_mode((inv_icm406xx *)this->inv_icm);
}

int Sensor::disable_accelerometer() {
  return inv_icm406xx_disable_accel((inv_icm406xx *)this->inv_icm);
}

int Sensor::disable_gyro() {
  return inv_icm406xx_disable_gyro((inv_icm406xx *)this->inv_icm);
}

// it appears this function is expected to return 1 on error and 0 otherwise (same with write_reg). I don't see any reason here why we should return 1
int read_reg(void *context, uint8_t reg, uint8_t *buf, uint32_t len) {
  SPI_Interface *my_spi = (SPI_Interface *)context;

  begin_transaction(my_spi);

  my_spi->spi->transfer(READ_FLAG | reg);

  // the ICM should auto-increment the target address, except for in certain situations such as when reading from the fifo register
  for (uint32_t i = 0; i < len; ++i) {
    buf[i] = my_spi->spi->transfer(0x00);
  }

  end_transaction(my_spi);
  return 0;
}

int write_reg(void *context, uint8_t reg, const uint8_t *buf, uint32_t len) {
  SPI_Interface *my_spi = (SPI_Interface *)context;

  begin_transaction(my_spi);

  my_spi->spi->transfer(WRITE_FLAG | reg);

  // the ICM should auto-increment the target address
  for (uint32_t i = 0; i < len; ++i) {
    my_spi->spi->transfer(buf[i]);
  }

  end_transaction(my_spi);
  return 0;
}

void Sensor::clear_calib() {
  memset(this->calib.gyro_bias, 0, sizeof(this->calib.gyro_bias));
  for (int i = 0; i < 3; ++i) {
    this->calib.accel_correction_bias[i] = 0;
    this->calib.accel_correction_gain[i] = 1;
  }
}

void Sensor::load_calib(const char *filename) {
  SDCard::load_bytes(filename, (uint8_t *)&this->calib, sizeof(this->calib));
}

void Sensor::write_calib(const char *filename) {
  SDCard::write_bytes(filename, (uint8_t *)&this->calib, sizeof(this->calib));
}

void begin_transaction(SPI_Interface *my_spi) {
  my_spi->spi->beginTransaction(SPI_SETTINGS);
  delayMicroseconds(1);
  digitalWrite(my_spi->cs, LOW); // CS is active low
}

void end_transaction(SPI_Interface *my_spi) {
  digitalWrite(my_spi->cs, HIGH);
  delayMicroseconds(1);
  my_spi->spi->endTransaction();
}

// everything below is for namespace IMU, not present in Sensor class

void IMU::calibrate_gyroscope() {
  // TODO asynchronous calibration of multiple IMUs
  IMUs[0].calibrate_gyro();
}

void cmd_calibrate_gyro() {
  IMU::calibrate_gyroscope();

  for (int i = 0; i < IMU_COUNT; ++i) {
    Router::printf(" [%d] Gyro Biases (X, Y, Z) (degrees/s): [%7.3lf, %7.3lf, %7.3lf]\n", i, IMUs[i].calib.gyro_bias[0], IMUs[i].calib.gyro_bias[1], IMUs[i].calib.gyro_bias[2]);
  }
}

void cmd_log_accel_for_calibration(const char *param) {
  int imu_index = atoi(param);
  if (imu_index < 0 || imu_index >= IMU_COUNT) {
    Router::print("Invalid IMU number entered. Assuming index=0.\n");
    imu_index = 0;
  }

  Sensor *imu = &IMUs[imu_index];

  Data last_packet;
  char serial_input[10];

  while (1) {
    while (!Serial.available()) {
      delay(100);
    }
    Serial.readBytesUntil('\n', serial_input, sizeof(serial_input));
    if (!strcmp(serial_input, "stop")) {
      break;
    }

    unsigned int count = 0;

    double accumulator[3];
    memset(accumulator, 0, sizeof(accumulator));

    // average acceleration data over 2 seconds
    unsigned long start_time = millis();
    while (millis() - start_time < 2000) {
      imu->read_latest_no_calib(&last_packet);

      for (int i = 0; i < 3; ++i) {
        accumulator[i] += last_packet.acc[i];
      }
      ++count;
      delay(5);
    }

    Router::printf("%lf,%lf,%lf\n", accumulator[0] / count, accumulator[1] / count, accumulator[2] / count);

    delay(30);
  }
}

void cmd_imu_log() {
  Data last_packet;
  Router::print("Time (s)");

  for (int i = 0; i < IMU_COUNT; ++i) {
    Router::printf(
        ", (%d) Acceleration X (m/s^2), (%d) Acceleration Y (m/s^2), (%d) Acceleration Z (m/s^2), (%d) Angular Velocity X (rad/s), (%d) Angular Velocity Y (rad/s), (%d) Angular Velocity Z (rad/s)", i,
        i, i, i, i, i);
  }

  Router::print('\n');

  unsigned long start_time = micros();
  // run until the user presses enter
  while (!Serial.available()) {
    double t = (micros() - start_time) * 1e-6;

    Router::printf("%lf", t);
    for (int i = 0; i < IMU_COUNT; ++i) {
      IMUs[i].read_latest(&last_packet);

      Router::printf(", %.15lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf", last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0], last_packet.gyro[1], last_packet.gyro[2]);
    }

    Router::print('\n');

    // target about 100 hz?
    delayMicroseconds(10000);
  }
}

void cmd_load_custom_calib(const char *arg) {
  int index = atoi(arg);
  if (index < 0 || index >= IMU_COUNT) {
    Router::print("Invalid imu index. Assuming index=0.\n");
    index = 0;
  }

  IMUs[index].load_custom_calib();
}

void cmd_load_calib(const char *arg) {
  char filename[35];
  int imu_index;

  sscanf(arg, "%d %34s", &imu_index, filename);

  if (imu_index < 0 || imu_index >= IMU_COUNT) {
    Router::print("Invalid IMU index entered.");
    return;
  }

  IMUs[imu_index].load_calib(filename);
}

void cmd_write_calib(const char *arg) {
  char filename[35];
  int imu_index;

  sscanf(arg, "%d %34s", &imu_index, filename);

  if (imu_index < 0 || imu_index >= IMU_COUNT) {
    Router::print("Invalid IMU index entered.");
    return;
  }

  IMUs[imu_index].write_calib(filename);
}

void cmd_output_calib(const char *arg) {
  int imu_index = atoi(arg);
  if (imu_index < 0 || imu_index >= IMU_COUNT) {
    imu_index = 0;
  }

  Sensor *imu = &IMUs[imu_index];

  Router::printf("Calibration for IMU %d:\n", imu_index);
  Router::printf("Gyro Bias (deg/s) (X, Y, Z): %lf, %lf, %lf\n", imu->calib.gyro_bias[0], imu->calib.gyro_bias[1], imu->calib.gyro_bias[2]);
  Router::printf("Accelerometer Bias (g) (X, Y, Z): %lf, %lf, %lf\n", imu->calib.accel_correction_bias[0], imu->calib.accel_correction_bias[1], imu->calib.accel_correction_bias[2]);
  Router::printf("Accelerometer Gain (X, Y, Z): %lf, %lf, %lf\n", imu->calib.accel_correction_gain[0], imu->calib.accel_correction_gain[1], imu->calib.accel_correction_gain[2]);
}

void imu_speed_test()
{
  const int transaction_count = 5000;
  Data last_data;
  unsigned int begin_time = micros();

  for (int i = 0; i < transaction_count; ++i)
  {
    IMUs[0].read_latest(&last_data);
  }

  double delta_ms = (micros() - begin_time) / 1000.0;
  Router::printf("IMU Speed Test Finished\nTransactions: %d\nAverage transaction time (ms): %lf\n", transaction_count, delta_ms / transaction_count);
}

int IMU::begin() {

  int error = 0;
  error |= IMUs[0].init();
  error |= IMUs[0].enable_accel();
  error |= IMUs[0].enable_gyro();

  Router::add({cmd_calibrate_gyro, "imu_calibrate_gyro"});
  Router::add({cmd_imu_log, "imu_log"});
  Router::add({cmd_load_custom_calib, "imu_enter_calib"});
  Router::add({cmd_write_calib, "imu_write_calib"});
  Router::add({cmd_output_calib, "imu_print_calib"});
  Router::add({cmd_log_accel_for_calibration, "imu_log_accel_for_calib"});
  Router::add({imu_speed_test, "imu_speed_test"});

  return error;
}