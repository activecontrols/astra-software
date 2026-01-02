#include <Arduino.h>
#include <SPI.h>

#include "IMU.h"

#include "./Invn/Drivers/Icm406xx/Icm406xxDriver_HL.h"
#include "./Invn/Drivers/Icm406xx/Icm406xxTransport.h"
#include "./Invn/EmbUtils/ErrorHelper.h"

#include "Router.h"
#include "SDCard.h"
#include "fc_pins.h"

#define WRITE_FLAG (0)
#define READ_FLAG (1 << 7)

// Unit: Hz
// !!! DO NOT EXCEED 24MHz !!!
#define SPI_RATE (1 * 1000000) // TODO - 24 used to work, check on a scope

#define G_TO_MS2 9.80145

IMU IMU::IMUs[IMU_COUNT]{};

// g / LSB (this is for +-8g fsr)
const double ACCEL_RESOLUTION = 1.0 / 4096.0;

// dps / LSB (this is for +- 2000dps fsr)
const double GYRO_RESOLUTION = 1.0 / 16.4;

#define SPI_SETTINGS SPISettings(SPI_RATE, MSBFIRST, SPI_MODE0)

using namespace IMU;

Sensor IMU::IMUs[IMU_COUNT] = {Sensor(IMU_CS, &fc_spi)};

int read_reg(void *context, uint8_t reg, uint8_t *buf, uint32_t len);
int write_reg(void *context, uint8_t reg, const uint8_t *buf, uint32_t len);

void begin_transaction(SPI_Interface *my_spi);
void end_transaction(SPI_Interface *my_spi);

Sensor::Sensor(int cs, SPIClass *spi) {
  this->spi_interface.cs = cs;
  this->spi_interface.spi = spi;
IMU::IMU() {

  this->clear_calib();

  this->inv_icm = malloc(sizeof(inv_icm406xx));
}

IMU::~IMU() {
  free(this->inv_icm);
}

int IMU::init(int cs, SPIClass *spi) {
  int status;

  this->spi_interface.cs = cs;
  this->spi_interface.spi = spi;

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

  // adjust these per the datasheet to choose the aaf filter bandwidth
  // these values have been chosen for a filter bandwidth of 394 Hz
  const uint8_t accel_aaf_delt = 9;
  const uint16_t accel_aaf_deltsqr = 81;
  const uint8_t accel_aaf_bitshift = 9;

  const uint8_t gyro_aaf_delt = 9;
  const uint16_t gyro_aaf_deltsqr = 81;
  const uint8_t gyro_aaf_bitshift = 9;

  // switch to user bank 1
  status = inv_icm406xx_wr_reg_bank_sel((inv_icm406xx *)this->inv_icm, 1);

  // enable anti-alias filter on gyro
  status |= this->write_reg_mask(MPUREG_GYRO_CONFIG_STATIC2_B1, (1 << 1), 0);

  // adjust gyro aaf (anti alias filter) bandwidth

  status |= this->write_reg_mask(MPUREG_GYRO_CONFIG_STATIC3_B1, 0b0011'1111, gyro_aaf_delt);
  status |= this->write_reg_mask(MPUREG_GYRO_CONFIG_STATIC4_B1, 0xFF, gyro_aaf_deltsqr & 0xFF);
  status |= this->write_reg_mask(MPUREG_GYRO_CONFIG_STATIC5_B1, 0b0000'1111, gyro_aaf_deltsqr >> 8);
  status |= this->write_reg_mask(MPUREG_GYRO_CONFIG_STATIC5_B1, 0b1111'0000, gyro_aaf_bitshift << 4);

  // switch to user bank 2
  status |= inv_icm406xx_wr_reg_bank_sel((inv_icm406xx *)this->inv_icm, 2);

  // enable anti-alias filter on accelerometer
  status |= this->write_reg_mask(MPU_ACCEL_CONFIG_STATIC2_B2, (1 << 0), 0);

  // adjust accel aaf (anti alias filter) bandwidth
  status |= this->write_reg_mask(MPU_ACCEL_CONFIG_STATIC2_B2, 0b0111'1110, accel_aaf_delt << 1);
  status |= this->write_reg_mask(MPU_ACCEL_CONFIG_STATIC3_B2, 0xFF, accel_aaf_deltsqr & 0xFF);
  status |= this->write_reg_mask(MPU_ACCEL_CONFIG_STATIC4_B2, 0b0000'1111, accel_aaf_deltsqr >> 8);
  status |= this->write_reg_mask(MPU_ACCEL_CONFIG_STATIC4_B2, 0b1111'0000, accel_aaf_bitshift << 4);

  // switch back to user bank 0 for normal operation
  status |= inv_icm406xx_wr_reg_bank_sel((inv_icm406xx *)this->inv_icm, 0);

  /* =========================================================================================
      ---- done enabling anti-alias filter ----
     =========================================================================================*/

  if (status) {
    const char *error_message = inv_error_str(status);
    Router::printf("Error occurred while enabling anti-alias filter on IMU: %s\n", error_message);
    return status;
  }

  // set gyro and accel odr
  status |= inv_icm406xx_wr_gyro_config0_odr((inv_icm406xx *)this->inv_icm, ICM406XX_GYRO_CONFIG0_ODR_2_KHZ);
  status |= inv_icm406xx_wr_accel_config0_odr((inv_icm406xx *)this->inv_icm, ICM406XX_ACCEL_CONFIG0_ODR_2_KHZ);

  if (status) {
    const char *error_message = inv_error_str(status);
    Router::printf("Error occurred while setting odr on IMU: %s\n", error_message);
    return status;
  }

  return status;
}

int IMU::write_reg_mask(uint8_t addr, uint8_t mask, uint8_t val) {
  uint8_t og_val;
  uint8_t new_val;
  int status = 0;

  status |= read_reg(&this->spi_interface, addr, &og_val, 1);

  new_val = (og_val & ~mask) | (val & mask);

  status |= write_reg(&this->spi_interface, addr, &new_val, 1);

  return status;
}

// loads three doubles from serial and stores them in dest
int load_calibration_helper(double *dest) {
  char buf[3][100];
  char *line = Router::read();
  Router::println(line);
  int read = sscanf(line, "%99s %99s %99s", buf[0], buf[1], buf[2]);
  if (read != 3) {
    return read;
  }
  for (int i = 0; i < 3; ++i) {
    dest[i] = atof(buf[i]);
  }
  return read;
}

void IMU::load_custom_calib() {
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

  memcpy(&this->calib, &new_calib, sizeof(IMU::calib));
  Router::printf("Calibration written in memory. Remember to save calibration data to SD Card!!!");
}

int IMU::read_accel_config(uint8_t *value) {
  return inv_icm406xx_read_reg((inv_icm406xx *)this->inv_icm, MPUREG_ACCEL_CONFIG0, 1, value);
}

// output in LSBs
void IMU::read_latest_gyro_raw(int16_t *out) {
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
void IMU::read_latest_accel_raw(int16_t *out) {
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
void IMU::read_latest_no_calib(IMU::Measurement *output) {
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
void IMU::read_latest(IMU::Measurement *output) {

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

int IMU::enable_accel() {
  return inv_icm406xx_enable_accel_low_noise_mode((inv_icm406xx *)this->inv_icm);
}

int IMU::enable_gyro() {
  return inv_icm406xx_enable_gyro_low_noise_mode((inv_icm406xx *)this->inv_icm);
}

int IMU::disable_accelerometer() {
  return inv_icm406xx_disable_accel((inv_icm406xx *)this->inv_icm);
}

int IMU::disable_gyro() {
  return inv_icm406xx_disable_gyro((inv_icm406xx *)this->inv_icm);
}

// it appears this function is expected to return 1 on error and 0 otherwise (same with write_reg). I don't see any reason here why we should return 1
int IMU::read_reg(void *context, uint8_t reg, uint8_t *buf, uint32_t len) {
  IMU::SPI_Interface *my_spi = static_cast<IMU::SPI_Interface *>(context);

  begin_transaction(my_spi);

  my_spi->spi->transfer(READ_FLAG | reg);

  // the ICM should auto-increment the target address, except for in certain situations such as when reading from the fifo register
  my_spi->spi->transfer(nullptr, buf, len);

  end_transaction(my_spi);
  return 0;
}

int IMU::write_reg(void *context, uint8_t reg, const uint8_t *buf, uint32_t len) {
  SPI_Interface *my_spi = static_cast<IMU::SPI_Interface *>(context);

  begin_transaction(my_spi);

  my_spi->spi->transfer(WRITE_FLAG | reg);

  // the ICM should auto-increment the target write address
  my_spi->spi->transfer(buf, nullptr, len);

  end_transaction(my_spi);
  return 0;
}

void IMU::clear_calib() {
  memset(this->calib.gyro_bias, 0, sizeof(this->calib.gyro_bias));
  for (int i = 0; i < 3; ++i) {
    this->calib.accel_correction_bias[i] = 0;
    this->calib.accel_correction_gain[i] = 1;
  }
}

void IMU::load_calib(const char *filename) {
  SDCard::load_bytes(filename, (uint8_t *)&this->calib, sizeof(this->calib));
}

void IMU::write_calib(const char *filename) {
  SDCard::write_bytes(filename, (uint8_t *)&this->calib, sizeof(this->calib));
}

void IMU::begin_transaction(SPI_Interface *my_spi) {
  my_spi->spi->beginTransaction(SPI_SETTINGS);
  delayMicroseconds(1);
  digitalWrite(my_spi->cs, LOW); // CS is active low
}

void IMU::end_transaction(SPI_Interface *my_spi) {
  digitalWrite(my_spi->cs, HIGH);
  delayMicroseconds(1);
  my_spi->spi->endTransaction();
}

void IMU::read_latest_all(IMU::Measurement output[IMU_COUNT]) {
  for (int i = 0; i < IMU_COUNT; ++i) {
    IMUs[i].read_latest(&output[i]);
  }

  return;
}

void IMU::calibrate_gyroscope_all() {
  const unsigned long delay_millis = 20000; // 20 seconds

  int64_t sums[IMU_COUNT][3];
  memset(sums, 0, sizeof(sums));

  int16_t last_measurement[IMU_COUNT][3];

  unsigned int measurement_count = 0;

  unsigned long start_millis = millis();
  // simultaneous calibration of all IMUs
  while (millis() - start_millis < delay_millis) {
    // collect raw measurements
    for (int i = 0; i < IMU_COUNT; ++i) {
      IMUs[i].read_latest_gyro_raw(last_measurement[i]);
    }

    for (int i = 0; i < IMU_COUNT; ++i) {
      for (int j = 0; j < 3; ++j) {
        sums[i][j] += last_measurement[i][j];
      }
    }

    ++measurement_count;
    delay(1);
  }

  // compute average biases and store
  for (int i = 0; i < IMU_COUNT; ++i) {
    for (int j = 0; j < 3; ++j) {
      IMUs[i].calib.gyro_bias[j] = static_cast<double>(sums[i][j]) / measurement_count * GYRO_RESOLUTION;
    }
  }

  return;
}

void IMU::cmd_calibrate_gyro() {
  IMU::calibrate_gyroscope_all();

  for (int i = 0; i < IMU_COUNT; ++i) {
    Router::printf(" [%d] Gyro Biases (X, Y, Z) (degrees/s): [%7.3lf, %7.3lf, %7.3lf]\n", i, IMU::IMUs[i].calib.gyro_bias[0], IMU::IMUs[i].calib.gyro_bias[1], IMU::IMUs[i].calib.gyro_bias[2]);
  }
}

void IMU::cmd_log_accel_for_calibration(const char *param) {
  int imu_index = atoi(param);
  if (imu_index < 0 || imu_index >= IMU_COUNT) {
    Router::print("Invalid IMU number entered. Assuming index=0.\n");
    imu_index = 0;
  }

  IMU *imu = &IMU::IMUs[imu_index];

  IMU::Measurement last_packet;
  char serial_input[10];

  while (1) {
    while (!Router::available()) {
      delay(100);
    }
    external_uart.readBytesUntil('\n', serial_input, sizeof(serial_input));
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

void IMU::cmd_imu_log() {
  IMU::Measurement last_packet;
  Router::print("Time (s)");

  for (int i = 0; i < IMU_COUNT; ++i) {
    Router::printf(
        ",(%d) Acceleration X (m/s^2),(%d) Acceleration Y (m/s^2),(%d) Acceleration Z (m/s^2),(%d) Angular Velocity X (rad/s),(%d) Angular Velocity Y (rad/s),(%d) Angular Velocity Z (rad/s)", i, i, i,
        i, i, i);
  }

  Router::print('\n');

  unsigned long start_time = micros();
  // run until the user presses enter
  while (!Router::available()) {
    double t = (micros() - start_time) * 1e-6;

    Router::printf("%lf", t);
    for (int i = 0; i < IMU_COUNT; ++i) {
      IMU::IMUs[i].read_latest(&last_packet);

      Router::printf(", %.15lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf", last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0], last_packet.gyro[1], last_packet.gyro[2]);
    }

    Router::print('\n');

    // target about 100 hz?
    delayMicroseconds(10000);
  }
}

void IMU::cmd_load_custom_calib(const char *arg) {
  int index = atoi(arg);
  if (index < 0 || index >= IMU_COUNT) {
    Router::print("Invalid imu index. Assuming index=0.\n");
    index = 0;
  }

  IMU::IMUs[index].load_custom_calib();
}

void IMU::cmd_load_calib(const char *arg) {
  char filename[35];
  int imu_index;

  sscanf(arg, "%d %34s", &imu_index, filename);

  if (imu_index < 0 || imu_index >= IMU_COUNT) {
    Router::print("Invalid IMU index entered.");
    return;
  }

  IMU::IMUs[imu_index].load_calib(filename);
}

void IMU::cmd_write_calib(const char *arg) {
  char filename[35];
  int imu_index;

  sscanf(arg, "%d %34s", &imu_index, filename);

  if (imu_index < 0 || imu_index >= IMU_COUNT) {
    Router::print("Invalid IMU index entered.");
    return;
  }

  IMU::IMUs[imu_index].write_calib(filename);
}

void IMU::cmd_output_calib(const char *arg) {
  int imu_index = atoi(arg);
  if (imu_index < 0 || imu_index >= IMU_COUNT) {
    imu_index = 0;
  }

  IMU *imu = &IMU::IMUs[imu_index];

  Router::printf("Calibration for IMU %d:\n", imu_index);
  Router::printf("Gyro Bias (deg/s) (X, Y, Z): %lf, %lf, %lf\n", imu->calib.gyro_bias[0], imu->calib.gyro_bias[1], imu->calib.gyro_bias[2]);
  Router::printf("Accelerometer Bias (g) (X, Y, Z): %lf, %lf, %lf\n", imu->calib.accel_correction_bias[0], imu->calib.accel_correction_bias[1], imu->calib.accel_correction_bias[2]);
  Router::printf("Accelerometer Gain (X, Y, Z): %lf, %lf, %lf\n", imu->calib.accel_correction_gain[0], imu->calib.accel_correction_gain[1], imu->calib.accel_correction_gain[2]);
}

void IMU::cmd_imu_speed_test() {
  const int transaction_count = 5000;
  IMU::Measurement last_data;
  unsigned int begin_time = micros();

  for (int i = 0; i < transaction_count; ++i) {
    IMU::IMUs[0].read_latest(&last_data);
  }

  double delta_ms = (micros() - begin_time) / 1000.0;
  Router::printf("IMU Speed Test Finished\nTransactions: %d\nAverage transaction time (ms): %lf\n", transaction_count, delta_ms / transaction_count);
}

void IMU::begin() {

  for (int i = 0; i < IMU_COUNT; ++i) {
    bool error = IMUs[i].init(IMU_CS[i], &SPI);
    error = error || IMUs[i].enable_accel();
    error = error || IMUs[i].enable_gyro();

    if (error)
      Router::printf("IMU %d Failed to Initialize\n", i);
  }

  Router::add({cmd_calibrate_gyro, "imu_calibrate_gyro"});
  Router::add({cmd_imu_log, "imu_log"});
  Router::add({cmd_load_custom_calib, "imu_enter_calib"});
  Router::add({cmd_write_calib, "imu_write_calib"});
  Router::add({cmd_output_calib, "imu_print_calib"});
  Router::add({cmd_log_accel_for_calibration, "imu_log_accel_for_calib"});
  Router::add({cmd_imu_speed_test, "imu_speed_test"});

  return;
}