#include <Arduino.h>
#include <SPI.h>

#include "IMU.h"

#include "./Invn/Drivers/Icm406xx/Icm406xxDriver_HL.h"
#include "./Invn/Drivers/Icm406xx/Icm406xxTransport.h"

#include "SDCard.h"
#include "Router.h"

#define WRITE_FLAG (0)
#define READ_FLAG (1 << 7)

// LSB / g
#define ACCEL_RESOLUTION 8192.0

// LSB / (degree/s)
#define GYRO_RESOLUTION 16.4
// #define GYRO_RESOLUTION 65.5

// do not exceed 24 MHz
#define SPI_SETTINGS SPISettings(1 * 1000000, MSBFIRST, SPI_MODE0)

int read_reg(void *context, uint8_t reg, uint8_t *buf, uint32_t len);
int write_reg(void *context, uint8_t reg, const uint8_t *buf, uint32_t len);

void begin_transaction(IMU::SPI_Interface *my_spi);
void end_transaction(IMU::SPI_Interface *my_spi);

IMU::IMU(int cs, arduino::MbedSPI *spi) {
  this->spi_interface.cs = cs;
  this->spi_interface.spi = spi;

  this->clear_calib();

  this->inv_icm = malloc(sizeof(inv_icm406xx));
}

IMU::~IMU() {
  free(this->inv_icm);
}

int IMU::begin() {

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

  if (status)
    return status;

  // tell IMU to report data in little endian !!!DO NOT REMOVE THIS LINE!!!
  status = inv_icm406xx_wr_intf_config0_data_endian((inv_icm406xx *)this->inv_icm, ICM406XX_INTF_CONFIG0_DATA_LITTLE_ENDIAN);

  return status;
}

// loads three doubles and stores them in dest
int load_calibration_helper(double* dest)
{
  char buf[3][100];
  String line = Router::read(100);
  line.trim();
  Router::println(line);
  int read = sscanf(line.c_str(), "%99s %99s %99s", buf[0], buf[1], buf[2]);
  if (read != 3)
  {
    return read;
  }

  for (int i = 0; i < 3; ++i)
  {
    dest[i] = atof(buf[i]);
  }


  return read;
}


void IMU::load_custom_calib()
{
  IMU::IMU_Calib new_calib;

  const String prompts[] = {"Gyroscope bias", "Accelerometer bias", "Accelerometer gain"};
  double* destinations[] = {new_calib.gyro_bias, new_calib.accel_correction_bias, new_calib.accel_correction_gain};

  for (int i = 0; i < 3; ++i)
  {
    Router::printf("Enter %s (X Y Z) separated by spaces: ", prompts[i].c_str());
    int read = load_calibration_helper(destinations[i]);
    if (read != 3)
    {
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

void IMU::calibrate_gyro() {
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
    this->calib.gyro_bias[i] = (div + frac) / GYRO_RESOLUTION;
  }

  return;
}

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

void IMU::read_latest_raw(IMU::sensor_data *output) {
  begin_transaction(&this->spi_interface);

  this->spi_interface.spi->transfer(READ_FLAG | MPUREG_ACCEL_DATA_X1_UI);
  // status = inv_icm406xx_read_reg((inv_icm406xx *)this->inv_icm, MPUREG_ACCEL_DATA_X1_UI, 12, (uint8_t*)temp);

  for (int i = 0; i < 3; ++i) {
    uint8_t lower = this->spi_interface.spi->transfer(0x00);
    uint8_t upper = this->spi_interface.spi->transfer(0x00);
    output->acc[i] = ((int16_t)(upper << 8) | lower) / ACCEL_RESOLUTION;
  }

  for (int i = 0; i < 3; ++i) {
    uint8_t lower = this->spi_interface.spi->transfer(0x00);
    uint8_t upper = this->spi_interface.spi->transfer(0x00);
    output->gyro[i] = ((int16_t)(upper << 8) | lower) / GYRO_RESOLUTION;
  }

  end_transaction(&this->spi_interface);
}

void IMU::read_latest(IMU::sensor_data *output) {

  this->read_latest_raw(output);

  // apply bias to both accelerometer and gyroscope data
  for (int i = 0; i < 3; ++i) {
    output->acc[i] = (output->acc[i] - this->calib.accel_correction_bias[i]) / this->calib.accel_correction_gain[i];
  }
  for (int i = 0; i < 3; ++i) {
    output->gyro[i] = output->gyro[i] - this->calib.gyro_bias[i];
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
int read_reg(void *context, uint8_t reg, uint8_t *buf, uint32_t len) {
  IMU::SPI_Interface *my_spi = (IMU::SPI_Interface *)context;

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
  IMU::SPI_Interface *my_spi = (IMU::SPI_Interface *)context;

  begin_transaction(my_spi);

  my_spi->spi->transfer(WRITE_FLAG | reg);

  // the ICM should auto-increment the target address
  for (uint32_t i = 0; i < len; ++i) {
    my_spi->spi->transfer(buf[i]);
  }

  end_transaction(my_spi);
  return 0;
}

void IMU::clear_calib()
{
  memset(this->calib.gyro_bias, 0, sizeof(this->calib.gyro_bias));
  for (int i = 0; i < 3; ++i)
  {
    this->calib.accel_correction_bias[i] = 0;
    this->calib.accel_correction_gain[i] = 1;
  }
}

void IMU::load_calib(const char *filename) {
  SDCard::load_bytes(filename, (uint8_t*)&this->calib, sizeof(this->calib));
}

void IMU::write_calib(const char *filename) {
  SDCard::write_bytes(filename, (uint8_t*)&this->calib, sizeof(this->calib));
}

void begin_transaction(IMU::SPI_Interface *my_spi) {
  my_spi->spi->beginTransaction(SPI_SETTINGS);
  delayMicroseconds(1);
  digitalWrite(my_spi->cs, LOW); // CS is active low
}

void end_transaction(IMU::SPI_Interface *my_spi) {
  digitalWrite(my_spi->cs, HIGH);
  delayMicroseconds(1);
  my_spi->spi->endTransaction();
}
