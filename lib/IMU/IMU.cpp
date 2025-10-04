#include <Arduino.h>
#include <SPI.h>

#include "IMU.h"

#include "./Invn/Drivers/Icm406xx/Icm406xxDriver_HL.h"
#include "./Invn/Drivers/Icm406xx/Icm406xxTransport.h"

#define WRITE_MASK (0)
#define READ_MASK (1 << 7)

// do not exceed 24 MHz
#define SPI_SETTINGS SPISettings(15 * 1000000, MSBFIRST, SPI_MODE0)

int read_reg(void *context, uint8_t reg, uint8_t *buf, uint32_t len);
int write_reg(void *context, uint8_t reg, const uint8_t *buf, uint32_t len);

void begin_transaction(IMU::SPI_Interface *my_spi);
void end_transaction(IMU::SPI_Interface *my_spi);

IMU::IMU(int cs, arduino::MbedSPI *spi) {
  this->spi_interface.cs = cs;
  this->spi_interface.spi = spi;

  this->inv_icm = malloc(sizeof(inv_icm406xx));

}

IMU::~IMU() {
  free(this->inv_icm);
}

void sum(int16_t* in, int64_t* in2_out, int dim){
  
  for (int i = 0; i < dim; ++i){
    in2_out[i] += in[i];
  }
}

void IMU::fifo_callback(void *ctx, inv_icm406xx_sensor_event_t *event) {
  IMU::acc* accumulator = (IMU::acc*)ctx;

  sum(event->accel, accumulator->accel, 3);
  sum(event->gyro, accumulator->gyro, 3);
  ++accumulator->count;
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

  status = inv_icm406xx_init((inv_icm406xx *)this->inv_icm, &serif, IMU::fifo_callback);

  if (status) return status;

  // tell IMU to report data in little endian
  status = inv_icm406xx_wr_intf_config0_data_endian((inv_icm406xx *)this->inv_icm, ICM406XX_INTF_CONFIG0_DATA_LITTLE_ENDIAN);

  return status;
}

int IMU::read_accel_config(uint8_t* value){
  return inv_icm406xx_read_reg((inv_icm406xx *)this->inv_icm, MPUREG_ACCEL_CONFIG0, 1, value);
}

// sensor events are passed to the fifo callback
int IMU::read_fifo(IMU::sensor_data* output) {
  memset(&this->accumulator, 0, sizeof(this->accumulator));

  int status = inv_icm406xx_get_data_from_fifo(&this->accumulator, (inv_icm406xx *)this->inv_icm);

  for (int i = 0; i < 3; ++i){
    output->acc[i] = ((this->accumulator.accel[i] + this->accumulator.count/2) / this->accumulator.count) / 8192.0;  //TODO get rid of these magic numbers once corrected - these are the sensitivities from the datasheet
    output->gyro[i] = ((this->accumulator.gyro[i] + this->accumulator.count / 2) / this->accumulator.count) / 2097.2;
  }
}

int IMU::read_latest(IMU::sensor_data* output){
  int status;
  int16_t temp[6];

  status = inv_icm406xx_read_reg((inv_icm406xx *)this->inv_icm, MPUREG_ACCEL_DATA_X1_UI, 12, (uint8_t*)temp);

  // convert values (TODO get rid of magic numbers)

  for (int i = 0; i < 3; ++i){
    output->acc[i] = temp[i] / 8192.0;
    output->gyro[i] = temp[i+3] / 2097.2;
  }

  return status;
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

  my_spi->spi->transfer(READ_MASK | reg);

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

  my_spi->spi->transfer(WRITE_MASK | reg);

  // the ICM should auto-increment the target address
  for (uint32_t i = 0; i < len; ++i) {
    my_spi->spi->transfer(buf[i]);
  }

  end_transaction(my_spi);
  return 0;
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
