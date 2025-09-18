#include<Arduino.h>
#include<SPI.h>

#include "IMU.h"

#define WRITE_MASK (0)
#define READ_MASK  (1 << 7)

#define SPI_SETTINGS SPISettings(1000000, MSBFIRST, SPI_MODE0)


#include "./Invn/Drivers/Icm406xx/Icm406xxDriver_HL.h"



int read_reg  (void* context, uint8_t reg, uint8_t* buf, uint32_t len);
int write_reg (void *context, uint8_t reg, const uint8_t *buf, uint32_t len);

void begin_transaction (IMU::SPI_Interface* my_spi);
void end_transaction   (IMU::SPI_Interface* my_spi);

IMU::IMU(int cs, arduino::MbedSPI* spi, void (*fifo_callback)(Sensor_Event *event)){
    this->spi_interface.cs = cs;
    this->spi_interface.spi = spi;

    this->inv_icm = malloc(sizeof(inv_icm406xx));

    this->fifo_callback = fifo_callback;
}

IMU::~IMU(){
    free(this->inv_icm);
}

int IMU::begin(){

    inv_icm406xx_serif serif;

    digitalWrite(this->spi_interface.cs, HIGH); // write before setting mode to output
    pinMode(this->spi_interface.cs, OUTPUT);
    
    // configure the serial interface (serif) structure that is used by TDK's driver
    serif.context       = &this->spi_interface;
    serif.read_reg      = &read_reg;
    serif.write_reg     = &write_reg;
    serif.max_read      = 1024 * 32;
    serif.max_write     = 1024 * 32;
    serif.serif_type    = ICM406XX_UI_SPI4;

    return inv_icm406xx_init((inv_icm406xx*)this->inv_icm, &serif, this->fifo_callback);
}


// sensor events are passed to the fifo callback
int IMU::read_fifo(){
    return inv_icm406xx_get_data_from_fifo((inv_icm406xx*)this->inv_icm);
}

int IMU::enable_accel(){
    return inv_icm406xx_enable_accel_low_noise_mode((inv_icm406xx*)this->inv_icm);
}

int IMU::enable_gyro(){
    return inv_icm406xx_enable_gyro_low_noise_mode((inv_icm406xx*)this->inv_icm);
}


int IMU::disable_accelerometer(){
    return inv_icm406xx_disable_accel((inv_icm406xx*)this->inv_icm);
}

int IMU::disable_gyro(){
    return inv_icm406xx_disable_gyro((inv_icm406xx*)this->inv_icm);
}


// it appears this function is expected to return 1 on error and 0 otherwise (same with write_reg). I don't see any reason here why we should return 1
int read_reg(void* context, uint8_t reg, uint8_t* buf, uint32_t len){
    IMU::SPI_Interface* my_spi = (IMU::SPI_Interface*)context;

    begin_transaction(my_spi);

    my_spi->spi->transfer(READ_MASK | reg);

    // the ICM should auto-increment the target address, except for in certain situations such as when reading from the fifo register
    for (uint32_t i = 0; i < len; ++i){
        buf[i] = my_spi->spi->transfer(0x00);
    }
    
    end_transaction(my_spi);
    return 0;
}

int write_reg(void *context, uint8_t reg, const uint8_t *buf, uint32_t len){
    IMU::SPI_Interface* my_spi = (IMU::SPI_Interface*)context;

    begin_transaction(my_spi);

    my_spi->spi->transfer(WRITE_MASK | reg);

    // the ICM should auto-increment the target address
    for (uint32_t i = 0; i < len; ++i){
        my_spi->spi->transfer(buf[i]);
    }

    end_transaction(my_spi);
    return 0;
}

void begin_transaction(IMU::SPI_Interface* my_spi){
    my_spi->spi->beginTransaction(SPI_SETTINGS);
    delayMicroseconds(1);
    digitalWrite(my_spi->cs, LOW); // CS is active low
}

void end_transaction(IMU::SPI_Interface* my_spi){
    digitalWrite(my_spi->cs, HIGH);
    delayMicroseconds(1);
    my_spi->spi->endTransaction();
}
