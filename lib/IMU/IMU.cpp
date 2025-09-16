#include<Arduino.h>
#include<SPI.h>

#include "IMU.h"

#define WRITE_MASK (0)
#define READ_MASK  (1 << 7)

#define SPI_SETTINGS SPISettings(1000000, MSBFIRST, SPI_MODE0)


#include "./Invn/Drivers/Icm406xx/Icm406xxDriver_HL.h"


int read_reg(void* context, uint8_t reg, uint8_t* buf, uint32_t len);
int write_reg(void *context, uint8_t reg, const uint8_t *buf, uint32_t len);

void begin_transaction(int cs);
void end_transaction(int cs);

struct IMU::Impl{
    inv_icm406xx inv_icm;
    inv_icm406xx_serif serif;
};

IMU::IMU(int cs){
    this->cs = cs;

    this->pimpl_ = std::make_unique<IMU::Impl>();
}

void IMU::begin(){
    digitalWrite(this->cs, HIGH); // write before setting mode to output
    pinMode(this->cs, OUTPUT);
    
    // configure the serial interface (serif) structure that is used by TDK's driver
    this->pimpl_->serif.context       = (&this->cs);
    this->pimpl_->serif.read_reg      = &read_reg;
    this->pimpl_->serif.write_reg     = &write_reg;
    this->pimpl_->serif.max_read      = 1024 * 32;
    this->pimpl_->serif.max_write     = 1024 * 32;
    this->pimpl_->serif.serif_type    = ICM406XX_UI_SPI4;

    // note: the third parameter of this function is for a callback which also receives every packet of gyro/accel data from the imu
    inv_icm406xx_init(&this->pimpl_->inv_icm, &this->pimpl_->serif, nullptr);
}



// it appears this function is expected to return 1 on error and 0 otherwise (same with write_reg). I don't see any reason here why we should return 1
int read_reg(void* context, uint8_t reg, uint8_t* buf, uint32_t len){
    int cs = *(int*)context;

    begin_transaction(cs);

    SPI.transfer(READ_MASK | reg);

    // the ICM should auto-increment the target address, except for in certain situations such as when reading from the fifo register
    for (uint32_t i = 0; i < len; ++i){
        buf[i] = SPI.transfer(0x00);
    }
    
    end_transaction(cs);
    return 0;
}

int write_reg(void *context, uint8_t reg, const uint8_t *buf, uint32_t len){
    int cs = *(int*)context;

    begin_transaction(cs);

    SPI.transfer(WRITE_MASK | reg);

    // the ICM should auto-increment the target address
    for (uint32_t i = 0; i < len; ++i){
        SPI.transfer(buf[i]);
    }

    end_transaction(cs);
    return 0;
}

void begin_transaction(int cs){
    SPI.beginTransaction(SPI_SETTINGS);
    delayMicroseconds(1);
    digitalWrite(cs, LOW); // CS is active low
}

void end_transaction(int cs){
    digitalWrite(cs, HIGH);
    delayMicroseconds(1);
    SPI.endTransaction();
}