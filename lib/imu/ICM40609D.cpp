#include <SPI.h>
#include <Arduino.h>
#include "ICM40609D.h"

// datasheet says 24MHz max over SPI
//#define SPI_HZ 24000000

#define SPI_HZ 1000000

#define READ  (1 << 7)
#define WRITE (0)



// register addresses


#define SETTINGS SPISettings(SPI_HZ, MSBFIRST, SPI_MODE3)



/*

DS: Do not modify any registers other than (GYRO_ODR, ACCEL_ODR, GYRO_FS_SEL, ACCEL_FS_SEL, GYRO_MODE,
ACCEL_MODE) when accel and gyro are active (11.5)



we probably want to enable stream to FIFO mode and burst read the fifo when a position update is needed

we should use low noise mode (see 11.1)

13.4 FIFO_CONFIG - set to stream mode

13.22 & 23 FIFO_COUNT

13.24 FIFO DATA - i think we poll this a bunch to read data from FIFO?

13.27 INTF_CONFIG0 - change what is represented by FIFO_COUNT (either # of bytes or # of entries)

13.29 PWR_MGMT0 - enable/disable gyro, accelerometer and change mode lp or ln

REG_BANK_SEL - need to select register banks before accessing them

*/


ICM40609D::ICM40609D(int chip_sel){
    this->CS = chip_sel;
    pinMode(this->CS, OUTPUT);
    digitalWrite(this->CS, HIGH);
}

void ICM40609D::begin(){
    // enable recommended SPI settings 
    this->write_register_mask(0x13, 0b101, 0b00111111);

    // do any other writing to registers for setup here

    // set fifo size reporting mode to report No. of entries instead of No. of bytes
    this->write_register_mask(0x76, 0b01000000, 0b01000000);

    //this->write_register_mask16(0x79, 0b, 0b1110000011100000);

    // TODO: enable FIFO stream





    //this->enable_accel_gyro();
}


// get the temperature in degrees celsius
double ICM40609D::get_temp_c(){
    return this->read_register16(0x1D) / 132.48 + 25.00;
}


void ICM40609D::write_register(uint8_t addr, uint8_t data){
    this->begin_transaction();

    SPI.transfer(WRITE | addr);
    SPI.transfer(data);

    this->end_transaction();
}

void ICM40609D::write_register16(uint8_t addr, uint16_t data){
    this->begin_transaction();

    SPI.transfer(WRITE | addr);
    SPI.transfer16(data);

    this->end_transaction();
}

// read 8-bit unsigned integer from addr
uint8_t ICM40609D::read_register(uint8_t addr){
    uint8_t res;
    this->begin_transaction();
    
    SPI.transfer(READ | addr);
    res = SPI.transfer(0x00);

    this->end_transaction();
    return res;
}

// read 16-bit unsigned integer from addr
// note: some register values are stored little endian and others are stored big endian
// the caller will need to swap the order of the bytes
// this function assumes the byte stored at (addr) is most significant
uint16_t ICM40609D::read_register16(uint8_t addr){
    uint16_t res;
    this->begin_transaction();
    
    SPI.transfer(READ | addr);
    res = SPI.transfer16(0x00);

    this->end_transaction();
    return res;
}

void ICM40609D::write_register_mask(uint8_t addr, uint8_t data, uint8_t mask){
    uint8_t old_data = this->read_register(addr);
    uint8_t new_data = (old_data & mask) | data;

    this->write_register(addr, new_data);
}

void ICM40609D::write_register_mask16(uint8_t addr, uint16_t data, uint16_t mask){
    uint16_t old_data = this->read_register16(addr);
    uint16_t new_data = (old_data & mask) | data;

    this->write_register(addr, new_data);
}

void ICM40609D::begin_transaction(){
    SPI.beginTransaction(SETTINGS);
    delayMicroseconds(1); // datasheet says wait at least 39ns
    digitalWrite(this->CS, 0); // active low
}

void ICM40609D::end_transaction(){
    digitalWrite(this->CS, 1);
    delayMicroseconds(1);
    SPI.endTransaction();
}


// note: wait 45ms after enabling before doing anything
void ICM40609D::enable_accel_gyro(){
    // put both in LN mode
    this->write_register_mask(0x4E, 0b00001111, 0b00001111);
}

// note: wait 200 microseconds after running this function before writing any other registers
void ICM40609D::disable_accel_gyro(){
    this->write_register_mask(0x4E, 0x00, 0b00001111);
}