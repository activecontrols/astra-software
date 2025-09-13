#pragma once

class ICM40609D{
public:
    ICM40609D(int);

    void begin();
    double get_temp_c();
private:
    int CS; // chip select pin
    void write_register(uint8_t addr, uint8_t data);
    void write_register16(uint8_t addr, uint16_t data);

    uint8_t read_register(uint8_t addr);
    uint16_t read_register16(uint8_t addr);
    

    void write_register_mask(uint8_t addr, uint8_t data, uint8_t mask);
    void write_register_mask16(uint8_t addr, uint16_t data, uint16_t mask);

    void begin_transaction();
    void end_transaction();

    void enable_accel_gyro();
    void disable_accel_gyro();
};