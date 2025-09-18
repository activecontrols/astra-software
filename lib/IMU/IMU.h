#pragma once
#include "./Invn/Drivers/Icm406xx/Sensor_Event.h"
#include <SPI.h>

typedef inv_icm406xx_sensor_event_t Sensor_Event;


//TODO: add function to enable/disable gyro, accel
class IMU{
    public:

    IMU(int cs, arduino::MbedSPI* spi, void (*fifo_callback)(Sensor_Event *event));
    ~IMU();
    int begin();
    int read_fifo();

    int enable_accel();
    int enable_gyro();

    int disable_accelerometer();
    int disable_gyro();

    struct SPI_Interface{
        int cs;
        arduino::MbedSPI *spi;
    };

    private:
    SPI_Interface spi_interface;
    void (*fifo_callback)(Sensor_Event *event);

    void* inv_icm;
};

