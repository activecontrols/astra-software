#include <Arduino.h>

#include "./Invn/Drivers/Icm406xx/Icm406xxExtFunc.h"


extern "C" {

void inv_icm406xx_sleep_us(uint32_t us){
    delayMicroseconds(us);
}


uint64_t inv_icm406xx_get_time_us(void){
    return micros();
}

}
