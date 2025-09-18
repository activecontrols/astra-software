#pragma once
#include<stdint.h>


#ifndef SENSOR_EVENT_DEF_H_
#define SENSOR_EVENT_DEF_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Sensor event structure definition
 */
typedef struct {
	int      sensor_mask;
	uint16_t timestamp_fsync;
	int16_t  accel[3];
	int16_t  gyro[3];
	int16_t  temperature;
} inv_icm406xx_sensor_event_t;



#ifdef __cplusplus
}
#endif

#endif