/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
#ifndef _STANDALONE_HELPER_H_
#define _STANDALONE_HELPER_H_

#include <stdint.h>

#include "Invn/Drivers/Icm406xx/Icm406xxDefs.h"
#include "Invn/Drivers/Icm406xx/Icm406xxExtFunc.h"
#include "Invn/Drivers/Icm406xx/Icm406xxDriver_HL.h"

/*
 * The 20-bits timestamp rollover necessarily occured after ~1.05s (time resolution is 1bit/us)
 */
#define TIME_US_FOR_20BITS_TMSP_ROLLOVER 0x100000

/*
 * Time between 2 MCU/ICM clock calibrations and for 1 calibration
 */
#define TIME_US_FOR_CLOCK_CALIBRATION 1200000

/*
 * Number of neccessary rollover occured during the TIME_US_FOR_CLOCK_CALIBRATION
 */
#define NB_ROLLOVER (uint8_t)(TIME_US_FOR_CLOCK_CALIBRATION / TIME_US_FOR_20BITS_TMSP_ROLLOVER)

/*
 * Number of samples needed before computing the clock calibration coefficient 
 * To round up the division is critical when the number of samples is low, i.e when the sensor frequency set is low.
 */
#define NUMBER_OF_SAMPLES_FOR_ODR(odr_us)                                                          \
	TIME_US_FOR_CLOCK_CALIBRATION / (odr_us) + TIME_US_FOR_CLOCK_CALIBRATION % (odr_us);

/** @brief Icm406xx Timestamp resolution definition
 */
enum inv_icm406xx_timestamp_res {
	INV_ICM406XX_TIMESTAMP_RES_1USEC  = 0, /**< default mode for HMD-VR use cases  
	                                           the FIFO timestamp outputted 20-bits timestamp low bits */
	INV_ICM406XX_TIMESTAMP_RES_16USEC = 4 /**< default mode for mobile use cases  
	                                           the FIFO timestamp outputted 20-bits timestamp high bits (register value << 4) */
};

/* 
 * Coefficient to interpolate the linear regression between the ICM time and the MCU time
 */
typedef struct clk_calib {
	int     initial_recalib_after_n_samples;
	int     recalib_after_n_samples;
	uint8_t on_going;
	float   time_factor;
	uint8_t first_data_after_enable
	    [INV_ICM406XX_SENSOR_MAX]; /**< keep track of the first data after enable to feed the timestamp state machine  */
	uint64_t last_timestamp_sent
	    [INV_ICM406XX_SENSOR_MAX]; /**< last timestamp sent when using the extended timestamp API (may contain timestamp from FIFO or from upper layer)*/
	uint32_t fifo_timestamp
	    [INV_ICM406XX_SENSOR_MAX]; /**< keep track of the timestamp from the FIFO to compute a delta timestamp when available */
	float   dt_error; /**< keep track of the error on the timestamp due to the usec resolution */
	uint8_t timestamp_res;
} clk_calib_t;

/** @brief Extend timestamp from FIFO by getting time base from upper layer
 *  @param[in] states     placeholder to inv_icm406xx_t states
 *  @param[in] clk_calib  placeholder to clk_calib_t states
 *  @param[in] timestamp_fsync according which is selected :
 *         - timestamp counter  absolute time at 16us/1us rate resolution according to settings (16us by default)
 *         - fsync counter 
 * @param[in] irq_timestamp     interrupt data ready timestamp computing by upper layer
 * @param[in] packet_header    header of the FIFO packet whose timestamp is extended by this function
 * @param[out] timestamp extended timestamp
 */
int inv_helper_extend_timestamp_from_fifo(struct inv_icm406xx *s, struct clk_calib *clk_cal,
                                          uint16_t timestamp_fsync, uint64_t irq_timestamp,
                                          int sensor_mask, uint64_t *timestamp);

/** @brief Intitialisation function to be executed before clock_calibration_update
 *  @param[in] states     placeholder to inv_icm406xx_t states
 *  @param[in] clk_calib  placeholder to clk_calib_t states
 */
void clock_calibration_reset(struct inv_icm406xx *s, struct clk_calib *clk_cal);

/** @brief Reset calibration data linked to acc and gyr sensors. This is typically used when 
 *  enabling a sensor.
 *
 *  Note: This function should be called when enabling Accel or Gyro
 *
 *  @param[in] states     placeholder to inv_icm406xx_t states
 *  @param[in] clk_calib  placeholder to clk_calib_t states
 *  @param[in] sensor     sensor to be restarted (INV_ICM406XX_SENSOR_ACCEL or INV_ICM406XX_SENSOR_ACCEL)
 */
void clock_calibration_reset_sensors_stats(struct inv_icm406xx *s, struct clk_calib *clk_cal,
                                           enum inv_icm406xx_sensor);

/** @brief First calibration done just after start'up to calculate first time factor
 *  @param[in] states     placeholder to inv_icm406xx_t states
 *  @param[in] clk_calib  placeholder to clk_calib_t states
 */
int clock_calibration_init(struct inv_icm406xx *s, struct clk_calib *clk_cal);

/** @brief Update time factor for clock calibration
 *  @param[in] states     placeholder to inv_icm406xx_t states
 *  @param[in] clk_calib  placeholder to clk_calib_t states
 */
int clock_calibration_update(struct inv_icm406xx *s, struct clk_calib *clk_cal);

/** @brief Restart the computation of the time factor.
 * This function finds the smallest ODR of running sensors among ACC and GYR. Then it restarts
 * the time deviation computation based on this ODR.
 *
 *  Note: This function should be called when enabling Accel or Gyro or when changing ODR
 *
 *  @param[in] states     placeholder to inv_icm406xx_t states
 *  @param[in] clk_calib  placeholder to clk_calib_t states
 */
int clock_calibration_restart(struct inv_icm406xx *s, struct clk_calib *clk_cal);

/*!
 * \brief Converter function from period (in usec) to frequency (in Hz)
 */
uint32_t period_us_to_frequency(const uint32_t period_us);

/** @brief Hook to disable IRQ in low level.
 */
extern void inv_helper_disable_irq(void);

/** @brief Hook to enable IRQ in low level.
 */
extern void inv_helper_enable_irq(void);

#endif /* !_STANDALONE_HELPER_H_ */
