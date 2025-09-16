/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
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

#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/ErrorHelper.h"

#include "Invn/Helpers/Icm406xx/helperClockCalib.h"

/* forward declaration */
static int     clock_calibration_initial(struct inv_icm406xx *s, struct clk_calib *clk_cal);
static int64_t apply_time_factor(struct inv_icm406xx *s, struct clk_calib *clk_cal, int64_t dt_fifo,
                                 float time_factor);
static int     clock_calibration_update_coef(struct inv_icm406xx *s, struct clk_calib *clk_cal,
                                             uint8_t *on_going);
static int     helper_get_fastest_running_odr(struct inv_icm406xx *s, uint32_t *odr);
static uint32_t helper_convert_odr_bitfield_to_us(uint32_t odr_bitfield);

static int64_t apply_time_factor(struct inv_icm406xx *s, struct clk_calib *clk_cal, int64_t dt_fifo,
                                 float time_factor)
{
	float dt_float    = ((float)(dt_fifo)*time_factor) + clk_cal->dt_error;
	clk_cal->dt_error = dt_float - (int64_t)(dt_float);

	return (int64_t)(dt_float);
}

int inv_helper_extend_timestamp_from_fifo(struct inv_icm406xx *s, struct clk_calib *clk_cal,
                                          uint16_t timestamp_fsync, uint64_t irq_timestamp,
                                          int sensor_mask, uint64_t *timestamp)
{
	// by default we assume there is nothing in FIFO. tiemstamp is reset.
	*timestamp = 0;

	if (sensor_mask & (1 << INV_ICM406XX_SENSOR_FSYNC_EVENT)) {
		// FSYNC event, so timestamp from FIFO not used
		*timestamp                                         = irq_timestamp;
		clk_cal->fifo_timestamp[INV_ICM406XX_SENSOR_ACCEL] = 0xDEADBEEF;
		clk_cal->fifo_timestamp[INV_ICM406XX_SENSOR_GYRO]  = 0xDEADBEEF;
		clk_cal->dt_error                                  = 0; // reset error on deltatime*/
	} else if ((sensor_mask & (1 << INV_ICM406XX_SENSOR_ACCEL)) &&
	           (sensor_mask & (1 << INV_ICM406XX_SENSOR_GYRO))) {
		// 2 sensors ACC + GYR in FIFO
		if ((clk_cal->first_data_after_enable[INV_ICM406XX_SENSOR_ACCEL]) &&
		    (clk_cal->first_data_after_enable[INV_ICM406XX_SENSOR_GYRO])) {
			// it is the first data after enable for both sensors, so timestamp from FIFO not used
			*timestamp        = irq_timestamp;
			clk_cal->dt_error = 0; // reset error on deltatime
		} else {
			enum inv_icm406xx_sensor sensor_id;
			int32_t                  dt_fifo;
			int64_t                  dt_us;

			if (clk_cal->first_data_after_enable[INV_ICM406XX_SENSOR_GYRO])
				// it is the first gyro data, use the accel last timestamp from FIFO
				sensor_id = INV_ICM406XX_SENSOR_ACCEL;
			else
				// otherwise use the gyro last timestamp from FIFO
				sensor_id = INV_ICM406XX_SENSOR_GYRO;

			if (clk_cal->fifo_timestamp[sensor_id] == 0xDEADBEEF) {
				// no last timestamp from FIFO to compute a valid delta, so timestamp from FIFO not used
				*timestamp        = irq_timestamp;
				clk_cal->dt_error = 0; // reset error on deltatime
			} else {
				// last timestamp from FIFO is avaliable to compute a valid dt based on read timestamp from FIFO
				dt_fifo = timestamp_fsync - (uint16_t)clk_cal->fifo_timestamp[sensor_id];

				if (dt_fifo > 0)
					dt_us =
					    apply_time_factor(s, clk_cal, ((int64_t)dt_fifo) << clk_cal->timestamp_res,
					                      clk_cal->time_factor);
				else
					dt_us = apply_time_factor(s, clk_cal,
					                          ((int64_t)((UINT16_MAX + 1) + dt_fifo))
					                              << clk_cal->timestamp_res,
					                          clk_cal->time_factor);

				*timestamp = clk_cal->last_timestamp_sent[sensor_id] + dt_us;
			}
		}
		clk_cal->fifo_timestamp[INV_ICM406XX_SENSOR_ACCEL]          = timestamp_fsync;
		clk_cal->fifo_timestamp[INV_ICM406XX_SENSOR_GYRO]           = timestamp_fsync;
		clk_cal->first_data_after_enable[INV_ICM406XX_SENSOR_ACCEL] = 0;
		clk_cal->first_data_after_enable[INV_ICM406XX_SENSOR_GYRO]  = 0;
		clk_cal->last_timestamp_sent[INV_ICM406XX_SENSOR_ACCEL]     = *timestamp;
		clk_cal->last_timestamp_sent[INV_ICM406XX_SENSOR_GYRO]      = *timestamp;
	} else {
		// 1 sensor ( ACC || GYR ) in FIFO
		enum inv_icm406xx_sensor sensor_id;
		if ((sensor_mask & (1 << INV_ICM406XX_SENSOR_ACCEL)))
			// ACC in FIFO
			sensor_id = INV_ICM406XX_SENSOR_ACCEL;
		else if ((sensor_mask & (1 << INV_ICM406XX_SENSOR_GYRO)))
			// GYR in FIFO
			sensor_id = INV_ICM406XX_SENSOR_GYRO;
		else {
			// TEMP in FIFO
			*timestamp                                         = irq_timestamp;
			clk_cal->fifo_timestamp[INV_ICM406XX_SENSOR_ACCEL] = 0xDEADBEEF;
			clk_cal->fifo_timestamp[INV_ICM406XX_SENSOR_GYRO]  = 0xDEADBEEF;
			clk_cal->dt_error                                  = 0; // reset error on deltatime*/
			return 0;
		}

		if (clk_cal->first_data_after_enable[sensor_id] ||
		    (clk_cal->fifo_timestamp[sensor_id] == 0xDEADBEEF)) {
			// it is the first data after enable
			// OR
			// no last timestamp from FIFO to compute a valid delta, so timestamp from FIFO not used
			*timestamp        = irq_timestamp;
			clk_cal->dt_error = 0; // reset error on deltatime
		} else {
			// last timestamp from FIFO is avaliable to compute a valid dt based on read timestamp from FIFO
			int64_t dt_us;
			int32_t dt_fifo = timestamp_fsync - (uint16_t)clk_cal->fifo_timestamp[sensor_id];

			if (dt_fifo > 0)
				dt_us = apply_time_factor(s, clk_cal, ((int64_t)dt_fifo) << clk_cal->timestamp_res,
				                          clk_cal->time_factor);
			else
				dt_us = apply_time_factor(
				    s, clk_cal, ((int64_t)((UINT16_MAX + 1) + dt_fifo)) << clk_cal->timestamp_res,
				    clk_cal->time_factor);

			*timestamp = clk_cal->last_timestamp_sent[sensor_id] + dt_us;
		}
		clk_cal->fifo_timestamp[sensor_id]          = timestamp_fsync;
		clk_cal->first_data_after_enable[sensor_id] = 0;
		clk_cal->last_timestamp_sent[sensor_id]     = *timestamp;
	}
	return 0;
}

void clock_calibration_reset(struct inv_icm406xx *s, struct clk_calib *clk_cal)
{
	/* Let's compute the number of samples before calibration based on the ODR configured at POR */
	clk_cal->initial_recalib_after_n_samples = NUMBER_OF_SAMPLES_FOR_ODR(
	    helper_convert_odr_bitfield_to_us(ICM406XX_ACCEL_CONFIG0_ODR_200_HZ));
	clk_cal->recalib_after_n_samples = clk_cal->initial_recalib_after_n_samples;
	clk_cal->on_going                = 0;
	clk_cal->dt_error                = 0;
	clk_cal->timestamp_res           = INV_ICM406XX_TIMESTAMP_RES_16USEC;

	clock_calibration_reset_sensors_stats(s, clk_cal, INV_ICM406XX_SENSOR_ACCEL);
	clock_calibration_reset_sensors_stats(s, clk_cal, INV_ICM406XX_SENSOR_GYRO);
}

void clock_calibration_reset_sensors_stats(struct inv_icm406xx *s, struct clk_calib *clk_cal,
                                           enum inv_icm406xx_sensor sensor)
{
	clk_cal->last_timestamp_sent[sensor]     = 0;
	clk_cal->fifo_timestamp[sensor]          = 0xDEADBEEF;
	clk_cal->first_data_after_enable[sensor] = 1;
}

int clock_calibration_restart(struct inv_icm406xx *s, struct clk_calib *clk_cal)
{
	int      rc;
	uint32_t smallest_running_odr;
	uint32_t smallest_running_odr_us;

	if ((rc = helper_get_fastest_running_odr(s, &smallest_running_odr)) != 0)
		return rc;

	/* in case no valid odr was returned, let's use the default odr value */
	if (smallest_running_odr == 0xffffffff)
		smallest_running_odr = 10;

	/* convert odr bit field to odr in microseconds */
	smallest_running_odr_us = helper_convert_odr_bitfield_to_us(smallest_running_odr);

	clk_cal->recalib_after_n_samples = NUMBER_OF_SAMPLES_FOR_ODR(smallest_running_odr_us);
	clk_cal->on_going                = 0;

	return rc;
}

int clock_calibration_init(struct inv_icm406xx *s, struct clk_calib *clk_cal)
{
	int rc;

	clock_calibration_reset(s, clk_cal);
	rc = clock_calibration_initial(s, clk_cal);

	return rc;
}

static int clock_calibration_initial(struct inv_icm406xx *s, struct clk_calib *clk_cal)
{
	uint32_t tICM_1, tICM_2;
	uint64_t tMCU_1, tMCU_2;
	int      status;
	uint8_t  nb_rollover;
	/*
	 * Enable the ICM time register reading
	 */
	status = inv_icm406xx_enable_timestamp_to_register(s);

	/*
	 * Power ON the ICM
	 * Enable gyro (the gyro clock is used for combo accel/gyro clocking)
	 * But disable interrupt since we don't want the data here
	 */
	status |= inv_icm406xx_wr_int_source0(s, 0);
	status |= inv_icm406xx_enable_gyro_low_noise_mode(s);

	/*
	 * Wait 200ms for MEMS start-up
	 */
	tMCU_1 = inv_icm406xx_get_time_us();
	while (inv_icm406xx_get_time_us() < (tMCU_1 + 200000))
		;

	/*
	 * Get the MCU and ICM time
	 */
	inv_helper_disable_irq();
	status |= inv_icm406xx_get_current_timestamp(s, &tICM_1);
	tMCU_1 = inv_icm406xx_get_time_us();
	inv_helper_enable_irq();

	/* 
	 * Wait 200ms 
	 */
	while (inv_icm406xx_get_time_us() < (tMCU_1 + (200 * 1000)))
		;

	/*
	 * Get the MCU and ICM time once again
	 */
	inv_helper_disable_irq();
	status |= inv_icm406xx_get_current_timestamp(s, &tICM_2);
	tMCU_2 = inv_icm406xx_get_time_us();
	inv_helper_enable_irq();

	/*
	 * Compute the time factor in order to align the MCU time to the icm406xx timestamp outputted from the FIFO 
	 */
	nb_rollover = (uint8_t)((tMCU_2 - tMCU_1) / TIME_US_FOR_20BITS_TMSP_ROLLOVER);
	if (tICM_2 > tICM_1)
		clk_cal->time_factor =
		    (tMCU_2 - tMCU_1) /
		    (float)(nb_rollover * TIME_US_FOR_20BITS_TMSP_ROLLOVER + tICM_2 - tICM_1);
	else
		clk_cal->time_factor =
		    (tMCU_2 - tMCU_1) /
		    (float)((nb_rollover + 1) * TIME_US_FOR_20BITS_TMSP_ROLLOVER + tICM_2 - tICM_1);
	/*
	 * Disable the 20-bits timestamp register reading
	 */
	status |= inv_icm406xx_disable_timestamp_to_register(s);

	/*
	 * Power OFF the ICM
	 * And re-enable interrupts
	 */
	status |= inv_icm406xx_disable_gyro(s);
	inv_icm406xx_sleep_us(150000);
	status |= inv_icm406xx_wr_int_source0(s, BIT_INT_SOURCE0_FIFO_THS_INT1_EN);

	/*
	 * Reset FIFO to remove unwanted events
	 */
	status |= inv_icm406xx_reset_fifo(s);

	return status;
}

/*
 * Update the calibration factor after N samples (60 x 20ms at default ODR, every 1200ms)
 */
int clock_calibration_update(struct inv_icm406xx *s, struct clk_calib *clk_cal)
{
	int rc = 0;

	if ((--clk_cal->recalib_after_n_samples) == 0) {
		uint32_t smallest_running_odr;
		uint32_t smallest_running_odr_us;

		rc = clock_calibration_update_coef(s, clk_cal, &clk_cal->on_going);

		/* now find the smallest running odr to program new calibration */
		if ((rc = helper_get_fastest_running_odr(s, &smallest_running_odr)) != 0) {
			/* in case off error we set recalib n sample to the initial one */
			clk_cal->recalib_after_n_samples = clk_cal->initial_recalib_after_n_samples;
			return rc;
		}
		/* in case no valid odr was returned, let's use the default odr value */
		if (smallest_running_odr == 0xffffffff)
			smallest_running_odr = 10;

		/* convert odr bit field to odr in microseconds */
		smallest_running_odr_us = helper_convert_odr_bitfield_to_us(smallest_running_odr);

		clk_cal->recalib_after_n_samples = NUMBER_OF_SAMPLES_FOR_ODR(smallest_running_odr_us);
	}

	return rc;
}

static int clock_calibration_update_coef(struct inv_icm406xx *s, struct clk_calib *clk_cal,
                                         uint8_t *on_going)
{
	static uint64_t tMCU_1;
	uint64_t        tMCU_2;
	static uint32_t tICM_1, tICM_2;
	float           computed_coef_time;
	int             status = 0;
	uint8_t         nb_rollover;

	if (*on_going == 0) {
		/*
		* Get the MCU and ICM time
		*/
		inv_helper_disable_irq();
		status |= inv_icm406xx_get_current_timestamp(s, &tICM_1);
		tMCU_1 = inv_icm406xx_get_time_us();
		inv_helper_enable_irq();
		if (tICM_1 == tICM_2) {
			INV_MSG(INV_MSG_LEVEL_VERBOSE,
			        "helperClockCalib: Timestamp latching failed. Keeping %f as clock ceofficient.",
			        clk_cal->time_factor);
			return status;
		}
		*on_going = 1;
	} else if (*on_going == 1) {
		/*
		* Get the MCU and ICM time
		*/
		inv_helper_disable_irq();
		status |= inv_icm406xx_get_current_timestamp(s, &tICM_2);
		tMCU_2 = inv_icm406xx_get_time_us();
		inv_helper_enable_irq();
		if (tICM_2 == tICM_1) {
			INV_MSG(INV_MSG_LEVEL_VERBOSE,
			        "helperClockCalib: Timestamp latching failed. Keeping %f as clock ceofficient.",
			        clk_cal->time_factor);
			return status;
		}

		/*
		 * Compute the time factors, adding 20bits max value because of TIME_US_FOR_CLOCK_CALIBRATION which implies at least 1 full timestamp rollover
		 */
		nb_rollover = (uint8_t)((tMCU_2 - tMCU_1) / TIME_US_FOR_20BITS_TMSP_ROLLOVER);
		if (tICM_2 > tICM_1)
			computed_coef_time =
			    (tMCU_2 - tMCU_1) /
			    (float)(nb_rollover * TIME_US_FOR_20BITS_TMSP_ROLLOVER + tICM_2 - tICM_1);
		else
			computed_coef_time =
			    (tMCU_2 - tMCU_1) /
			    (float)((nb_rollover + 1) * TIME_US_FOR_20BITS_TMSP_ROLLOVER + tICM_2 - tICM_1);

		/* error management : only allow 90-110% variation around reference value */
		if ((computed_coef_time < 1.1 * clk_cal->time_factor) &&
		    (computed_coef_time > 0.9 * clk_cal->time_factor))
			clk_cal->time_factor = computed_coef_time;
		else
			INV_MSG(INV_MSG_LEVEL_ERROR,
			        "Bad calibration clock coefficient computed %f, skipping it and keeping %f",
			        computed_coef_time, clk_cal->time_factor);
		*on_going = 0;
	}
	return status;
}

static int helper_get_fastest_running_odr(struct inv_icm406xx *s, uint32_t *odr)
{
	int     rc;
	uint8_t value[3];

	uint32_t                         smallest_odr = 0xffffffff;
	ICM406XX_PWR_MGMT_0_ACCEL_MODE_t accel_mode;
	ICM406XX_PWR_MGMT_0_GYRO_MODE_t  gyro_mode;
	ICM406XX_ACCEL_CONFIG0_ODR_t     accel_odr;
	ICM406XX_GYRO_CONFIG0_ODR_t      gyro_odr;

	/* Let's get current sensors ODR or return
	 * Let's use only one burst read to get the three needed registers
	 */
	rc = inv_icm406xx_read_reg(s, MPUREG_PWR_MGMT_0, 3, value);
	if (rc)
		return rc;

	accel_mode = (ICM406XX_PWR_MGMT_0_ACCEL_MODE_t)(value[0] & BIT_PWR_MGMT_0_ACCEL_MODE_MASK);
	gyro_mode  = (ICM406XX_PWR_MGMT_0_GYRO_MODE_t)(value[0] & BIT_PWR_MGMT_0_GYRO_MODE_MASK);
	gyro_odr   = (ICM406XX_GYRO_CONFIG0_ODR_t)(value[1] & BIT_GYRO_CONFIG0_ODR_MASK);
	accel_odr  = (ICM406XX_ACCEL_CONFIG0_ODR_t)(value[2] & BIT_ACCEL_CONFIG0_ODR_MASK);

	/* now let's find the smallest ODR of running sensors among ACC and GYR */
	if (accel_mode != ICM406XX_PWR_MGMT_0_ACCEL_MODE_OFF)
		smallest_odr = (uint32_t)accel_odr;

	if (gyro_mode != ICM406XX_PWR_MGMT_0_GYRO_MODE_OFF)
		if ((uint32_t)gyro_odr < smallest_odr)
			smallest_odr = gyro_odr;

	*odr = smallest_odr;

	return rc;
}

static uint32_t helper_convert_odr_bitfield_to_us(uint32_t odr_bitfield)
{
	switch (odr_bitfield) {
	/* case ICM406XX_GYRO_CONFIG0_ODR_1_KHZ:  */
	case ICM406XX_ACCEL_CONFIG0_ODR_1_KHZ:
		return 1000;
	/* case ICM406XX_GYRO_CONFIG0_ODR_200_HZ: */
	case ICM406XX_ACCEL_CONFIG0_ODR_200_HZ:
		return 5000;
	/* case ICM406XX_GYRO_CONFIG0_ODR_100_HZ: */
	case ICM406XX_ACCEL_CONFIG0_ODR_100_HZ:
		return 10000;
	/* case ICM406XX_GYRO_CONFIG0_ODR_50_HZ:  */
	case ICM406XX_ACCEL_CONFIG0_ODR_50_HZ:
		return 20000;
	/* case ICM406XX_GYRO_CONFIG0_ODR_25_HZ:  */
	case ICM406XX_ACCEL_CONFIG0_ODR_25_HZ:
		return 40000;
	case ICM406XX_ACCEL_CONFIG0_ODR_12_5_HZ:
		return 80000;
	case ICM406XX_ACCEL_CONFIG0_ODR_6_25_HZ:
		return 160000;
	/* POR value */
	default:
		return 5000;
	}
}

/* Convert a period in usec in a frequency */
uint32_t period_us_to_frequency(const uint32_t period_us)
{
	uint32_t frequency = (1000000 / period_us);

	/* Round up frequency */
	if (period_us != 1000000 / frequency)
		frequency++;

	return frequency;
}
