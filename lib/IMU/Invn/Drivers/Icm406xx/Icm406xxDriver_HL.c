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

#include "Icm406xxDefs.h"
#include "Icm406xxExtFunc.h"
#include "Icm406xxDriver_HL.h"

static int inv_icm406xx_configure_serial_interface(struct inv_icm406xx *s);
static int inv_icm406xx_init_hardware_from_ui(struct inv_icm406xx *s);

int inv_icm406xx_init(struct inv_icm406xx *s, struct inv_icm406xx_serif *serif,
                      void (*sensor_event_cb)(void* ctx, inv_icm406xx_sensor_event_t *event))
{
	int rc;

	memset(s, 0, sizeof(*s));

	s->serif = *serif;

	rc = inv_icm406xx_configure_serial_interface(s);
	if (rc != INV_ERROR_SUCCESS)
		return rc;

	// register the callback to be executed each time inv_icm406xx_get_data_from_fifo extracts
	// a packet from fifo.
	s->sensor_event_cb = sensor_event_cb;

	// initialize hardware
	rc = inv_icm406xx_init_hardware_from_ui(s);

	/* Workaround HW Bg 9863 : First Gyro data wrong after enable
	 * Keep track of the ICM time when enabling the Gyro and set a default value at init */
	s->gyro_start_time_us = UINT32_MAX;

	/* Gyro power-off to power-on transition can cause ring down issue
	 * This variable keeps track of timestamp when gyro is power off. Set to UINT32_MAX at init
	 */
	s->gyro_power_off_tmst = UINT32_MAX;

	return rc;
}

int inv_icm406xx_device_reset(struct inv_icm406xx *s)
{
	int                             status  = 0;
	int                             timeout = 1000; /* 1s */
	uint8_t                         data;
	ICM406XX_INTF_CONFIG4_AUX_SPI_t save_aux_spi;
	ICM406XX_INTF_CONFIG4_AP_SPI_t  save_ap_spi;

	/* Workaround Bug 9226 : Regarding interrupt output pin state in open-drain configuration
	INT1 pin is configured to be always in Push pull mode
	Before we do a SW reset, we need to set INT1_POLARITY = 0, and ensure the INT1 is not asserted.
	Otherwise the chip can go into Scan mode during SW reset
	*/
	status = inv_icm406xx_wr_int_source0(s, 0);
	status |= inv_icm406xx_wr_int_source1(s, 0);
	status |= inv_icm406xx_wr_int_cfg_int1_polarity(s, ICM406XX_INT_CONFIG_INT1_POLARITY_LOW);

	// save registers necessary to perform soft reset while still keeping communication link alive
	status |= inv_icm406xx_rd_intf_config4_aux_spi(s, &save_aux_spi);
	status |= inv_icm406xx_rd_intf_config4_ap_spi(s, &save_ap_spi);

	// Reset the internal registers and restores the default settings.
	// The bit automatically clears to 0 once the reset is done.
	status |= inv_icm406xx_wr_device_config_reset(s, ICM406XX_DEVICE_CONFIG_RESET_EN);
	if (status)
		return status;

	// Wait 250µs for soft reset to be effective before trying to perform any further read
	inv_icm406xx_sleep_us(250);

	do {
		status |= inv_icm406xx_wr_intf_config4_aux_spi(s, save_aux_spi);
		status |= inv_icm406xx_wr_intf_config4_ap_spi(s, save_ap_spi);
		status |= inv_icm406xx_rd_int_status(s, &data);
		if (status)
			return status;

		inv_icm406xx_sleep_us(1000);
		timeout -= 1;

		if (timeout < 0)
			return INV_ERROR_TIMEOUT;
	} while (data != BIT_INT_STATUS_RESET_DONE); // this is the default expected value

	return INV_ERROR_SUCCESS;
}

int inv_icm406xx_get_who_am_i(struct inv_icm406xx *s, uint8_t *who_am_i)
{
	return inv_icm406xx_rd_who_am_i(s, who_am_i);
}

int inv_icm406xx_enable_accel_low_power_mode(struct inv_icm406xx *s)
{
	int                              status = 0;
	ICM406XX_PWR_MGMT_0_ACCEL_MODE_t accel_mode;
	ICM406XX_PWR_MGMT_0_GYRO_MODE_t  gyro_mode;

	status = inv_icm406xx_rd_pwr_mgmt0_accel_mode(s, &accel_mode);
	status |= inv_icm406xx_rd_pwr_mgmt0_gyro_mode(s, &gyro_mode);

	// Enable accel and gyro in the FIFO
	// This is needed to switch to 16B FIFO packet format because it is required to have
	// FIFO timestamp in the packet and only 16B and 20B contain it.
	// Note that for this FIFO reconfiguration to be taken into account, it needs to be
	// emptied first (first enabling of either accel/gyro will need this condition and
	// second enabling of the other sensor will attempt to re-apply the same setting,
	// which will have no effect on a situation which does not require changing).
	if (accel_mode == ICM406XX_PWR_MGMT_0_ACCEL_MODE_OFF &&
	    gyro_mode == ICM406XX_PWR_MGMT_0_GYRO_MODE_OFF) {
		status |= inv_icm406xx_wr_fifo_config1_accel_en(s, ICM406XX_FIFO_CONFIG1_ACCEL_EN);
		status |= inv_icm406xx_wr_fifo_config1_gyro_en(s, ICM406XX_FIFO_CONFIG1_GYRO_EN);
	}
	status |= inv_icm406xx_wr_pwr_mgmt0_accel_mode(s, ICM406XX_PWR_MGMT_0_ACCEL_MODE_LP);
	inv_icm406xx_sleep_us(200);
	return status;
}

int inv_icm406xx_enable_accel_low_noise_mode(struct inv_icm406xx *s)
{
	int                              status = 0;
	ICM406XX_PWR_MGMT_0_ACCEL_MODE_t accel_mode;
	ICM406XX_PWR_MGMT_0_GYRO_MODE_t  gyro_mode;

	status = inv_icm406xx_rd_pwr_mgmt0_accel_mode(s, &accel_mode);
	status |= inv_icm406xx_rd_pwr_mgmt0_gyro_mode(s, &gyro_mode);

	// Enable accel and gyro in the FIFO
	// This is needed to switch to 16B FIFO packet format because it is required to have
	// FIFO timestamp in the packet and only 16B and 20B contain it.
	// Note that for this FIFO reconfiguration to be taken into account, it needs to be
	// emptied first (first enabling of either accel/gyro will need this condition and
	// second enabling of the other sensor will attempt to re-apply the same setting,
	// which will have no effect on a situation which does not require changing).
	if (accel_mode == ICM406XX_PWR_MGMT_0_ACCEL_MODE_OFF &&
	    gyro_mode == ICM406XX_PWR_MGMT_0_GYRO_MODE_OFF) {
		status |= inv_icm406xx_wr_fifo_config1_accel_en(s, ICM406XX_FIFO_CONFIG1_ACCEL_EN);
		status |= inv_icm406xx_wr_fifo_config1_gyro_en(s, ICM406XX_FIFO_CONFIG1_GYRO_EN);
	}
	status |= inv_icm406xx_wr_pwr_mgmt0_accel_mode(s, ICM406XX_PWR_MGMT_0_ACCEL_MODE_LN);
	inv_icm406xx_sleep_us(200);
	return status;
}

int inv_icm406xx_disable_accel(struct inv_icm406xx *s)
{
	int                             status = 0;
	ICM406XX_PWR_MGMT_0_GYRO_MODE_t gyro_mode;

	/* Workaround Bug 9077:When switching all sensors OFF sigpath may stay active with clock ON
		There is a SW fix for it which is: 
		- switching one sensor OFF at a time (at least 100us apart from each other) 
		- after switching the last sensor OFF the following procedure: 
		* wait 100 us 
		* switch the sensor ON again 
		* wait 150 us 
		* switch the sensor OFF again
	*/
	status = inv_icm406xx_wr_pwr_mgmt0_accel_mode(s, ICM406XX_PWR_MGMT_0_ACCEL_MODE_OFF);

	inv_icm406xx_sleep_us(100);

	status |= inv_icm406xx_rd_pwr_mgmt0_gyro_mode(s, &gyro_mode);
	if (gyro_mode == ICM406XX_PWR_MGMT_0_GYRO_MODE_OFF) {
		// Was the last sensor to be on
		status |= inv_icm406xx_wr_pwr_mgmt0_accel_mode(s, ICM406XX_PWR_MGMT_0_ACCEL_MODE_LN);
		inv_icm406xx_sleep_us(150);
		status |= inv_icm406xx_wr_pwr_mgmt0_accel_mode(s, ICM406XX_PWR_MGMT_0_ACCEL_MODE_OFF);
		/* Gyro is off and accel is about to be turned off.
		 * Due to OIS path enabling UI data in FIFO, sensors
		 * data are copied in the FIFO even when not wanted.
		 * When both sensors are off, further leakage can be
		 * prevented by disabling the sensors' data in the FIFO
		 */
		status |= inv_icm406xx_wr_fifo_config1_accel_en(s, ICM406XX_FIFO_CONFIG1_ACCEL_DIS);
		status |= inv_icm406xx_wr_fifo_config1_gyro_en(s, ICM406XX_FIFO_CONFIG1_GYRO_DIS);
		/* Reset FIFO explicitely so the new configuration is taken into account */
		status |= inv_icm406xx_reset_fifo(s);
	}
	return status;
}

int inv_icm406xx_enable_gyro_low_noise_mode(struct inv_icm406xx *s)
{
	int                              status = 0;
	ICM406XX_PWR_MGMT_0_ACCEL_MODE_t accel_mode;
	ICM406XX_PWR_MGMT_0_GYRO_MODE_t  gyro_mode;
	uint64_t                         current_time;

	/* Powering the gyroscope on immediately after powering it off can result in device failure. 
	 * The gyroscope proof mass can continue vibrating after it has been powered off, 
	 * and powering it back on immediately can result in unpredictable proof mass movement.
	 * After powering the gyroscope off, a period of > 150ms should be allowed to elapse before it is powered back on. */
	if (s->gyro_power_off_tmst != UINT32_MAX) {
		current_time = inv_icm406xx_get_time_us();
		/* Handle rollover */
		if (current_time <= s->gyro_power_off_tmst)
			current_time += UINT32_MAX;
		/* If 150 ms are not elapsed since power-off error is returned */
		if ((current_time - s->gyro_power_off_tmst) <= (150 * 1000))
			return INV_ERROR;
	}

	status = inv_icm406xx_rd_pwr_mgmt0_accel_mode(s, &accel_mode);
	status |= inv_icm406xx_rd_pwr_mgmt0_gyro_mode(s, &gyro_mode);

	// Enable accel and gyro in the FIFO.
	// This is needed to switch to 16B FIFO packet format because it is required to have
	// FIFO timestamp in the packet and only 16B and 20B contain it.
	// Note that for this FIFO reconfiguration to be taken into account, it needs to be
	// emptied first (first enabling of either accel/gyro will need this condition and
	// second enabling of the other sensor will attempt to re-apply the same setting,
	// which will have no effect on a situation which does not require changing).
	if (accel_mode == ICM406XX_PWR_MGMT_0_ACCEL_MODE_OFF &&
	    gyro_mode == ICM406XX_PWR_MGMT_0_GYRO_MODE_OFF) {
		status |= inv_icm406xx_wr_fifo_config1_gyro_en(s, ICM406XX_FIFO_CONFIG1_GYRO_EN);
		status |= inv_icm406xx_wr_fifo_config1_accel_en(s, ICM406XX_FIFO_CONFIG1_ACCEL_EN);
	}
	status |= inv_icm406xx_wr_pwr_mgmt0_gyro_mode(s, ICM406XX_PWR_MGMT_0_GYRO_MODE_LN);
	inv_icm406xx_sleep_us(200);

	/* Workaround HW Bug 9863 : 3 first data are wrong after gyro enable using IIR filter
	 There is no signal that says Gyro start-up has completed and data are stable using FIR filter
	 and the Gyro max start-up time is 40ms 
	 So keep track of the ICM time at start-up to discard the invalid data, about 60ms after enable 
	*/

	/* Enable the ICM time register reading */
	status |= inv_icm406xx_enable_timestamp_to_register(s);

	/* Get the ICM time */
	status |= inv_icm406xx_get_current_timestamp(s, &s->gyro_start_time_us);

	/* Disable the ICM time register reading to avoid any over consumption */
	status |= inv_icm406xx_disable_timestamp_to_register(s);

	return status;
}

int inv_icm406xx_disable_gyro(struct inv_icm406xx *s)
{
	int                              status = 0;
	ICM406XX_PWR_MGMT_0_ACCEL_MODE_t accel_mode;

	/* Workaround Bug 9077:When switching all sensors OFF sigpath may stay active with clock ON
		There is a SW fix for it which is: 
		- switching one sensor OFF at a time (at least 100us apart from each other) 
		- after switching the last sensor OFF the following procedure: 
		* wait 100 us 
		* switch the sensor ON again 
		* wait 150 us 
		* switch the sensor OFF again
	*/
	status |= inv_icm406xx_wr_pwr_mgmt0_gyro_mode(s, ICM406XX_PWR_MGMT_0_GYRO_MODE_OFF);

	/* keep track of gyro power-off time to check if gyro will be power-on after more than 150ms*/
	s->gyro_power_off_tmst = inv_icm406xx_get_time_us();

	inv_icm406xx_sleep_us(100);

	status |= inv_icm406xx_rd_pwr_mgmt0_accel_mode(s, &accel_mode);
	if (accel_mode == ICM406XX_PWR_MGMT_0_ACCEL_MODE_OFF) {
		// Was the last sensor to be on
		status |= inv_icm406xx_wr_pwr_mgmt0_gyro_mode(s, ICM406XX_PWR_MGMT_0_GYRO_MODE_LN);
		inv_icm406xx_sleep_us(150);
		status |= inv_icm406xx_wr_pwr_mgmt0_gyro_mode(s, ICM406XX_PWR_MGMT_0_GYRO_MODE_OFF);
		/* Accel is off and gyro is about to be turned off.
		 * Due to OIS path enabling UI data in FIFO, sensors
		 * data are copied in the FIFO even when not wanted.
		 * When both sensors are off, further leakage can be
		 * prevented by disabling the sensors' data in the FIFO
		 */
		status |= inv_icm406xx_wr_fifo_config1_gyro_en(s, ICM406XX_FIFO_CONFIG1_GYRO_DIS);
		status |= inv_icm406xx_wr_fifo_config1_accel_en(s, ICM406XX_FIFO_CONFIG1_ACCEL_DIS);
		/* Reset FIFO explicitely so the new configuration is taken into account */
		status |= inv_icm406xx_reset_fifo(s);
	}
	return status;
}

int inv_icm406xx_enable_fsync(struct inv_icm406xx *s)
{
	int status;

	//Enable Fsync
	status = inv_icm406xx_fsync_config_ui_sel(s, ICM406XX_FSYNC_CONFIG_UI_SEL_TEMP);
	status |= inv_icm406xx_wr_tmst_config_fsync_en(s, ICM406XX_TMST_CONFIG_TMST_FSYNC_EN);

	return status;
}

int inv_icm406xx_disable_fsync(struct inv_icm406xx *s)
{
	int status;

	// Disable Fsync
	status = inv_icm406xx_fsync_config_ui_sel(s, ICM406XX_FSYNC_CONFIG_UI_SEL_NO);
	status |= inv_icm406xx_wr_tmst_config_fsync_en(s, ICM406XX_TMST_CONFIG_TMST_FSYNC_DIS);

	return status;
}

int inv_icm406xx_configure_wom(struct inv_icm406xx *s, const uint8_t wom_x_th,
                               const uint8_t wom_y_th, const uint8_t wom_z_th)
{
	int status = 0;

	status |= inv_icm406xx_wr_accel_wom_x_thr(s, wom_x_th); // Set X threshold
	status |= inv_icm406xx_wr_accel_wom_y_thr(s, wom_y_th); // Set Y threshold
	status |= inv_icm406xx_wr_accel_wom_z_thr(s, wom_z_th); // Set Z threshold
	inv_icm406xx_sleep_us(1000);

	return status;
}

int inv_icm406xx_enable_wom(struct inv_icm406xx *s)
{
	int     status = 0;
	uint8_t data;

	// Enable WOM itself now that accel is correctly configured
	// Set WoM interrupt on OR of xyz axes interrupt.
	// Compare current sample with the previous sample.
	// Set SMD mode to WOM
	status |= inv_icm406xx_wr_smd_config_wom_int_mode(s, ICM406XX_SMD_CONFIG_WOM_INT_MODE_ORED);
	status |= inv_icm406xx_wr_smd_config_wom_mode(s, ICM406XX_SMD_CONFIG_WOM_MODE_CMP_PREV);
	status |= inv_icm406xx_wr_smd_config_smd_mode(s, ICM406XX_SMD_CONFIG_SMD_MODE_WOM);
	inv_icm406xx_sleep_us(1000);

	// Disable FIFO THS Interrupt
	status |= inv_icm406xx_rd_int_source0(s, &data);
	status |= inv_icm406xx_wr_int_source0(s, data & ~BIT_INT_SOURCE0_FIFO_THS_INT1_EN);

	// Set WOM Interrupt on INT1
	status |= inv_icm406xx_rd_int_source1(s, &data);
	data |= BIT_INT_SOURCE1_WOM_Z_INT1_EN | BIT_INT_SOURCE1_WOM_Y_INT1_EN |
	        BIT_INT_SOURCE1_WOM_X_INT1_EN;
	status |= inv_icm406xx_wr_int_source1(s, data);
	inv_icm406xx_sleep_us(50000);

	s->is_wom_enabled = 1;

	return status;
}

int inv_icm406xx_disable_wom(struct inv_icm406xx *s)
{
	int     status = 0;
	uint8_t data;

	// Set SMD mode to disabled
	status |= inv_icm406xx_wr_smd_config_smd_mode(s, ICM406XX_SMD_CONFIG_SMD_MODE_DISABLED);

	// Unset WOM Interrupt on INT1
	status |= inv_icm406xx_rd_int_source1(s, &data);
	data &= ~(BIT_INT_SOURCE1_WOM_Z_INT1_EN | BIT_INT_SOURCE1_WOM_Y_INT1_EN |
	          BIT_INT_SOURCE1_WOM_X_INT1_EN);
	status |= inv_icm406xx_wr_int_source1(s, data);

	// Re-Enable FIFO THS Interrupt
	status |= inv_icm406xx_rd_int_source0(s, &data);
	status |= inv_icm406xx_wr_int_source0(s, data | BIT_INT_SOURCE0_FIFO_THS_INT1_EN);

	s->is_wom_enabled = 0;

	return status;
}

int inv_icm406xx_get_data_from_fifo(void* ctx, struct inv_icm406xx *s)
{
	int      rc;
	uint8_t  int_status;
	uint16_t packet_count_i, packet_count = 0;
	uint16_t packet_size = FIFO_HEADER_SIZE + FIFO_ACCEL_DATA_SIZE + FIFO_GYRO_DATA_SIZE +
	                       FIFO_TEMP_DATA_SIZE + FIFO_TS_FSYNC_SIZE;
	fifo_header_t *header;

	/* Ensure data ready status bit is set */
	rc = inv_icm406xx_rd_int_status(s, &int_status);
	if (rc != INV_ERROR_SUCCESS)
		return rc;

	if ((int_status & BIT_INT_STATUS_FIFO_THS) || (int_status & BIT_INT_STATUS_FIFO_FULL)) {
		uint8_t data[2];
		/* FIFO record mode configured at driver init, so we read packet number, not byte count */
		rc = inv_icm406xx_rd_fifo_count(s, data);
		if (rc != INV_ERROR_SUCCESS)
			return rc;
		packet_count = (uint16_t)(data[0] | (data[1] << 8));

		if (packet_count > 0) {
			/* Read FIFO only when data is expected in FIFO */
			/* fifo_idx type variable must be large enough to parse the FIFO_MIRRORING_SIZE */
			uint16_t fifo_idx = 0;

			/* Workaround HW Bug 9450 : a double interrupt is triggered when read FIFO_DATA_REG after 100µs of first interrupt
			- FIFO that does not release properly the Watermark threshold, workaround is to read one byte more of MPUREG_FIFO_DATA_REG
			 In the case where packet count is greater than 1, fifo data have to be read by a single burst to avoid additional interrupt 
			 because FIFO_THS is asserted at the end of read operation.
			*/
			rc = inv_icm406xx_rd_fifo(s, packet_size * packet_count + 1, s->fifo_data);
			if (rc != INV_ERROR_SUCCESS) {
				/* sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
					reset FIFO and try next chance */
				inv_icm406xx_reset_fifo(s);
				return rc;
			}

			for (packet_count_i = 0; packet_count_i < packet_count; packet_count_i++) {
				inv_icm406xx_sensor_event_t event;
				event.sensor_mask = 0;

				header = (fifo_header_t *)&s->fifo_data[fifo_idx];
				fifo_idx += FIFO_HEADER_SIZE;

				/* Decode packet */
				if (header->bits.msg_bit) {
					/* MSG BIT set in FIFO header, Resetting FIFO */
					inv_icm406xx_reset_fifo(s);
					return INV_ERROR;
				}

				if (header->bits.accel_bit) {
					event.accel[0] = (s->fifo_data[1 + fifo_idx] << 8) | s->fifo_data[0 + fifo_idx];
					event.accel[1] = (s->fifo_data[3 + fifo_idx] << 8) | s->fifo_data[2 + fifo_idx];
					event.accel[2] = (s->fifo_data[5 + fifo_idx] << 8) | s->fifo_data[4 + fifo_idx];
					fifo_idx += FIFO_ACCEL_DATA_SIZE;

					/* new accel data */
					if ((event.accel[0] != INVALID_VALUE_FIFO) &&
					    (event.accel[1] != INVALID_VALUE_FIFO) &&
					    (event.accel[2] != INVALID_VALUE_FIFO))
						event.sensor_mask |= (1 << INV_ICM406XX_SENSOR_ACCEL);
				}

				if (header->bits.gyro_bit) {
					event.gyro[0] = (s->fifo_data[1 + fifo_idx] << 8) | s->fifo_data[0 + fifo_idx];
					event.gyro[1] = (s->fifo_data[3 + fifo_idx] << 8) | s->fifo_data[2 + fifo_idx];
					event.gyro[2] = (s->fifo_data[5 + fifo_idx] << 8) | s->fifo_data[4 + fifo_idx];
					fifo_idx += FIFO_GYRO_DATA_SIZE;

					/* new gyro data */
					/* Workaround HW Bug 9863 : 3 first data are wrong after gyro enable using IIR filter
					 There is no signal that says Gyro start-up has completed and data are stable using FIR filter
					 and the Gyro max start-up time is 40ms 
					 So test if the start-up time to discard the Gyro data is elapsed
					*/
					if ((s->gyro_start_time_us == UINT32_MAX) &&
					    (event.gyro[0] != INVALID_VALUE_FIFO) &&
					    (event.gyro[1] != INVALID_VALUE_FIFO) &&
					    (event.gyro[2] != INVALID_VALUE_FIFO))
						event.sensor_mask |= (1 << INV_ICM406XX_SENSOR_GYRO);
				}

				if ((header->bits.accel_bit) || (header->bits.gyro_bit)) {
					event.temperature =
					    (int8_t)s->fifo_data
					        [0 +
					         fifo_idx]; /* cast to int8_t since FIFO is in 16 bits mode (temperature on 8 bits) */
					fifo_idx += FIFO_TEMP_DATA_SIZE;

					/* new temperature data */
					if (event.temperature != INVALID_VALUE_FIFO_1B)
						event.sensor_mask |= (1 << INV_ICM406XX_SENSOR_TEMPERATURE);
				}

				if ((header->bits.timestamp_bit) || (header->bits.fsync_bit)) {
					event.timestamp_fsync =
					    (s->fifo_data[1 + fifo_idx] << 8) | s->fifo_data[0 + fifo_idx];
					fifo_idx += FIFO_TS_FSYNC_SIZE;

					/* new fsync event */
					if (header->bits.fsync_bit) {
						/*
						 * Workaround HW Bug #9700 : TMST_FSYNC and Delta Time stamp in FIFO may overflow with one second period
						 * Whenever FSYNC pulse happens on counter wrapping time, the timestamp overflows, it returns 65536 (-1) value
						 * This issue may occur if the FSYNC signal is triggered < 50us before sampling
						 * 
						 * Given the very low-occurence of the issue, return FSYNC dt = 25us instead and still trigger the event
						 */
						if (event.timestamp_fsync == UINT16_MAX)
							event.timestamp_fsync = 25;

						event.sensor_mask |= (1 << INV_ICM406XX_SENSOR_FSYNC_EVENT);
					}
				}

				/* Workaround HW Bug 9863 : 3 first data are wrong after gyro enable using IIR filter
				 There is no signal that says Gyro start-up has completed and data are stable using FIR filter
				 and the Gyro max start-up time is 40ms 
				 So test the delta time from Gyro start-up to valid Gyro data sampled and discard any Gyro data sampled before 60ms
				*/
				if ((header->bits.gyro_bit) && (s->gyro_start_time_us != UINT32_MAX) &&
				    (event.gyro[0] != INVALID_VALUE_FIFO) &&
				    (event.gyro[1] != INVALID_VALUE_FIFO) &&
				    (event.gyro[2] != INVALID_VALUE_FIFO) && (!header->bits.fsync_bit)) {
					uint32_t timestamp_us;

					/* Compute the delta time and handle a possible rollover */
					if ((uint32_t)(event.timestamp_fsync * 16 /* timestamp resolution is 16us */) <
					    s->gyro_start_time_us)
						timestamp_us = (event.timestamp_fsync + UINT16_MAX) * 16;
					else
						timestamp_us = event.timestamp_fsync * 16;

					/* Once the start-up time for correct data is elapsed, notify the Gyro data sampled */
					if (timestamp_us - s->gyro_start_time_us >= 60000) {
						s->gyro_start_time_us = UINT32_MAX;
						event.sensor_mask |= (1 << INV_ICM406XX_SENSOR_GYRO);
					}
				}

				/* call sensor event callback */
				if (s->sensor_event_cb)
					s->sensor_event_cb(ctx, &event);
			} /* end of FIFO read for loop */
		}
		/*else: packet_count was 0*/
	}
	/*else: FIFO threshold was not reached and FIFO was not full*/

	return INV_ERROR_SUCCESS;
}

int inv_icm406xx_set_accel_frequency(struct inv_icm406xx *              s,
                                     const ICM406XX_ACCEL_CONFIG0_ODR_t frequency)
{
	return inv_icm406xx_wr_accel_config0_odr(s, frequency);
}

int inv_icm406xx_set_gyro_frequency(struct inv_icm406xx *             s,
                                    const ICM406XX_GYRO_CONFIG0_ODR_t frequency)
{
	return inv_icm406xx_wr_gyro_config0_odr(s, frequency);
}

int inv_icm406xx_set_accel_fsr(struct inv_icm406xx *s, ICM406XX_ACCEL_CONFIG0_FS_SEL_t accel_fsr_g)
{
	return inv_icm406xx_wr_accel_config0_fs_sel(s, accel_fsr_g);
}

int inv_icm406xx_set_gyro_fsr(struct inv_icm406xx *s, ICM406XX_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps)
{
	return inv_icm406xx_wr_gyro_config0_fs_sel(s, gyro_fsr_dps);
}

int inv_icm406xx_reset_fifo(struct inv_icm406xx *s)
{
	int                              rc;
	ICM406XX_PWR_MGMT_0_ACCEL_MODE_t acc_mode;
	ICM406XX_PWR_MGMT_0_GYRO_MODE_t  gyr_mode;

	int     status;
	uint8_t saved;

	rc = inv_icm406xx_rd_pwr_mgmt0_gyro_mode(s, &gyr_mode);
	if (rc != INV_ERROR_SUCCESS)
		return rc;

	rc = inv_icm406xx_rd_pwr_mgmt0_accel_mode(s, &acc_mode);
	if (rc != INV_ERROR_SUCCESS)
		return rc;

	/* Workaround HW Bug 9052 only works if at least one sensor is enabled and if accel isn't in low power mode*/
	if (((gyr_mode != ICM406XX_PWR_MGMT_0_GYRO_MODE_OFF) ||
	     (acc_mode != ICM406XX_PWR_MGMT_0_ACCEL_MODE_OFF)) &&
	    (acc_mode != ICM406XX_PWR_MGMT_0_ACCEL_MODE_LP)) {
		uint8_t burst_read[3];
		/*
		Bug 9052: FIFO-count mis-match due to FIFO-flush is not getting cleared if there is no serial interface activity
		The cleanest way to issue a FIFO_FLUSH should be as follows:
		- Stop DMA data feeding to FIFO by setting fifo_accel_en, fifo_gyro_en and fifo_hires_en to 0 in FIFO_CONFIG1 register; 
		- Set the FIFO_FLUSH bit;
		- Wait for 100us;
		- Make a read burst of 3 bytes; 
		- Enable back DMA data generation to FIFO by setting fifo_accel_en, fifo_gyro_en and fifo_hires_en back to 1;
		*/
		status = inv_icm406xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &saved);
		status |= inv_icm406xx_wr_fifo_config1_hires(s, ICM406XX_FIFO_CONFIG1_HIRES_DIS);
		status |= inv_icm406xx_wr_fifo_config1_gyro_en(s, ICM406XX_FIFO_CONFIG1_GYRO_DIS);
		status |= inv_icm406xx_wr_fifo_config1_accel_en(s, ICM406XX_FIFO_CONFIG1_ACCEL_DIS);
		status |=
		    inv_icm406xx_wr_signal_path_rst_fifo_flush(s, ICM406XX_SIGNAL_PATH_RESET_FIFO_FLUSH_EN);
		inv_icm406xx_sleep_us(100);
		status |= inv_icm406xx_read_reg(s, MPUREG_WHO_AM_I, 3, burst_read);
		status |= inv_icm406xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &saved);
	}
	/* Workaround HW Bug 9052 if sensors are disabled */
	else {
		uint8_t                     dummy;
		ICM406XX_FIFO_CONFIG_MODE_t saved_fifo_mode;
		/* Change the FIFO_MODE to “bypass” (00) mode to force the FIFO reset, potential remaining data will be flushed 
		 * Then proceed to a dummy read to released the FIFO reset synchronously with the serial clock
		 */
		status = inv_icm406xx_rd_fifo_config_mode(s, &saved_fifo_mode);
		status |= inv_icm406xx_wr_fifo_config_mode(s, ICM406XX_FIFO_CONFIG_MODE_BYPASS);
		status |= inv_icm406xx_wr_fifo_config_mode(s, saved_fifo_mode);
		status |= inv_icm406xx_rd_who_am_i(s, &dummy);
	}

	return status;
}

int inv_icm406xx_enable_timestamp_to_register(struct inv_icm406xx *s)
{
	int status = 0;

	if (!s->tmst_to_reg_en_cnt) {
		/* 
		 * Enable the 20-bits timestamp register reading
		 * It's needed to wait at least 200us before doing the strobe 
		 */
		status |= inv_icm406xx_wr_tmst_config_tmst_to_reg(s, ICM406XX_TMST_CONFIG_TMST_TO_REGS_EN);

		inv_icm406xx_sleep_us(200);
	}
	s->tmst_to_reg_en_cnt++;

	return status;
}

int inv_icm406xx_disable_timestamp_to_register(struct inv_icm406xx *s)
{
	int status = 0;

	if (!s->tmst_to_reg_en_cnt) {
		/* 
		 * Disable the 20-bits timestamp register reading
		 */
		status |= inv_icm406xx_wr_tmst_config_tmst_to_reg(s, ICM406XX_TMST_CONFIG_TMST_TO_REGS_DIS);
	}
	s->tmst_to_reg_en_cnt--;

	return status;
}

int inv_icm406xx_get_current_timestamp(struct inv_icm406xx *s, uint32_t *icm_time)
{
	int     status;
	uint8_t data[3];

	/* 
	 * Enable timestamp counter to be latched in timestamp register 
	 */
	status =
	    inv_icm406xx_wr_signal_path_rst_tmst_strobe(s, ICM406XX_SIGNAL_PATH_RESET_TMST_STROBE_EN);

	/* 
	 * Get ICM timestamp 
	 */
	status |= inv_icm406xx_rd_tmst_val(s, data);
	*icm_time = ((uint32_t)(data[2] & 0x0F) << 16) + ((uint32_t)data[1] << 8) + data[0];

	return status;
}

/*
 * Static functions definition
 */

static int inv_icm406xx_configure_serial_interface(struct inv_icm406xx *s)
{
	int rc;

	switch (s->serif.serif_type) {
	case ICM406XX_UI_I2C:
		// nothing to do to set I2C link up.
		rc = INV_ERROR_SUCCESS;
		break;

	case ICM406XX_UI_SPI4:
		rc = inv_icm406xx_wr_intf_config4_ap_spi(s, ICM406XX_INTF_CONFIG4_AP_SPI4W);
		break;

	case ICM406XX_UI_SPI3:
		rc = inv_icm406xx_wr_intf_config4_ap_spi(s, ICM406XX_INTF_CONFIG4_AP_SPI3W);
		break;

	default:
		rc = INV_ERROR_BAD_ARG;
	}

	return rc;
}

static int inv_icm406xx_init_hardware_from_ui(struct inv_icm406xx *s)
{
	int     status = 0;
	uint8_t data;

	// Based on datasheet, start up time for register read/write after POR is 1ms and supply ramp time is 3ms
	inv_icm406xx_sleep_us(3000);

	/* Reset device */
	status |= inv_icm406xx_device_reset(s);

	// Setup MEMs properties.
	status |= inv_icm406xx_wr_gyro_config0_fs_sel(s, ICM406XX_GYRO_CONFIG0_FS_SEL_2000dps);
	status |= inv_icm406xx_wr_accel_config0_fs_sel(s, ICM406XX_ACCEL_CONFIG0_FS_SEL_4g);

	/* make sure FIFO is disabled */
	status |= inv_icm406xx_wr_fifo_config_mode(s, ICM406XX_FIFO_CONFIG_MODE_BYPASS);

	/* Configure:
	  - FIFO record mode i.e FIFO count unit is packet 
	  - FIFO snapshot mode i.e drop the data when the FIFO overflows
	  - Timestamp is logged in FIFO
	  - Little Endian fifo_count and fifo_data
	*/

	/*
	
	status |=
	    inv_icm406xx_wr_intf_config0_fifo_count_rec(s, ICM406XX_INTF_CONFIG0_FIFO_COUNT_REC_RECORD);
	status |= inv_icm406xx_wr_intf_config0_fifo_count_endian(
	    s, ICM406XX_INTF_CONFIG0_FIFO_COUNT_LITTLE_ENDIAN);
	status |= inv_icm406xx_wr_intf_config0_data_endian(s, ICM406XX_INTF_CONFIG0_DATA_LITTLE_ENDIAN);
	status |= inv_icm406xx_wr_fifo_config_mode(s, ICM406XX_FIFO_CONFIG_MODE_STREAM); // we want stream to fifo mode so fifo is up-to-date with latest packets
	status |= inv_icm406xx_wr_tmst_config_en(s, ICM406XX_TMST_CONFIG_TMST_EN);

	// Enable Interrupts.
	// Set fifo_wm_int_w generating condition : fifo_wm_int_w generated when counter >= threshold;
	status |= inv_icm406xx_wr_fifo_config1_wm_gt_th(s, ICM406XX_FIFO_CONFIG1_WM_GT_TH_EN);
	// Configure FIFO WM so that INT is triggered for each packet
	data = 0x1;
	status |= inv_icm406xx_wr_fifo_config2(s, data);
	// Enable FIFO THS Interrupt
	status |= inv_icm406xx_wr_int_source0(s, BIT_INT_SOURCE0_FIFO_THS_INT1_EN);
	// Configure the INT1 interrupt pulse as active high
	status |= inv_icm406xx_wr_int_cfg_int1_polarity(s, ICM406XX_INT_CONFIG_INT1_POLARITY_HIGH);
	// Enable push pull on INT1 
	status |= inv_icm406xx_wr_int_cfg_int1_drive_circuit(s, ICM406XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_PP);
	
	// Workaround HW Bug 9139: GLS: asy_rst_disable=0 is causing timing violations in interrupt output FSM
	// SW workaround: use only asy_rst_disable=1 configuration or only latched mode configuration,
	// which are both safe and robust enough, in Havana A1
	status |= inv_icm406xx_wr_int_config1_asy_rst_dis(s, ICM406XX_INT_CONFIG1_ASY_RST_DISABLED);

	*/
	// Desactivate FSYNC by default
	status |= inv_icm406xx_wr_tmst_config_fsync_en(s, ICM406XX_TMST_CONFIG_TMST_FSYNC_DIS);

	// Set default timestamp resolution 16us (Mobile use cases)
	status |= inv_icm406xx_wr_tmst_config_resolution(s, ICM406XX_TMST_CONFIG_RESOL_16us);

	/* restart and reset FIFO configuration */
	/*
	status |= inv_icm406xx_wr_fifo_config1_temp_en(s, ICM406XX_FIFO_CONFIG1_TEMP_EN);
	status |= inv_icm406xx_wr_fifo_config1_gyro_en(s, ICM406XX_FIFO_CONFIG1_GYRO_DIS);
	status |= inv_icm406xx_wr_fifo_config1_accel_en(s, ICM406XX_FIFO_CONFIG1_ACCEL_DIS);
	status |= inv_icm406xx_wr_fifo_config1_tmst_fsync(s, ICM406XX_FIFO_CONFIG1_TMST_FSYNC_EN);
	*/

	/* reset wom enable status */
	s->is_wom_enabled = 0;

	return status;
}
