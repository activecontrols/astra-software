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

/** @defgroup DriverIcm406xxDriver_HL Icm406xx driver high level functions
 *  @brief High-level function to setup an Icm406xx device
 *  @ingroup  DriverIcm406xx
 *  @{
 */

/** @file Icm406xxDriver_HL.h
 * High-level function to setup an Icm406xx device
 */

#ifndef _INV_ICM406xx_DRIVER_HL_H_
#define _INV_ICM406xx_DRIVER_HL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Icm406xxDefs.h"
#include "Icm406xxTransport.h"

#include "Invn/InvError.h"

#include <stdint.h>
#include <string.h>

/** @brief Icm406xx maximum buffer size mirrored from FIFO at polling time
 *  @warning fifo_idx type variable must be large enough to parse the FIFO_MIRRORING_SIZE
 */
#define ICM406XX_FIFO_MIRRORING_SIZE 1041 // packet size * max_count + 1 = 16 * 65 + 1

/** @brief Sensor identifier for UI control and OIS function
 */
enum inv_icm406xx_sensor {
	INV_ICM406XX_SENSOR_ACCEL, /**< Accelerometer (UI control path) */
	INV_ICM406XX_SENSOR_GYRO, /**< Gyroscope (UI control path) */
	INV_ICM406XX_SENSOR_FSYNC_EVENT, /**< Used by OIS and UI control layers*/
	INV_ICM406XX_SENSOR_OIS, /**< Only used by OIS layer */
	INV_ICM406XX_SENSOR_TEMPERATURE, /**< Chip temperature, enabled by default. However,
	it will be reported only if Accel and/or Gyro are also enabled. The Temperature's ODR (Output Data Rate) will
	match the ODR of Accel or Gyro, or the fastest if both are enabled*/
	INV_ICM406XX_SENSOR_MAX
};

/** @brief Sensor event structure definition
 */
typedef struct {
	int      sensor_mask;
	uint16_t timestamp_fsync;
	int16_t  accel[3];
	int16_t  gyro[3];
	int16_t  temperature;
} inv_icm406xx_sensor_event_t;

/** @brief Icm406xx driver states definition
 */
struct inv_icm406xx {
	struct inv_icm406xx_serif serif; /**> Warning : this field MUST be the first one of 
										  struct icm406xx */

	void (*sensor_event_cb)(
	    inv_icm406xx_sensor_event_t
	        *event); /**> callback executed by inv_icm406xx_get_data_from_fifo function
																	   for each data packet extracted from fifo.
																	   This field may be NULL if inv_icm406xx_get_data_from_fifo
																	   is not used by application. */

	int gyro_ois_st_bias[3]; /**> collected bias values (lsb) during self test */
	int accel_ois_st_bias[3];
	int gyro_lp_st_bias[3];
	int accel_lp_st_bias[3];
	int gyro_st_bias[3];
	int accel_st_bias[3];

	uint8_t fifo_data[ICM406XX_FIFO_MIRRORING_SIZE]; /**>  FIFO mirroring memory area */

	uint8_t
	         tmst_to_reg_en_cnt; /**> internal counter to keep track of the timestamp to register access availability */
	uint32_t gyro_start_time_us; /**> internal state needed for Workaround HW Bug 9863 */
	uint8_t  endianess_data; /**> internal status of data endianess mode to poll correctly datas */
	uint8_t  is_wom_enabled; /**> internal status of wom */
	uint64_t
	    gyro_power_off_tmst; /**< This variable keeps track of timestamp when gyro is power off */
};

/** @brief Configure the serial interface used to access the device and execute hardware initialization.
 *
 *  This functions first configures serial interface passed in parameter to make sure device 
 *  is accessible both in read and write. Thus no serial access should be done before 
 *  succesfully executing the present function.
 *
 *  Then if requested serial interface is a primary interface (aka UI interface or AP 
 *  interface), this function initializes the device using the following hardware settings:
 *    - gyroscope fsr = 2000dps
 *    - accelerometer fsr = 4g
 *    - set timestamp resolution to 16us
 *    - enable FIFO mechanism with the following configuration:
 *        - FIFO record mode i.e FIFO count unit is packet 
 *        - FIFO snapshot mode i.e drop the data when the FIFO overflows
 *        - Timestamp is logged in FIFO
 *        - Little Endian fifo_count and fifo_data
 *        - generate FIFO threshold interrupt when packet count reaches FIFO watermark
 *        - set FIFO watermark to 1 packet
 *        - enable temperature and timestamp data to go to FIFO
 *
 *  In case requested serial interface is an auxliary interface (i.e. AUX1 or AUX2) this 
 *  function does not perform any hardware initialization as write accesses are restricted
 *  from auxiliary ports. Hardware should then be initialized through primary interface
 *  prior to using auxiliary interface.
 *
 *
 *  @param[in] s driver structure. Note that first field of this structure MUST be a struct 
 *  inv_icm406xx_serif.
 *
 *  @param[in] serif pointer on serial interface structure to be used to access icm406xx.
 *
 *  @param[in] sensor_event_cb callback executed by inv_icm406xx_get_data_from_fifo function
 *  each time it extracts some valid data from fifo. Thus this parameter is optional as long
 *  as inv_icm406xx_get_data_from_fifo function is not used.
 *
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_init(struct inv_icm406xx *s, struct inv_icm406xx_serif *serif,
                      void (*sensor_event_cb)(inv_icm406xx_sensor_event_t *event));

/** @brief Perform a soft reset of the device
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_device_reset(struct inv_icm406xx *s);

/** @brief return WHOAMI value
 *  @param[out] who_am_i WHOAMI for device
 *  @return     0 on success, negative value on error
 */
int inv_icm406xx_get_who_am_i(struct inv_icm406xx *s, uint8_t *who_am_i);

/** @brief Put accel in low power mode
 *  @return 0 on success, negative value on error.
 *  @details
 *  It enables accel and gyro data in the FIFO (so
 *  the packet format is 16 bytes). If called first,
 *  the configuration will be applied, otherwise it
 *  will be ignored if the FIFO is not empty (but since
 *  the new configuration is identical it is not a issue).
 */
int inv_icm406xx_enable_accel_low_power_mode(struct inv_icm406xx *s);

/** @brief Put accel in low noise mode
 *  @return 0 on success, negative value on error.
 *  @details
 *  It enables accel and gyro data in the FIFO (so
 *  the packet format is 16 bytes). If called first,
 *  the configuration will be applied, otherwise it
 *  will be ignored if the FIFO is not empty (but since
 *  the new configuration is identical it is not a issue).
 */
int inv_icm406xx_enable_accel_low_noise_mode(struct inv_icm406xx *s);

/** @brief Disable all 3 axes of accel
 *  @return 0 on success, negative value on error.
 *  @details
 *  If both accel and gyro are turned off as a result of this
 *  function, they will also be removed from the FIFO and a
 *  FIFO reset will be performed (to guarantee no side effects
 *  until the next enable sensor call)
 */
int inv_icm406xx_disable_accel(struct inv_icm406xx *s);

/** @brief Put gyro in low noise mode
 *  @return 0 on success, negative value on error.
 *  @details
 *  It enables gyro and accel data in the FIFO (so
 *  the packet format is 16 bytes). If called first,
 *  the configuration will be applied, otherwise it
 *  will be ignored if the FIFO is not empty (but since
 *  the new configuration is identical it is not a issue).
 */
int inv_icm406xx_enable_gyro_low_noise_mode(struct inv_icm406xx *s);

/** @brief Disable all 3 axes of gyro
 *  @return 0 on success, negative value on error.
 *  @details
 *  If both accel and gyro are turned off as a result of this
 *  function, they will also be removed from the FIFO and a
 *  FIFO reset will be performed (to guarantee no side effects
 *  until the next enable sensor call)
 */
int inv_icm406xx_disable_gyro(struct inv_icm406xx *s);

/** @brief Enable fsync tagging functionnality.
 *  In details it:
 *     - enables fsync
 *     - enables timestamp to registers. Once fysnc is enabled fsync counter is pushed to 
 *       fifo instead of timestamp. So timestamp is made available in registers. Note that 
 *       this increase power consumption.
 *     - enables fsync related interrupt
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_enable_fsync(struct inv_icm406xx *s);

/** @brief Disable fsync tagging functionnality.
 *  In details it:
 *     - disables fsync
 *     - disables timestamp to registers. Once fysnc is disabled  timestamp is pushed to fifo 
 *        instead of fsync counter. So in order to decrease power consumption, timestamp is no 
 *        more available in registers.
 *     - disables fsync related interrupt
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_disable_fsync(struct inv_icm406xx *s);

/** @brief  Enable Wake On Motion.
 *  @param[in] wom_x_th threshold value for the Wake on Motion Interrupt for X-axis accelerometer.
 *  @param[in] wom_y_th threshold value for the Wake on Motion Interrupt for Y-axis accelerometer.
 *  @param[in] wom_z_th threshold value for the Wake on Motion Interrupt for Z-axis accelerometer.
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_configure_wom(struct inv_icm406xx *s, const uint8_t wom_x_th,
                               const uint8_t wom_y_th, const uint8_t wom_z_th);

/** @brief  Enable Wake On Motion.
 *  note : WoM requests to have the accelerometer enabled to work. 
 *  As a consequence Fifo water-mark interrupt is disabled to only trigger WoM interrupts.
 *  To have good performance, it's recommended to set accelerometer ODR (Output Data Rate) to 20ms
 *  and the accelerometer in Low Power Mode.
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_enable_wom(struct inv_icm406xx *s);

/** @brief  Disable Wake On Motion.
 *  note : Fifo water-mark interrupt is re-enabled when WoM is disabled.
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_disable_wom(struct inv_icm406xx *s);

/** @brief Read all available packets from the FIFO. For each packet function builds a
 *  sensor event containing packet data and validity information. Then it calls 
 *  sensor_event_cb funtion passed in parameter of inv_icm406xx_init function for each 
 *  packet.
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_get_data_from_fifo(struct inv_icm406xx *s);

/** @brief Configure accel Output Data Rate
 *  @param[in] frequency The requested frequency. See @sa ICM406XX_ACCEL_CONFIG0_ODR_t
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_set_accel_frequency(struct inv_icm406xx *              s,
                                     const ICM406XX_ACCEL_CONFIG0_ODR_t frequency);

/** @brief Configure gyro Output Data Rate
 *  @param[in] frequency The requested frequency. See @sa ICM406XX_GYRO_CONFIG0_ODR_t
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_set_gyro_frequency(struct inv_icm406xx *             s,
                                    const ICM406XX_GYRO_CONFIG0_ODR_t frequency);

/** @brief Set accel full scale range
 *  @param[in] accel_fsr_g requested full scale range. See @sa ICM406XX_ACCEL_CONFIG0_FS_SEL_t.
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_set_accel_fsr(struct inv_icm406xx *s, ICM406XX_ACCEL_CONFIG0_FS_SEL_t accel_fsr_g);

/** @brief Set gyro full scale range
 *  @param[in] gyro_fsr_dps requested full scale range. See @sa ICM406XX_GYRO_CONFIG0_FS_SEL_t.
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_set_gyro_fsr(struct inv_icm406xx *s, ICM406XX_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps);

/** @brief reset ICM406XX fifo
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_reset_fifo(struct inv_icm406xx *s);

/** @brief Enable the 20bits-timestamp register access to read in a reliable way the strobed timestamp. To do that, the fine clock is forced enabled at some power cost.
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_enable_timestamp_to_register(struct inv_icm406xx *s);

/** @brief Disable the 20bits-timestamp register access. Register read always return 0's.
 *  @return 0 on success, negative value on error.
 */
int inv_icm406xx_disable_timestamp_to_register(struct inv_icm406xx *s);

/** @brief Get the timestamp value of icm406xx from register
 *  @param[in] icm_time timestamp read from register
 *  @return 0 on success, negative value on error.
 *  @warning Prior to call this API, the read access to timestamp register msut be enabled (see @ref inv_icm406xx_enable_timestamp_to_register() function)
 */
int inv_icm406xx_get_current_timestamp(struct inv_icm406xx *s, uint32_t *icm_time);

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM406xx_DRIVER_HL_H_ */

/** @} */
