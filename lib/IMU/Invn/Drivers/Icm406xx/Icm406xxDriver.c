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

/** @defgroup DriverIcm406xxUnit Icm406xx driver unit functions
 *  @brief Unit functions to access Icm406xx device registers
 *  @ingroup  DriverIcm406xx
 *  @{
 */

#include "Icm406xxDefs.h"
#include "Icm406xxExtFunc.h"
#include "Icm406xxTransport.h"

/** @brief Set DEVICE_CONFIG register SPI_MODE bits
 *
 *  Selects SPI_MODE[1:0] <br>
 *  When set to ‘1’ Mode1 and Mode2. <br>
 *  When set to ‘0’ Mode0 and Mode3.
 *
 *  @param[in] new_value See enum ICM406XX_DEVICE_CONFIG_SPI_MODE_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_device_config_spi_mode(struct inv_icm406xx *             s,
                                           ICM406XX_DEVICE_CONFIG_SPI_MODE_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_DEVICE_CONFIG, 1, &value);
	if (status)
		return status;

	value &= ~BIT_DEVICE_CONFIG_SPI_MODE_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_DEVICE_CONFIG, 1, &value);

	return status;
}

/** @brief Set DEVICE_CONFIG register SOFT_RESET_CONFIG bit
 *
 *  When set to ‘1’ Enable reset. <br>
 *  When set to ‘0’ Normal.
 *
 *  @param[in] new_value See enum ICM406XX_DEVICE_CONFIG_RESET_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_device_config_reset(struct inv_icm406xx *          s,
                                        ICM406XX_DEVICE_CONFIG_RESET_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_DEVICE_CONFIG, 1, &value);
	if (status)
		return status;

	value &= ~BIT_DEVICE_CONFIG_RESET_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_DEVICE_CONFIG, 1, &value);

	return status;
}

/** @brief Set INT_CONFIG register INT1_DRIVE_CIRCUIT bit
 *
 *  When set to ‘1’, Interrupt 1 is in push-pull.
 *  When set to ‘0’, Interrupt 1 is in open drain
 *
 *  @param[in] new_value See enum ICM406XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_int_cfg_int1_drive_circuit(struct inv_icm406xx *                    s,
                                               ICM406XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_INT_CONFIG, 1, &value);
	if (status)
		return status;

	value &= ~BIT_INT_CONFIG_INT1_DRIVE_CIRCUIT_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_INT_CONFIG, 1, &value);

	return status;
}

/** @brief Set INT_CONFIG register INT1_POLARITY bit
 *
 *  When set to ‘1’, Interrupt 1 is active high
 *  When set to ‘0’, Interrupt 1 is active low
 *
 *  @param[in] new_value See enum ICM406XX_INT_CONFIG_INT1_POLARITY_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_int_cfg_int1_polarity(struct inv_icm406xx *               s,
                                          ICM406XX_INT_CONFIG_INT1_POLARITY_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_INT_CONFIG, 1, &value);
	if (status)
		return status;

	value &= ~BIT_INT_CONFIG_INT1_POLARITY_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_INT_CONFIG, 1, &value);

	return status;
}

/** @brief Set FIFO_CONFIG register FIFO_MODE bit
 *
 *  When set to ‘2’, when the FIFO is full, additional writes will not be written to FIFO. <br>
 *  When set to ‘1’, when the FIFO is full, additional writes will be written to the FIFO, replacing the oldest data.
 *  When set to ‘0’, Disable the fifo.
 *
 *  @param[in] new_value See enum ICM406XX_FIFO_CONFIG_MODE_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_fifo_config_mode(struct inv_icm406xx *s, ICM406XX_FIFO_CONFIG_MODE_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_FIFO_CONFIG, 1, &value);
	if (status)
		return status;

	value &= ~BIT_FIFO_CONFIG_MODE_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_FIFO_CONFIG, 1, &value);

	return status;
}

/** @brief Read FIFO_CONFIG register FIFO_MODE bit
 *
 *  @param[out] value See enum ICM406XX_FIFO_CONFIG_MODE_t
 *                    ‘2’, when the FIFO is full, additional writes will not be written to FIFO.
 *                    ‘1’, when the FIFO is full, additional writes will be written to the FIFO, replacing the oldest data.
 *                    ‘0’, Disable the fifo.
 
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_fifo_config_mode(struct inv_icm406xx *s, ICM406XX_FIFO_CONFIG_MODE_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_FIFO_CONFIG, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_FIFO_CONFIG_MODE_MASK;

	return status;
}

/** @brief Get INT_STATUS register value
 *
 *  @param[out] * value Interrupt Status
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_int_status(struct inv_icm406xx *s, uint8_t *value)
{
	return inv_icm406xx_read_reg(s, MPUREG_INT_STATUS, 1, value);
}

/** @brief Get FIFO count from FIFO_COUNTL and FIFO_COUNTH registers
 *
 *  @param[out] value 16-bit FIFO count.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_fifo_count(struct inv_icm406xx *s, uint8_t value[2])
{
	return inv_icm406xx_read_reg(s, MPUREG_FIFO_COUNTH, 2, value);
}

/** @brief Read FIFO_DATA_REG
 *
 *  @param[in]  len Length of data to read, in bytes.
 *  @param[out] value FIFO data.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_fifo(struct inv_icm406xx *s, int len, uint8_t *value)
{
	return inv_icm406xx_read_reg(s, MPUREG_FIFO_DATA, len, value);
}

/** @brief Set SIGNAL_PATH_RESET register TMST_STROBE bit
 *
 *  <pre>
 *  1 – Timestamp counter is latched into timestamp register
 *  0 – Disable
 *  </pre>
 * @param[in] new_value See enum ICM406XX_SIGNAL_PATH_RESET_TMST_STROBE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_signal_path_rst_tmst_strobe(struct inv_icm406xx *                    s,
                                                ICM406XX_SIGNAL_PATH_RESET_TMST_STROBE_t new_value)
{
	uint8_t value;
	int     status;

	/* The signal_path_reset register does not need to be read first as it does not 
	 * contains any persistent information (just some activation bits for tmst 
	 * strobe and fifo flush).
	 * Thus let's save some precious time by not reading register. For more reliability
	 * let's make sure that this function only access the tmst_strobe bit by masking the 
	 * input parameter.
	 */
	value = new_value & BIT_SIGNAL_PATH_RESET_TMST_STROBE_MASK;

	status = inv_icm406xx_write_reg(s, MPUREG_SIGNAL_PATH_RESET, 1, &value);
	return status;
}

/** @brief Set SIGNAL_PATH_RESET register FIFO_FLUSH bit
 *
 *  <pre>
 *  1 – Fifo flush enable
 *  0 – Fifo flush disable
 *  </pre>
 * @param[in] new_value See enum ICM406XX_SIGNAL_PATH_RESET_FIFO_FLUSH_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_signal_path_rst_fifo_flush(struct inv_icm406xx *                   s,
                                               ICM406XX_SIGNAL_PATH_RESET_FIFO_FLUSH_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_SIGNAL_PATH_RESET, 1, &value);
	if (status)
		return status;

	value &= ~BIT_SIGNAL_PATH_RESET_FIFO_FLUSH_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_SIGNAL_PATH_RESET, 1, &value);
	return status;
}

/** @brief Set INTF_CONFIG0 register FIFO_SREG_INVALID_IND bit
 *
 *  Selects whether INVALID_IND[1:0] if INVALID_IND_DIS == 1
 *     In FIFO: the last Valid and New sample is held and written into the FIFO upon reception of cnt_sreg_update.
 *  If no Valid and New sample had been received since the reset, 0x0000 will be written in FIFO
 *     In Sensor Data Registers: the last Valid and New sample is held
 *  If no Valid and New sample had been received since the reset, the register default value is read back (that is 0x8000)
 *  @param[in] new_value See enum ICM406XX_INTF_CONFIG0_FIFO_SREG_INVALID_IND_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_intf_config0_fifo_sreg_invalid_ind(
    struct inv_icm406xx *s, ICM406XX_INTF_CONFIG0_FIFO_SREG_INVALID_IND_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_INTF_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_FIFO_SREG_INVALID_IND_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_INTF_CONFIG0, 1, &value);

	return status;
}

/** @brief Set INTF_CONFIG0 register FIFO_COUNT_REC bit
 *
 *  Selects whether FIFO_COUNT[1:0] indicates the number of records or number of bytes. <br>
 *  When set to ‘1’ report FIFO count in terms of records. <br>
 *  When set to ‘0’ report FIFO count in terms of bytes.
 *
 *  @param[in] new_value See enum ICM406XX_CONFIG_FIFO_COUNT_REC_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_intf_config0_fifo_count_rec(struct inv_icm406xx *                  s,
                                                ICM406XX_INTF_CONFIG0_FIFO_COUNT_REC_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_INTF_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_FIFO_COUNT_REC_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_INTF_CONFIG0, 1, &value);

	return status;
}

/** @brief Set INTF_CONFIG0 register FIFO_COUNT_ENDIAN bit
 *
 *  Selects whether FIFO_COUNT_ENDIAN[1:0] indicates the endianess. <br>
 *  When set to ‘1’ report FIFO count in big endian. <br>
 *  When set to ‘0’ report FIFO count in little endian.
 *
 *  @param[in] new_value See enum ICM406XX_INTF_CONFIG0_FIFO_COUNT_ENDIAN_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_intf_config0_fifo_count_endian(
    struct inv_icm406xx *s, ICM406XX_INTF_CONFIG0_FIFO_COUNT_ENDIAN_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_INTF_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_FIFO_COUNT_ENDIAN_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_INTF_CONFIG0, 1, &value);

	return status;
}

/** @brief Set INTF_CONFIG0 register DATA_ENDIAN bit
 *
 *  Selects whether DATA_ENDIAN[1:0] indicates the endianess. <br>
 *  When set to ‘1’ report data in big endian. <br>
 *  When set to ‘0’ report data in little endian.
 *
 *  @param[in] new_value See enum ICM406XX_INTF_CONFIG0_DATA_COUNT_ENDIAN_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_intf_config0_data_endian(struct inv_icm406xx *                     s,
                                             ICM406XX_INTF_CONFIG0_DATA_COUNT_ENDIAN_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_INTF_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_DATA_ENDIAN_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_INTF_CONFIG0, 1, &value);

	return status;
}

/** @brief Get INTF_CONFIG0 register DATA_ENDIAN bit
 *
 *  Selects whether DATA_ENDIAN[1:0] indicates the endianess. <br>
 *  When set to ‘1’ report data in big endian. <br>
 *  When set to ‘0’ report data in little endian.
 *
 *  @param[in] new_value See enum ICM406XX_INTF_CONFIG0_DATA_COUNT_ENDIAN_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_intf_config0_data_endian(struct inv_icm406xx *                      s,
                                             ICM406XX_INTF_CONFIG0_DATA_COUNT_ENDIAN_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_INTF_CONFIG0, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_DATA_ENDIAN_MASK;

	return status;
}

/** @brief Set INTF_CONFIG0 register SPI_MODE_AUX2 bit
 *
 *  Selects whether SPI_MODE_AUX2[1:0] indicates the SPI mode. <br>
 *  When set to ‘1’ Mode1 and Mode2. <br>
 *  When set to ‘0’ Mode0 and Mode3.
 *
 *  @param[in] new_value See enum ICM406XX_INTF_CONFIG0_SPI_MODE_OIS2_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_intf_config0_spi_mode_ois2(struct inv_icm406xx *                 s,
                                               ICM406XX_INTF_CONFIG0_SPI_MODE_OIS2_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_INTF_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_SPI_MODE_OIS2_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_INTF_CONFIG0, 1, &value);

	return status;
}

/** @brief Set INTF_CONFIG0 register SPI_MODE_OIS1 bit
 *
 *  Selects whether SPI_MODE_AUX1[1:0] indicates the SPI mode. <br>
 *  When set to ‘1’ Mode1 and Mode2. <br>
 *  When set to ‘0’ Mode0 and Mode3.
 *
 *  @param[in] new_value See enum ICM406XX_INTF_CONFIG0_SPI_MODE_OIS1_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_intf_config0_spi_mode_ois1(struct inv_icm406xx *                 s,
                                               ICM406XX_INTF_CONFIG0_SPI_MODE_OIS1_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_INTF_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_SPI_MODE_OIS1_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_INTF_CONFIG0, 1, &value);

	return status;
}

/** @brief Set INTF_CONFIG1 register ACCEL_LP_CLK_SEL bit
 *
 *  Selects whether ACCEL_LP_CLK_SEL indicates the Accelerometer LP mode. <br>
 *  When set to ‘1’ Accelerometer LP mode uses RC Oscilator clock <br>
 *  When set to ‘0’ Accelerometer LP mode uses Wake Up oscilator clock.
 *
 *  @param[in] new_value See enum ICM406XX_INTF_CONFIG1_ACCEL_LP_CLK_SEL_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_intf_config1_accel_lp_clk_sel(struct inv_icm406xx *                 s,
                                               ICM406XX_INTF_CONFIG1_ACCEL_LP_CLK_SEL_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_INTF_CONFIG1, 1, &value);
	if (status)
		return status;

	value &= ~BIT_ACCEL_LP_CLK_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_INTF_CONFIG1, 1, &value);
	inv_icm406xx_sleep_us(1000);

	return status;
}

/** @brief Set PWR_MGMT_0 register TEMP_DIS bit
 *
 *  <pre>
 *  1 – disable temperature sensor
 *  0 – enable temperature sensor
 *  </pre>
 * @param[in] new_value See enum ICM406XX_PWR_MGMT_0_TEMP_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_pwr_mgmt0_temp_en(struct inv_icm406xx *s, ICM406XX_PWR_MGMT_0_TEMP_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_PWR_MGMT_0_TEMP_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &value);

	return status;
}

/** @brief Read PWR_MGMT_0 register TEMP_DIS bit
 *
 *  <pre>
 *  1 – disable temperature sensor
 *  0 – enable temperature sensor
 *  </pre>
 * @param[out] value See enum ICM406XX_PWR_MGMT_0_TEMP_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_pwr_mgmt0_temp_en(struct inv_icm406xx *s, ICM406XX_PWR_MGMT_0_TEMP_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_PWR_MGMT_0_TEMP_MASK;

	return status;
}

/** @brief Set PWR_MGMT_0 register GYRO_MODE bit
 *
 *  <pre>
 *  3 – gyro low noise mode
 *  0 – gyro off
 *  </pre>
 * note : others modes are not supported for gyro on 406xx
 * @param[in] new_value See enum ICM406XX_PWR_MGMT_0_GYRO_MODE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_pwr_mgmt0_gyro_mode(struct inv_icm406xx *           s,
                                        ICM406XX_PWR_MGMT_0_GYRO_MODE_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_PWR_MGMT_0_GYRO_MODE_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &value);

	return status;
}

/** @brief Read PWR_MGMT_0 register GYRO_MODE bit
 *
 *  <pre>
 *  3 – gyro low noise mode
 *  0 – gyro off
 *  </pre>
 * note : others modes are not supported for gyro on 406xx
 * @param[out] value See enum ICM406XX_PWR_MGMT_0_GYRO_MODE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_pwr_mgmt0_gyro_mode(struct inv_icm406xx *            s,
                                        ICM406XX_PWR_MGMT_0_GYRO_MODE_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_PWR_MGMT_0_GYRO_MODE_MASK;

	return status;
}

/** @brief Set PWR_MGMT_0 register ACCEL_MODE bit
 *
 *  <pre>
 *  3 – accel low noise mode
 *  2 – accel low power mode
 *  0 – accel off
 *  </pre>
 * @param[in] new_value See enum ICM406XX_PWR_MGMT_0_ACCEL_MODE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_pwr_mgmt0_accel_mode(struct inv_icm406xx *            s,
                                         ICM406XX_PWR_MGMT_0_ACCEL_MODE_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &value);

	return status;
}

/** @brief Read PWR_MGMT_0 register ACCEL_MODE bit
 *
 *  <pre>
 *  3 – accel low noise mode
 *  2 – accel low power mode
 *  0 – accel off
 *  </pre>
 * @param[out] value See enum ICM406XX_PWR_MGMT_0_ACCEL_MODE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_pwr_mgmt0_accel_mode(struct inv_icm406xx *             s,
                                         ICM406XX_PWR_MGMT_0_ACCEL_MODE_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_PWR_MGMT_0_ACCEL_MODE_MASK;

	return status;
}

/** @brief Set GYRO_CONFIG0 register GYRO_FS_SEL bits
 *
 *  Gyro Full Scale Select: <br>
 *  000 = ±2000dps <br>
 *  001 = ±1000dps <br>
 *  010 = ±500dps <br>
 *  011 = ±250dps <br>
 *  100 = ±125dps <br>
 *  101 = ±62dps <br>
 *  110 = ±31dps <br>
 *  111 = ±16dps <br>
 *
 *  @param[in] new_value See enum ICM406XX_GYRO_CONFIG0_FS_SEL_t.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_gyro_config0_fs_sel(struct inv_icm406xx *          s,
                                        ICM406XX_GYRO_CONFIG0_FS_SEL_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_GYRO_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_GYRO_CONFIG0_FS_SEL_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_GYRO_CONFIG0, 1, &value);

	return status;
}
/** @brief Get GYRO_CONFIG0 register GYRO_FS_SEL bits
 *
 *  Gyro Full Scale Select: <br>
 *  000 = ±2000dps <br>
 *  001 = ±1000dps <br>
 *  010 = ±500dps <br>
 *  011 = ±250dps <br>
 *  100 = ±125dps <br>
 *  101 = ±62dps <br>
 *  110 = ±31dps <br>
 *  111 = ±16dps <br>
 *
 *  @param[out] value See enum ICM406XX_GYRO_CONFIG0_FS_SEL_t.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_gyro_config0_fs_sel(struct inv_icm406xx *           s,
                                        ICM406XX_GYRO_CONFIG0_FS_SEL_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_GYRO_CONFIG0, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_GYRO_CONFIG0_FS_SEL_MASK;

	return status;
}

/** @brief Set GYRO_CONFIG0 register GYRO_ODR bits
 *
 *  Gyro odr Select: <br>
 *  0 - 64K <br>
 *  1 - 32K <br>
 *  2 - 16K <br>
 *  3 - 8k <br>
 *  4 - 4k <br>
 *  5 - 2k <br>
 *  6 - 1k : 1ms <br>
 *  7 - 200 (default) : 5 ms <br>
 *  8 - 100 : 10 ms <br>
 *  9 - 50 : 20 ms <br>
 *  10 - 25 : 40 ms <br>
 *  11 - 12.5 : 80 ms <br>
 *  12 - 6.25 : 160 ms <br>
 *
 *  @param[in] new_value See enum ICM406XX_GYRO_CONFIG0_ODR_t.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_gyro_config0_odr(struct inv_icm406xx *s, ICM406XX_GYRO_CONFIG0_ODR_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_GYRO_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_GYRO_CONFIG0_ODR_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_GYRO_CONFIG0, 1, &value);

	return status;
}

/** @brief Get GYRO_CONFIG0 register GYRO_ODR bits
 *
 *  Gyro odr Select: <br>
 *  0 - 64K <br>
 *  1 - 32K <br>
 *  2 - 16K <br>
 *  3 - 8k <br>
 *  4 - 4k <br>
 *  5 - 2k <br>
 *  6 - 1k : 1ms <br>
 *  7 - 200 (default) : 5 ms <br>
 *  8 - 100 : 10 ms <br>
 *  9 - 50 : 20 ms <br>
 *  10 - 25 : 40 ms <br>
 *  11 - 12.5 : 80 ms <br>
 *  12 - 6.25 : 160 ms <br>
 *
 *  @param[in] new_value See enum ICM406XX_GYRO_CONFIG0_ODR_t.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_gyro_config0_odr(struct inv_icm406xx *s, ICM406XX_GYRO_CONFIG0_ODR_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_GYRO_CONFIG0, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_GYRO_CONFIG0_ODR_MASK;

	return status;
}

/** @brief Set ACCEL_CONFIG0 register ACCEL_FS_SEL bits
 *
 *  Accel Full Scale Select: <br>
 *  000 = ±16g <br>
 *  001 = ±8g <br>
 *  010 = ±4g <br>
 *  011 = ±2g <br>
 *  100 = ±1g <br>
 *
 *  @param[in] new_value See enum ICM406XX_ACCEL_CONFIG0_FS_SEL_t.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_accel_config0_fs_sel(struct inv_icm406xx *           s,
                                         ICM406XX_ACCEL_CONFIG0_FS_SEL_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_ACCEL_CONFIG0_FS_SEL_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_ACCEL_CONFIG0, 1, &value);

	return status;
}

/** @brief Get ACCEL_CONFIG0 register ACCEL_FS_SEL bits
 *
 *  Accel Full Scale Select: <br>
 *  000 = ±16g <br>
 *  001 = ±8g <br>
 *  010 = ±4g <br>
 *  011 = ±2g <br>
 *  100 = ±1g <br>
 *
 *  @param[out] value See enum ICM406XX_ACCEL_CONFIG0_FS_SEL_t.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_accel_config0_fs_sel(struct inv_icm406xx *            s,
                                         ICM406XX_ACCEL_CONFIG0_FS_SEL_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_ACCEL_CONFIG0_FS_SEL_MASK;

	return status;
}

/** @brief Set ACCEL_CONFIG0 register ACCEL_ODR bits
 *
 *  Accel odr Select: <br>
 *  0 - 64K <br>
 *  1 - 32K <br>
 *  2 - 16K <br>
 *  3 - 8k <br>
 *  4 - 4k <br>
 *  5 - 2k <br>
 *  6 - 1k : 1ms <br>
 *  7 - 200 (default) : 5 ms <br>
 *  8 - 100 : 10 ms <br>
 *  9 - 50 : 20 ms <br>
 *  10 - 25 : 40 ms <br>
 *  11 - 12.5 : 80 ms <br>
 *  12 - 6.25 : 160 ms <br>
 *
 *  @param[in] new_value See enum ICM406XX_ACCEL_CONFIG0_ODR_t.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_accel_config0_odr(struct inv_icm406xx *        s,
                                      ICM406XX_ACCEL_CONFIG0_ODR_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_ACCEL_CONFIG0_ODR_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_ACCEL_CONFIG0, 1, &value);

	return status;
}

/** @brief Read ACCEL_CONFIG0 register ACCEL_ODR bits
 *
 *  Accel odr Select: <br>
 *  0 - 64K <br>
 *  1 - 32K <br>
 *  2 - 16K <br>
 *  3 - 8k <br>
 *  4 - 4k <br>
 *  5 - 2k <br>
 *  6 - 1k : 1ms <br>
 *  7 - 200 (default) : 5 ms <br>
 *  8 - 100 : 10 ms <br>
 *  9 - 50 : 20 ms <br>
 *  10 - 25 : 40 ms <br>
 *  11 - 12.5 : 80 ms <br>
 *  12 - 6.25 : 160 ms <br>
 *
 *  @param[out] value See enum ICM406XX_ACCEL_CONFIG0_ODR_t.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_accel_config0_odr(struct inv_icm406xx *s, ICM406XX_ACCEL_CONFIG0_ODR_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_ACCEL_CONFIG0_ODR_MASK;

	return status;
}

/** @brief Set GYRO_CONFIG1 register AVG_FILT_RATE bit
 *
 *  <pre>
 *  1 in LPM Average filter runs 8KHz
 *  0 in LPM Average filter runs 1KHz
 *  </pre>
 * @param[in] new_value See enum ICM406XX_GYRO_CONFIG1_AVG_FILT_RATE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_gyro_config1_avg_filt_rate(struct inv_icm406xx *                 s,
                                               ICM406XX_GYRO_CONFIG1_AVG_FILT_RATE_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_GYRO_CONFIG1, 1, &value);
	if (status)
		return status;

	value &= ~BIT_GYRO_CONFIG1_AVG_FILT_RATE_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_GYRO_CONFIG1, 1, &value);
	return status;
}

/** @brief Set GYRO_ACCEL_CONFIG0 register ACCEL_FILT bit
 *
 *  <pre>
 *  0 BW=ODR/2 FIR at 8*ODR
 *  1 BW=ODR/4 IIR at max(400Hz, ODR)
 *  2 BW=ODR/5 IIR at max(400Hz, ODR)
 *  3 BW=ODR/8 IIR at max(400Hz, ODR)
 *  4 BW=ODR/10 IIR at max(400Hz, ODR)
 *  5 BW=ODR/16 IIR at max(400Hz, ODR)
 *  6 BW=ODR/20 IIR at max(400Hz, ODR)
 *  7 BW=ODR/40 IIR at max(400Hz, ODR)
 *  </pre>
 * @param[in] new_value See enum ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_gyro_accel_config0_accel_filt_bw(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &value);

	return status;
}

/** @brief Get GYRO_ACCEL_CONFIG0 register ACCEL_FILT bit
 *
 *  <pre>
 *  0 BW=ODR/2 FIR at 8*ODR
 *  1 BW=ODR/4 IIR at max(400Hz, ODR)
 *  2 BW=ODR/5 IIR at max(400Hz, ODR)
 *  3 BW=ODR/8 IIR at max(400Hz, ODR)
 *  4 BW=ODR/10 IIR at max(400Hz, ODR)
 *  5 BW=ODR/16 IIR at max(400Hz, ODR)
 *  6 BW=ODR/20 IIR at max(400Hz, ODR)
 *  7 BW=ODR/40 IIR at max(400Hz, ODR)
 *  </pre>
 * @param[out] new_value See enum ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_gyro_accel_config0_accel_filt_bw(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK;

	return status;
}

/** @brief Set GYRO_ACCEL_CONFIG0 register ACCEL_FILT bit
 *
 *  <pre>
 *  1 AVG=1 AVG filt at 1kHz/8kHz
 *  2 AVG=2 AVG filt at 1kHz/8kHz
 *  3 AVG=3 AVG filt at 1kHz/8kHz
 *  4 AVG=4 AVG filt at 1kHz/8kHz
 *  5 AVG=8 AVG filt at 1kHz/8kHz
 *  6 AVG=16 AVG filt at 1kHz/8kHz
 *  7 AVG=32 AVG filt at 1kHz/8kHz
 *  8 AVG=64 AVG filt at 1kHz/8kHz
 *  9 AVG=128 AVG filt at 1kHz/8kHz
 *  </pre>
 * @param[in] new_value See enum ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_gyro_accel_config0_accel_filt_avg(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &value);

	return status;
}

/** @brief Get GYRO_ACCEL_CONFIG0 register ACCEL_FILT bit
 *
 *  <pre>
 *  1 AVG=1 AVG filt at 1kHz/8kHz
 *  2 AVG=2 AVG filt at 1kHz/8kHz
 *  3 AVG=3 AVG filt at 1kHz/8kHz
 *  4 AVG=4 AVG filt at 1kHz/8kHz
 *  5 AVG=8 AVG filt at 1kHz/8kHz
 *  6 AVG=16 AVG filt at 1kHz/8kHz
 *  7 AVG=32 AVG filt at 1kHz/8kHz
 *  8 AVG=64 AVG filt at 1kHz/8kHz
 *  9 AVG=128 AVG filt at 1kHz/8kHz
 *  </pre>
 * @param[out] new_value See enum ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_gyro_accel_config0_accel_filt_avg(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK;

	return status;
}

/** @brief Set GYRO_ACCEL_CONFIG0 register GYRO_FILT_BW bit
 *
 *  <pre>
 *  0 BW=ODR/2 FIR at 8*ODR
 *  1 BW=ODR/4 IIR at max(400Hz, ODR)
 *  2 BW=ODR/5 IIR at max(400Hz, ODR)
 *  3 BW=ODR/8 IIR at max(400Hz, ODR)
 *  4 BW=ODR/10 IIR at max(400Hz, ODR)
 *  5 BW=ODR/16 IIR at max(400Hz, ODR)
 *  6 BW=ODR/20 IIR at max(400Hz, ODR)
 *  7 BW=ODR/40 IIR at max(400Hz, ODR)
 *  </pre>
 * @param[in] new_value See enum ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_gyro_accel_config0_gyro_filt_bw(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &value);

	return status;
}

/** @brief Get GYRO_ACCEL_CONFIG0 register GYRO_FILT_BW bit
 *
 *  <pre>
 *  0 BW=ODR/2 FIR at 8*ODR
 *  1 BW=ODR/4 IIR at max(400Hz, ODR)
 *  2 BW=ODR/5 IIR at max(400Hz, ODR)
 *  3 BW=ODR/8 IIR at max(400Hz, ODR)
 *  4 BW=ODR/10 IIR at max(400Hz, ODR)
 *  5 BW=ODR/16 IIR at max(400Hz, ODR)
 *  6 BW=ODR/20 IIR at max(400Hz, ODR)
 *  7 BW=ODR/40 IIR at max(400Hz, ODR)
 *  </pre>
 * @param[out] new_value See enum ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_gyro_accel_config0_gyro_filt_bw(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_MASK;

	return status;
}

/** @brief Set GYRO_ACCEL_CONFIG0 register GYRO_FILT_AVG bit
 *
 *  <pre>
 *  1 AVG=1 AVG filt at 1kHz/8kHz
 *  2 AVG=2 AVG filt at 1kHz/8kHz
 *  3 AVG=3 AVG filt at 1kHz/8kHz
 *  4 AVG=4 AVG filt at 1kHz/8kHz
 *  5 AVG=8 AVG filt at 1kHz/8kHz
 *  6 AVG=16 AVG filt at 1kHz/8kHz
 *  7 AVG=32 AVG filt at 1kHz/8kHz
 *  8 AVG=64 AVG filt at 1kHz/8kHz
 *  9 AVG=128 AVG filt at 1kHz/8kHz
 *  </pre>
 * @param[in] new_value See enum ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_AVG_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_gyro_accel_config0_gyro_filt_avg(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_AVG_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &value);
	if (status)
		return status;

	value &= ~BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &value);
	return status;
}

/** @brief Set ACCEL_CONFIG1 register AVG_FILT_RATE bit
 *
 *  <pre>
 *  1 in LPM Average filter runs 8KHz
 *  0 in LPM Average filter runs 1KHz
 *  </pre>
 * @param[in] new_value See enum ICM406XX_ACCEL_CONFIG1_AVG_FILT_RATE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_accel_config1_avg_filt_rate(struct inv_icm406xx *                  s,
                                                ICM406XX_ACCEL_CONFIG1_AVG_FILT_RATE_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_ACCEL_CONFIG1, 1, &value);
	if (status)
		return status;

	value &= ~BIT_ACCEL_CONFIG1_AVG_FILT_RATE_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_ACCEL_CONFIG1, 1, &value);
	return status;
}

/** @brief Get ACCEL_CONFIG1 register AVG_FILT_RATE bit
 *
 *  <pre>
 *  1 in LPM Average filter runs 8KHz
 *  0 in LPM Average filter runs 1KHz
 *  </pre>
 * @param[out] new_value See enum ICM406XX_ACCEL_CONFIG1_AVG_FILT_RATE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_accel_config1_avg_filt_rate(struct inv_icm406xx *                   s,
                                                ICM406XX_ACCEL_CONFIG1_AVG_FILT_RATE_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_ACCEL_CONFIG1, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_ACCEL_CONFIG1_AVG_FILT_RATE_MASK;

	return status;
}

/** @brief Set ACCEL_WOM_X_THR register WOM_X_TH bits
 *
 *  This register holds the threshold value for the Wake on Motion Interrupt for X-axis accelerometer.
 *
 *  @param[in] new_value accel wake on motion x-axis threshold
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_accel_wom_x_thr(struct inv_icm406xx *s, uint8_t new_value)
{
#ifdef ICM40609D
	uint8_t bank;
	int status;

	// Set memory bank 4
	bank   = 4;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_write_reg(s, MPUREG_ACCEL_WOM_X_THR, 1, &new_value);

	// Set memory bank 0
	bank = 0;
	status |= inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	return status;
#else
	return inv_icm406xx_write_reg(s, MPUREG_ACCEL_WOM_X_THR, 1, &new_value);
#endif
}

/** @brief Set ACCEL_WOM_Y_THR register WOM_Y_TH bits
 *
 *  This register holds the threshold value for the Wake on Motion Interrupt for Y-axis accelerometer.
 *
 *  @param[in] new_value accel wake on motion y-axis threshold
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_accel_wom_y_thr(struct inv_icm406xx *s, uint8_t new_value)
{
#ifdef ICM40609D
	uint8_t bank;
	int status;

	// Set memory bank 4
	bank   = 4;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_write_reg(s, MPUREG_ACCEL_WOM_Y_THR, 1, &new_value);

	// Set memory bank 0
	bank = 0;
	status |= inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	return status;
#else
	return inv_icm406xx_write_reg(s, MPUREG_ACCEL_WOM_Y_THR, 1, &new_value);
#endif
}

/** @brief Set ACCEL_WOM_Z_THR register WOM_Z_TH bits
 *
 *  This register holds the threshold value for the Wake on Motion Interrupt for Z-axis accelerometer.
 *
 *  @param[in] new_value accel wake on motion z-axis threshold
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_accel_wom_z_thr(struct inv_icm406xx *s, uint8_t new_value)
{
#ifdef ICM40609D
	uint8_t bank;
	int status;

	// Set memory bank 4
	bank   = 4;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_write_reg(s, MPUREG_ACCEL_WOM_Z_THR, 1, &new_value);

	// Set memory bank 0
	bank = 0;
	status |= inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	return status;
#else
	return inv_icm406xx_write_reg(s, MPUREG_ACCEL_WOM_Z_THR, 1, &new_value);
#endif
}

/** @brief Set MPUREG_SMD_CONFIG register WOM_INT_MODE bit
 *
 *  <pre>
 *  1 – The WoM from 3 axis are ANDed to produce WoM signal
 *  0 – The WoM from 3 axis are ORed to produce WoM signal
 *  </pre>
 * @param[in] new_value See enum ICM406XX_SMD_CONFIG_WOM_INT_MODE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_smd_config_wom_int_mode(struct inv_icm406xx *              s,
                                            ICM406XX_SMD_CONFIG_WOM_INT_MODE_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_SMD_CONFIG, 1, &value);
	if (status)
		return status;

	value &= ~BIT_SMD_CONFIG_WOM_INT_MODE_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_SMD_CONFIG, 1, &value);
	return status;
}

/** @brief Get MPUREG_SMD_CONFIG register WOM_INT_MODE bit
 *
 *  <pre>
 *  1 – The WoM from 3 axis are ANDed to produce WoM signal
 *  0 – The WoM from 3 axis are ORed to produce WoM signal
 *  </pre>
 * @param[in] new_value See enum ICM406XX_SMD_CONFIG_WOM_INT_MODE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_smd_config_wom_int_mode(struct inv_icm406xx *               s,
                                            ICM406XX_SMD_CONFIG_WOM_INT_MODE_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_SMD_CONFIG, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_SMD_CONFIG_WOM_INT_MODE_MASK;

	return status;
}

/** @brief Set MPUREG_SMD_CONFIG register WOM_MODE bit
 *
 *  <pre>
 *  1 – Compare current sample to previous sample
 *  0 – Initial sample is stored. Future samples are compared to initial 
 *  </pre>
 * @param[in] new_value See enum ICM406XX_SMD_CONFIG_WOM_MODE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_smd_config_wom_mode(struct inv_icm406xx *          s,
                                        ICM406XX_SMD_CONFIG_WOM_MODE_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_SMD_CONFIG, 1, &value);
	if (status)
		return status;

	value &= ~BIT_SMD_CONFIG_WOM_MODE_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_SMD_CONFIG, 1, &value);
	return status;
}

/** @brief Get MPUREG_SMD_CONFIG register WOM_MODE bit
 *
 *  <pre>
 *  1 – Compare current sample to previous sample
 *  0 – Initial sample is stored. Future samples are compared to initial 
 *  </pre>
 * @param[in] new_value See enum ICM406XX_SMD_CONFIG_WOM_MODE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_smd_config_wom_mode(struct inv_icm406xx *           s,
                                        ICM406XX_SMD_CONFIG_WOM_MODE_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_SMD_CONFIG, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_SMD_CONFIG_WOM_MODE_MASK;

	return status;
}

/** @brief Set MPUREG_SMD_CONFIG register SMD_MODE bit
 *
 *  <pre>
 *  3 - SMD long (3 sec wait)
 *  2 - SMD short (1 sec wait)
 *  1 – WOM using old definition
 *  0 – Disabled
 *  </pre>
 * @param[in] new_value See enum ICM406XX_SMD_CONFIG_SMD_MODE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_smd_config_smd_mode(struct inv_icm406xx *          s,
                                        ICM406XX_SMD_CONFIG_SMD_MODE_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_SMD_CONFIG, 1, &value);
	if (status)
		return status;

	value &= ~BIT_SMD_CONFIG_SMD_MODE_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_SMD_CONFIG, 1, &value);
	return status;
}

/** @brief Get MPUREG_SMD_CONFIG register SMD_MODE bit
 *
 *  <pre>
 *  3 - SMD long (3 sec wait)
 *  2 - SMD short (1 sec wait)
 *  1 – WOM using old definition
 *  0 – Disabled
 *  </pre>
 * @param[in] new_value See enum ICM406XX_SMD_CONFIG_SMD_MODE_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_smd_config_smd_mode(struct inv_icm406xx *           s,
                                        ICM406XX_SMD_CONFIG_SMD_MODE_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_SMD_CONFIG, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_SMD_CONFIG_SMD_MODE_MASK;

	return status;
}

/** @brief Get INT_STATUS2 register value
 *
 *  @param[out] * value INT_STATUS2 register value
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_int_status2(struct inv_icm406xx *s, uint8_t *value)
{
	return inv_icm406xx_read_reg(s, MPUREG_INT_STATUS2, 1, value);
}

/** @brief Set TMST_CONFIG register TMST_TO_REGS_EN bit
 *
 *  When set to ‘1’, Timestamp to register enable.
 *  When set to ‘0’, Timestamp to register disable
 *
 *  @param[in] new_value See enum ICM406XX_TMST_CONFIG_TMST_TO_REGS_EN_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_tmst_config_tmst_to_reg(struct inv_icm406xx *                  s,
                                            ICM406XX_TMST_CONFIG_TMST_TO_REGS_EN_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_TMST_CONFIG, 1, &value);
	if (status)
		return status;

	value &= ~BIT_TMST_CONFIG_TMST_TO_REGS_EN_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_TMST_CONFIG, 1, &value);

	return status;
}

/** @brief Set TMST_CONFIG register TMST_RES bit
 *
 *  When set to ‘1’, Timestamp 16us resolution.
 *  When set to ‘0’, Timestamp 1us resolution
 *
 *  @param[in] new_value See enum ICM406XX_TMST_CONFIG_RESOL_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_tmst_config_resolution(struct inv_icm406xx *        s,
                                           ICM406XX_TMST_CONFIG_RESOL_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_TMST_CONFIG, 1, &value);
	if (status)
		return status;

	value &= ~BIT_TMST_CONFIG_RESOL_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_TMST_CONFIG, 1, &value);

	return status;
}

/** @brief Set TMST_CONFIG register TMST_FSYNC bit
 *
 *  When set to ‘1’, Timestamp fsync enable.
 *  When set to ‘0’, Timestamp fsync disable
 *
 *  @param[in] new_value See enum ICM406XX_TMST_CONFIG_TMST_FSYNC_EN_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_tmst_config_fsync_en(struct inv_icm406xx *                s,
                                         ICM406XX_TMST_CONFIG_TMST_FSYNC_EN_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_TMST_CONFIG, 1, &value);
	if (status)
		return status;

	value &= ~BIT_TMST_CONFIG_TMST_FSYNC_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_TMST_CONFIG, 1, &value);

	return status;
}

/** @brief Read TMST_CONFIG register TMST_FSYNC bit
 *
 *  @param[out] value pointer on read value (See enum ICM406XX_TMST_CONFIG_TMST_FSYNC_EN_t)
 *                    ‘1’, Timestamp fsync enable.
 *                    ‘0’, Timestamp fsync disable
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_tmst_config_fsync_en(struct inv_icm406xx *                 s,
                                         ICM406XX_TMST_CONFIG_TMST_FSYNC_EN_t *value)
{
	int status;

	status = inv_icm406xx_read_reg(s, MPUREG_TMST_CONFIG, 1, (uint8_t *)value);
	if (status)
		return status;

	*value &= BIT_TMST_CONFIG_TMST_FSYNC_MASK;

	return status;
}

/** @brief Set TMST_CONFIG register ICM406XX_TMST_CONFIG_EN bit
 *
 *  When set to ‘1’, Timestamp register enable.
 *  When set to ‘0’, Timestamp register disable
 *
 *  @param[in] new_value See enum ICM406XX_TMST_CONFIG_TMST_EN_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_tmst_config_en(struct inv_icm406xx *s, ICM406XX_TMST_CONFIG_TMST_EN_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_TMST_CONFIG, 1, &value);
	if (status)
		return status;

	value &= ~BIT_TMST_CONFIG_TMST_EN_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_TMST_CONFIG, 1, &value);

	return status;
}

/** @brief Set FIFO_CONFIG1 register FIFO_WM_GT_TH bit
 *
 *  When set to ‘1’ trigger FIFO-Watermark interrupt on every ODR(DMA Write)  if FIFO_COUNT>=FIFO_WM_TH . <br>
 *  When set to ‘0’ disable .
 *
 *  @param[in] new_value See enum ICM406XX_FIFO_CONFIG1_WM_GT_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_fifo_config1_wm_gt_th(struct inv_icm406xx *         s,
                                          ICM406XX_FIFO_CONFIG1_WM_GT_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &value);
	if (status)
		return status;

	value &= ~BIT_FIFO_CONFIG1_WM_GT_TH_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &value);

	return status;
}

/** @brief Set FIFO_CONFIG1 register FIFO_HIRES_EN bit
 *
 *  When set to ‘1’ enable hires. <br>
 *  When set to ‘0’ disable .
 *
 *  @param[in] new_value See enum ICM406XX_FIFO_CONFIG1_HIRES_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_fifo_config1_hires(struct inv_icm406xx *         s,
                                       ICM406XX_FIFO_CONFIG1_HIRES_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &value);
	if (status)
		return status;

	value &= ~BIT_FIFO_CONFIG1_HIRES_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &value);

	return status;
}

/** @brief Set FIFO_CONFIG1 register FIFO_TMST_FSYNC_EN bit
 *
 *  When set to ‘1’ Allows the TMST in the FIFO to be replaced by the FSYNC timestamp. <br>
 *  When set to ‘0’ disable .
 *
 *  @param[in] new_value See enum ICM406XX_FIFO_CONFIG1_TMST_FSYNC_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_fifo_config1_tmst_fsync(struct inv_icm406xx *              s,
                                            ICM406XX_FIFO_CONFIG1_TMST_FSYNC_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &value);
	if (status)
		return status;

	value &= ~BIT_FIFO_CONFIG1_TMST_FSYNC_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &value);

	return status;
}

/** @brief Set FIFO_CONFIG1 register FIFO_TEMP_EN bit
 *
 *  <pre>
 *  1 – write TEMP_OUT_H, TEMP_OUT_L, to the FIFO at the sample rate; If enabled, buffering
 *      of data occurs even if data path is in standby.
 *  0 – function is disabled
 *  </pre>
 *  
 *  Note: Can not be enabled alone, request ACCEL or GYRO 
 *
 *  @param[in] new_value See enum ICM406XX_FIFO_CONFIG1_TEMP_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_fifo_config1_temp_en(struct inv_icm406xx *        s,
                                         ICM406XX_FIFO_CONFIG1_TEMP_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &value);
	if (status)
		return status;

	value &= ~BIT_FIFO_CONFIG1_TEMP_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &value);

	return status;
}

/** @brief Set FIFO_CONFIG1 register FIFO_GYRO_EN bit
 *
 *  <pre>
 *  1 – write GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L,
 *      GYRO_ZOUT_H, and GYRO_ZOUT_L to the FIFO at the sample rate; If enabled, buffering
 *      of data occurs even if data path is in standby.
 *  0 – function is disabled
 *  </pre>
 *
 *  @param[in] new_value See enum ICM406XX_FIFO_CONFIG1_GYRO_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_fifo_config1_gyro_en(struct inv_icm406xx *        s,
                                         ICM406XX_FIFO_CONFIG1_GYRO_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &value);
	if (status)
		return status;

	value &= ~BIT_FIFO_CONFIG1_GYRO_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &value);

	return status;
}

/** @brief Set FIFO_CONFIG1 register FIFO_ACCEL_EN bit
 *
 *  <pre>
 *  1 – write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L,
 *      to the FIFO at the sample rate;
 *  0 – function is disabled
 *  </pre>
 *
 *  @param[in] new_value See enum ICM406XX_FIFO_CONFIG1_ACCEL_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_fifo_config1_accel_en(struct inv_icm406xx *         s,
                                          ICM406XX_FIFO_CONFIG1_ACCEL_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &value);
	if (status)
		return status;

	value &= ~BIT_FIFO_CONFIG1_ACCEL_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &value);

	return status;
}
/** @brief Set FIFO_CONFIG2 register value
 *
 *  @param[in] 8 LSB of FIFO Watermark value
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_fifo_config2(struct inv_icm406xx *s, uint8_t new_value)
{
	return inv_icm406xx_write_reg(s, MPUREG_FIFO_CONFIG2, 1, &new_value);
}

/** @brief Set FSYNC_CONFIG register FSYNC_UI_SEL bit
 *
 *  <pre>
 *  0 Do not tag Fsync flag
 *  1 Tag Fsync flag to TEMP_OUT’s LSB
 *  2 Tag Fsync flag to GYRO_XOUT’s LSB
 *  3 Tag Fsync flag to GYRO_YOUT’s LSB
 *  4 Tag Fsync flag to GYRO_ZOUT’s LSB
 *  5 Tag Fsync flag to ACCEL_XOUT’s LSB
 *  6 Tag Fsync flag to ACCEL_YOUT’s LSB
 *  7 Tag Fsync flag to ACCEL_ZOUT’s LSB
 *  </pre>
 * @param[in] new_value See enum ICM406XX_FSYNC_CONFIG_UI_SEL_t
 * @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_fsync_config_ui_sel(struct inv_icm406xx *          s,
                                     ICM406XX_FSYNC_CONFIG_UI_SEL_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_FSYNC_CONFIG, 1, &value);
	if (status)
		return status;

	value &= ~BIT_FSYNC_CONFIG_UI_SEL_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_FSYNC_CONFIG, 1, &value);
	return status;
}

/** @brief Set INT_CONFIG1 register ASY_RESET_DISABLE bit
 *
 *  @param[in] This bit is not usable in Havana A1. 
 *  Please set value of 1 always. Further details are described in use notes. 
 *  (Intended operation for future chips: Removes the asynchronous clear of the 
 *  interrupt output when disabling or cleaning all the active interrupt sources, 
 *  action which may causes a cut on the interrupt pulse length in pulsed mode.)
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_int_config1_asy_rst_dis(struct inv_icm406xx *          s,
                                            ICM406XX_INT_CONFIG1_ASY_RST_t new_value)
{
	uint8_t value;
	int     status;

	status = inv_icm406xx_read_reg(s, MPUREG_INT_CONFIG1, 1, &value);
	if (status)
		return status;

	value &= ~BIT_INT_CONFIG1_ASY_RST_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_INT_CONFIG1, 1, &value);

	return status;
}

/** @brief Set INT_SOURCE0 register value
 *
 *  @param[in] Interrupt source
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_int_source0(struct inv_icm406xx *s, uint8_t new_value)
{
	return inv_icm406xx_write_reg(s, MPUREG_INT_SOURCE0, 1, &new_value);
}

/** @brief Get INT_SOURCE0 register value
 *
 *  @param[out] Interrupt source
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_int_source0(struct inv_icm406xx *s, uint8_t *value)
{
	return inv_icm406xx_read_reg(s, MPUREG_INT_SOURCE0, 1, value);
}

/** @brief Set INT_SOURCE1 register value
 *
 *  @param[in] Interrupt source
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_int_source1(struct inv_icm406xx *s, uint8_t new_value)
{
	return inv_icm406xx_write_reg(s, MPUREG_INT_SOURCE1, 1, &new_value);
}

/** @brief Get INT_SOURCE1 register value
 *
 *  @param[out] Interrupt source
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_int_source1(struct inv_icm406xx *s, uint8_t *value)
{
	return inv_icm406xx_read_reg(s, MPUREG_INT_SOURCE1, 1, value);
}

/** @brief Get WHO_AM_I register value
 *
 *  Register to indicate to user which device is being accessed.
 *
 *  @param[out] value The whoami of the device.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_who_am_i(struct inv_icm406xx *s, uint8_t *value)
{
	return inv_icm406xx_read_reg(s, MPUREG_WHO_AM_I, 1, value);
}

/** @brief Set REG_BANK_SEL register value
 *
 *  Register to select bank of registers to be accessed with subsequent register access.
 *
 *  @param[in] value Bank to be selected.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_reg_bank_sel(struct inv_icm406xx *s, uint8_t new_value)
{
	return inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &new_value);
}

/** @brief Get TMST_VAL from TMST_VAL0_B1, TMST_VAL1_B1 and TMST_VAL2_B1 registers
 *
 *  @param[out] value 20-bit timestamp register.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_tmst_val(struct inv_icm406xx *s, uint8_t value[3])
{
	uint8_t bank;
	int     status;
	bank   = 1;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);
	status |= inv_icm406xx_read_reg(s, MPUREG_TMST_VAL0_B1, 3, value);
	bank = 0;
	status |= inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);
	return status;
}

/** @brief Set INTF_CONFIG4_B1 register SPI_AUX1_4WIRE bit
 *
 *  When set to ‘1’, SPI AUX is in 4 wire interface 
 *  When set to ‘0’, SPI AUX is in 3 wire interface 
 *
 *  @param[in] new_value See enum ICM406XX_INTF_CONFIG4_AUX_SPI_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_intf_config4_aux_spi(struct inv_icm406xx *           s,
                                         ICM406XX_INTF_CONFIG4_AUX_SPI_t new_value)
{
	uint8_t value, bank;
	int     status;

	// Set memory bank 1
	bank   = 1;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_read_reg(s, MPUREG_INTF_CONFIG4_B1, 1, &value);

	value &= ~BIT_INTF_CONFIG4_AUX_SPI_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_INTF_CONFIG4_B1, 1, &value);

	// Set memory bank 0
	bank = 0;
	status |= inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	return status;
}

/** @brief Get INTF_CONFIG4_B1 register SPI_AUX1_4WIRE bit
 *
 *  When set to ‘1’, SPI AUX is in 4 wire interface 
 *  When set to ‘0’, SPI AUX is in 3 wire interface 
 *
 *  @param[out] * value See enum ICM406XX_INTF_CONFIG4_AUX_SPI_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_intf_config4_aux_spi(struct inv_icm406xx *            s,
                                         ICM406XX_INTF_CONFIG4_AUX_SPI_t *value)
{
	int     status;
	uint8_t bank;

	// Set memory bank 1
	bank   = 1;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_read_reg(s, MPUREG_INTF_CONFIG4_B1, 1, (uint8_t *)value);

	// Set memory bank 0
	bank = 0;
	status |= inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	*value &= BIT_INTF_CONFIG4_AUX_SPI_MASK;

	return status;
}

/** @brief Set INTF_CONFIG4_B1 register SPI_AP_4WIRE bit
 *
 *  When set to ‘1’, SPI AP is in 4 wire interface 
 *  When set to ‘0’, SPI AP is in 3 wire interface 
 *
 *  @param[in] new_value See enum ICM406XX_INTF_CONFIG4_AP_SPI_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_intf_config4_ap_spi(struct inv_icm406xx *          s,
                                        ICM406XX_INTF_CONFIG4_AP_SPI_t new_value)
{
	uint8_t value, bank;
	int     status;

	// Set memory bank 1
	bank   = 1;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_read_reg(s, MPUREG_INTF_CONFIG4_B1, 1, &value);

	value &= ~BIT_INTF_CONFIG4_AP_SPI_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_INTF_CONFIG4_B1, 1, &value);

	// Set memory bank 0
	bank = 0;
	status |= inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	return status;
}

/** @brief Get INTF_CONFIG4_B1 register SPI_AP_4WIRE bit
 *
 *  When set to ‘1’, SPI AP is in 4 wire interface 
 *  When set to ‘0’, SPI AP is in 3 wire interface 
 *
 *  @param[in] * value See enum ICM406XX_INTF_CONFIG4_AP_SPI_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_intf_config4_ap_spi(struct inv_icm406xx *           s,
                                        ICM406XX_INTF_CONFIG4_AP_SPI_t *value)
{
	int     status;
	uint8_t bank;

	// Set memory bank 1
	bank   = 1;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_read_reg(s, MPUREG_INTF_CONFIG4_B1, 1, (uint8_t *)value);

	// Set memory bank 0
	bank = 0;
	status |= inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	*value &= BIT_INTF_CONFIG4_AP_SPI_MASK;

	return status;
}

/** @brief Set AUX1_CONFIG1 register AUX1_DEC bit
 *
 * Decimation factor for auxiliary1 signal path  (common to Gyro and Accel)
 * 0    1
 * 1    2
 * 2    4
 * 3    8
 * 4   16
 * 5   32
 * 6   N/A
 * 7   N/A
 * 
 *  @param[in] new_value See enum ICM406XX_OIS1_CONFIG1_DEC_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_ois1_config1_dec(struct inv_icm406xx *s, ICM406XX_OIS1_CONFIG1_DEC_t new_value)
{
	uint8_t value, bank;
	int     status;

	// Set memory bank 2
	bank   = 2;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_read_reg(s, MPUREG_AUX1_CONFIG1_B2, 1, &value);

	value &= ~BIT_OIS1_CONFIG1_DEC_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_AUX1_CONFIG1_B2, 1, &value);

	return status;
}

/** @brief Set AUX1_CONFIG1 register GYRO_AUX1_EN bit
 *
 * 0 - Disable AUX 1 Path
 * 1 - Enable AUX1 Path
 * 
 *  @param[in] new_value See enum ICM406XX_OIS1_CONFIG1_EN_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_ois1_config1_gyro_en(struct inv_icm406xx *           s,
                                         ICM406XX_OIS1_CONFIG1_GYRO_EN_t new_value)
{
	uint8_t value, bank;
	int     status;

	// Set memory bank 2
	bank   = 2;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_read_reg(s, MPUREG_AUX1_CONFIG1_B2, 1, &value);

	value &= ~BIT_OIS1_CONFIG1_GYRO_EN_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_AUX1_CONFIG1_B2, 1, &value);

	return status;
}

/** @brief Set AUX1_CONFIG2 register GYRO_AUX1_FS_SEL bit
 *
 *  Gyro OIS Full Scale Select: <br>
 *  000 = ±2000dps <br>
 *  001 = ±1000dps <br>
 *  010 = ±500dps <br>
 *  011 = ±250dps <br>
 *  100 = ±125dps <br>
 *  101 = ±62dps <br>
 *  110 = ±31dps <br>
 *  111 = ±16dps <br>
 *
 *  @param[in] new_value See enum ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_ois1_config2_gyro_fs_sel(struct inv_icm406xx *               s,
                                             ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_t new_value)
{
	uint8_t value, bank;
	int     status;

	// Set memory bank 2
	bank   = 2;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_read_reg(s, MPUREG_AUX1_CONFIG2_B2, 1, &value);

	value &= ~BIT_OIS1_CONFIG2_GYRO_FS_SEL_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_AUX1_CONFIG2_B2, 1, &value);

	return status;
}

/** @brief Get AUX1_CONFIG2 register GYRO_AUX1_FS_SEL bit
 *
 *  Gyro OIS Full Scale Select: <br>
 *  000 = ±2000dps <br>
 *  001 = ±1000dps <br>
 *  010 = ±500dps <br>
 *  011 = ±250dps <br>
 *  100 = ±125dps <br>
 *  101 = ±62dps <br>
 *  110 = ±31dps <br>
 *  111 = ±16dps <br>
 *
 *  @param[in] new_value See enum ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_ois1_config2_gyro_fs_sel(struct inv_icm406xx *                s,
                                             ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_t *value)
{
	int     status;
	uint8_t bank;

	// Set memory bank 2
	bank   = 2;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_read_reg(s, MPUREG_AUX1_CONFIG2_B2, 1, (uint8_t *)value);

	*value &= BIT_OIS1_CONFIG2_GYRO_FS_SEL_MASK;

	return status;
}

/** @brief Get MPUREG_GYRO_DATA_X0_AUX1_B2 
 *
 *  @param[out] value 3*2 OIS bytes.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_gyro_data_ois1(struct inv_icm406xx *s, uint8_t value[GYRO_DATA_SIZE])
{
	int     status;
	uint8_t bank;

	// Set memory bank 2
	bank   = 2;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |=
	    inv_icm406xx_read_reg(s, MPUREG_GYRO_DATA_X0_AUX1_B2, GYRO_DATA_SIZE, (uint8_t *)value);

	return status;
}

/** @brief Get INT_STATUS_AUX1 register value
 *
 *  @param[out] * value INT_STATUS_AUX1 register value
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_int_status_aux1(struct inv_icm406xx *s, uint8_t *value)
{
	int     status;
	uint8_t bank;

	// Set memory bank 2
	bank   = 2;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_read_reg(s, MPUREG_INT_STATUS_AUX1_B2, 1, value);

	return status;
}

/** @brief Set AUX2_CONFIG1 register AUX2_DEC bit
 *
 * Decimation factor for auxiliary2 signal path  (common to Gyro and Accel)
 * 0    1
 * 1    2
 * 2    4
 * 3    8
 * 4   16
 * 5   32
 * 6   N/A
 * 7   N/A
 * 
 *  @param[in] new_value See enum ICM406XX_OIS2_CONFIG1_DEC_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_ois2_config1_dec(struct inv_icm406xx *s, ICM406XX_OIS2_CONFIG1_DEC_t new_value)
{
	uint8_t value, bank;
	int     status;

	// Set memory bank 2
	bank   = 2;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_read_reg(s, MPUREG_AUX2_CONFIG1_B2, 1, &value);

	value &= ~BIT_OIS2_CONFIG1_DEC_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_AUX2_CONFIG1_B2, 1, &value);

	return status;
}

/** @brief Set AUX2_CONFIG1 register GYRO_AUX2_EN bit
 *
 * 0 - Disable AUX2 Path
 * 1 - Enable AUX2 Path
 * 
 *  @param[in] new_value See enum ICM406XX_OIS2_CONFIG1_EN_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_ois2_config1_gyro_en(struct inv_icm406xx *           s,
                                         ICM406XX_OIS2_CONFIG1_GYRO_EN_t new_value)
{
	uint8_t value, bank;
	int     status;

	// Set memory bank 2
	bank   = 2;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_read_reg(s, MPUREG_AUX2_CONFIG1_B2, 1, &value);

	value &= ~BIT_OIS2_CONFIG1_GYRO_EN_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_AUX2_CONFIG1_B2, 1, &value);

	return status;
}

/** @brief Set AUX2_CONFIG2 register GYRO_AUX2_FS_SEL bit
 *
 *  Gyro OIS Full Scale Select: <br>
 *  000 = ±2000dps <br>
 *  001 = ±1000dps <br>
 *  010 = ±500dps <br>
 *  011 = ±250dps <br>
 *  100 = ±125dps <br>
 *  101 = ±62dps <br>
 *  110 = ±31dps <br>
 *  111 = ±16dps <br>
 *
 *  @param[in] new_value See enum ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_wr_ois2_config2_gyro_fs_sel(struct inv_icm406xx *               s,
                                             ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_t new_value)
{
	uint8_t value, bank;
	int     status;

	// Set memory bank 2
	bank   = 2;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_read_reg(s, MPUREG_AUX2_CONFIG2_B2, 1, &value);

	value &= ~BIT_OIS2_CONFIG2_GYRO_FS_SEL_MASK;
	value |= new_value;

	status = inv_icm406xx_write_reg(s, MPUREG_AUX2_CONFIG2_B2, 1, &value);

	return status;
}

/** @brief Get AUX2_CONFIG2 register GYRO_AUX2_FS_SEL bit
 *
 *  Gyro OIS Full Scale Select: <br>
 *  000 = ±2000dps <br>
 *  001 = ±1000dps <br>
 *  010 = ±500dps <br>
 *  011 = ±250dps <br>
 *  100 = ±125dps <br>
 *  101 = ±62dps <br>
 *  110 = ±31dps <br>
 *  111 = ±16dps <br>
 *
 *  @param[in] new_value See enum ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_t
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_ois2_config2_gyro_fs_sel(struct inv_icm406xx *                s,
                                             ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_t *value)
{
	int     status;
	uint8_t bank;

	// Set memory bank 2
	bank   = 2;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |= inv_icm406xx_read_reg(s, MPUREG_AUX2_CONFIG2_B2, 1, (uint8_t *)value);

	*value &= BIT_OIS2_CONFIG2_GYRO_FS_SEL_MASK;

	return status;
}

/** @brief Get MPUREG_GYRO_DATA_X0_AUX2_B2 
 *
 *  @param[out] value 3*2 OIS bytes.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_icm406xx_rd_gyro_data_ois2(struct inv_icm406xx *s, uint8_t value[GYRO_DATA_SIZE])
{
	int     status;
	uint8_t bank;

	// Set memory bank 2
	bank   = 2;
	status = inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	status |=
	    inv_icm406xx_read_reg(s, MPUREG_GYRO_DATA_X0_AUX2_B2, GYRO_DATA_SIZE, (uint8_t *)value);

	return status;
}

/** @} */
