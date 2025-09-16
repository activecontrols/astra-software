/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
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

#include "Icm406xxSelfTest.h"
#include "Icm406xxDefs.h"
#include "Icm406xxExtFunc.h"
#include "Icm406xxTransport.h"
#include "Icm406xxDriver_HL.h"

#ifndef INV_ABS
#define INV_ABS(x) (((x) < 0) ? -(x) : (x))
#endif

/** default values to test for self-test results */
#define DEFAULT_ST_GYRO_FSR_DPS 250
#define DEFAULT_ST_ACCEL_FSR_G  2

#define MIN_DEFAULT_ST_GYRO_DPS        60 /* expected values higher than 60 dps */
#define MAX_DEFAULT_ST_GYRO_OFFSET_DPS 20 /* expected values smaller than 20 dps */

#define MIN_DEFAULT_ST_ACCEL_MG 225 /* expected values between 225 mgee and 675 mgee */
#define MAX_DEFAULT_ST_ACCEL_MG 675

/** register configuration for self-test procedure */

#define DEFAULT_ST_GYRO_UI_FILT_ORD_3     2 << 2
#define DEFAULT_ST_GYRO_UI_FILT_BW_ODR_10 4
#define DEFAULT_ST_GYRO_ODR_1_KHZ         6
#define DEFAULT_ST_GYRO_FSR_250_DPS       3 << 5

#define DEFAULT_ST_ACCEL_UI_FILT_ORD_3     2 << 3
#define DEFAULT_ST_ACCEL_UI_FILT_BW_ODR_10 4 << 4
#define DEFAULT_ST_ACCEL_ODR_1_KHZ         6
#if ICM40609D
	#define DEFAULT_ST_ACCEL_FSR_2_G           4 << 5
#else
	#define DEFAULT_ST_ACCEL_FSR_2_G           3 << 5
#endif

/** @brief Icm406xx HW Base sensor status based upon s->sensor_on_mask
 */
enum inv_icm406xx_sensor_on_mask {
	INV_ICM406XX_SENSOR_ON_MASK_ACCEL = (1L << INV_ICM406XX_SENSOR_ACCEL),
	INV_ICM406XX_SENSOR_ON_MASK_GYRO  = (1L << INV_ICM406XX_SENSOR_GYRO),
	INV_ICM406XX_SENSOR_ON_MASK_EIS   = (1L << INV_ICM406XX_SENSOR_FSYNC_EVENT),
	INV_ICM406XX_SENSOR_ON_MASK_OIS   = (1L << INV_ICM406XX_SENSOR_OIS),
	INV_ICM406XX_SENSOR_ON_MASK_TEMP  = (1L << INV_ICM406XX_SENSOR_TEMPERATURE),
};

/** @brief Icm406xx Sensor power modes
 */
enum inv_icm406xx_sensor_pwr_mode {
	INV_ICM406XX_SENSOR_PWR_MODE_LPM,
	INV_ICM406XX_SENSOR_PWR_MODE_LNM,
	INV_ICM406XX_SENSOR_PWR_MODE_MAX
};

struct recover_regs {
	/** bank 0 */
	uint8_t pwr_mgmt_0; /* REG_PWR_MGMT_0			*/
	uint8_t accel_config0; /* REG_ACCEL_CONFIG0		*/
	uint8_t accel_config1; /* REG_ACCEL_CONFIG1		*/
	uint8_t gyro_config0; /* REG_GYRO_CONFIG0			*/
	uint8_t gyro_config1; /* REG_GYRO_CONFIG1 		*/
	uint8_t accel_gyro_config0; /* REG_ACCEL_GYRO_CONFIG0 	*/
	/** bank 1 */
	uint8_t self_test_config; /* REG_SELF_TEST_CONFIG 	*/
};

static const uint16_t sSelfTestEquation[256] = {
	2620,  2646,  2672,  2699,  2726,  2753,  2781,  2808,  2837,  2865,  2894,  2923,  2952,
	2981,  3011,  3041,  3072,  3102,  3133,  3165,  3196,  3228,  3261,  3293,  3326,  3359,
	3393,  3427,  3461,  3496,  3531,  3566,  3602,  3638,  3674,  3711,  3748,  3786,  3823,
	3862,  3900,  3939,  3979,  4019,  4059,  4099,  4140,  4182,  4224,  4266,  4308,  4352,
	4395,  4439,  4483,  4528,  4574,  4619,  4665,  4712,  4759,  4807,  4855,  4903,  4953,
	5002,  5052,  5103,  5154,  5205,  5257,  5310,  5363,  5417,  5471,  5525,  5581,  5636,
	5693,  5750,  5807,  5865,  5924,  5983,  6043,  6104,  6165,  6226,  6289,  6351,  6415,
	6479,  6544,  6609,  6675,  6742,  6810,  6878,  6946,  7016,  7086,  7157,  7229,  7301,
	7374,  7448,  7522,  7597,  7673,  7750,  7828,  7906,  7985,  8065,  8145,  8227,  8309,
	8392,  8476,  8561,  8647,  8733,  8820,  8909,  8998,  9088,  9178,  9270,  9363,  9457,
	9551,  9647,  9743,  9841,  9939,  10038, 10139, 10240, 10343, 10446, 10550, 10656, 10763,
	10870, 10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771, 11889, 12008, 12128, 12249,
	12371, 12495, 12620, 12746, 12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802, 13940,
	14080, 14221, 14363, 14506, 14652, 14798, 14946, 15096, 15247, 15399, 15553, 15709, 15866,
	16024, 16184, 16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526, 17701, 17878, 18057,
	18237, 18420, 18604, 18790, 18978, 19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
	20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253, 22475, 22700, 22927, 23156, 23388,
	23622, 23858, 24097, 24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093, 26354, 26618,
	26884, 27153, 27424, 27699, 27976, 28255, 28538, 28823, 29112, 29403, 29697, 29994, 30294,
	30597, 30903, 31212, 31524, 31839, 32157, 32479, 32804
};

static int inv_icm406xx_reg_2_accel_fsr(ICM406XX_ACCEL_CONFIG0_FS_SEL_t reg)
{
	switch (reg) {
#ifndef ICM40609D
	case ICM406XX_ACCEL_CONFIG0_FS_SEL_1g:
		return 1000;
	case ICM406XX_ACCEL_CONFIG0_FS_SEL_2g:
		return 2000;
#endif
	case ICM406XX_ACCEL_CONFIG0_FS_SEL_4g:
		return 4000;
	case ICM406XX_ACCEL_CONFIG0_FS_SEL_8g:
		return 8000;
	case ICM406XX_ACCEL_CONFIG0_FS_SEL_16g:
		return 16000;
#ifdef ICM40609D
	case ICM406XX_ACCEL_CONFIG0_FS_SEL_32g:
		return 32000;
#endif
	default:
		return -1;
	}
}
static int inv_icm406xx_reg_2_gyro_fsr(ICM406XX_GYRO_CONFIG0_FS_SEL_t reg)
{
	switch (reg) {
	case ICM406XX_GYRO_CONFIG0_FS_SEL_16dps:
		return 16;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_31dps:
		return 31;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_62dps:
		return 62;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_125dps:
		return 125;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_250dps:
		return 250;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_500dps:
		return 500;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_1000dps:
		return 1000;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_2000dps:
		return 2000;
	default:
		return -1;
	}
}

static int save_settings(struct inv_icm406xx *s, struct recover_regs *saved_regs)
{
	int     result = 0;
	uint8_t bank   = 0;

	result |= inv_icm406xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &saved_regs->pwr_mgmt_0);
	result |= inv_icm406xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &saved_regs->accel_config0);
	result |= inv_icm406xx_read_reg(s, MPUREG_ACCEL_CONFIG1, 1, &saved_regs->accel_config1);
	result |= inv_icm406xx_read_reg(s, MPUREG_GYRO_CONFIG0, 1, &saved_regs->gyro_config0);
	result |= inv_icm406xx_read_reg(s, MPUREG_GYRO_CONFIG1, 1, &saved_regs->gyro_config1);
	result |=
	    inv_icm406xx_read_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &saved_regs->accel_gyro_config0);

	bank = 1;
	result |= inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);
	result |= inv_icm406xx_read_reg(s, MPUREG_SELF_TEST_CONFIG, 1, &saved_regs->self_test_config);

	bank = 0;
	result |= inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	return result;
}

static int recover_settings(struct inv_icm406xx *s, const struct recover_regs *saved_regs)
{
	int     result = 0;
	uint8_t bank;

	/* Set en_g{x/y/z}_st_d2a to 0 disable self-test for each axis */
	bank = 1;
	inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);
	inv_icm406xx_write_reg(s, MPUREG_SELF_TEST_CONFIG, 1, &saved_regs->self_test_config);

	/*Restore gyro_avg_filt_rate , gyro_dec2_m2_ord, gyro_ui_filt_ord_ind and gyro_ui_filt_bw_ind to previous values.*/
	bank = 0;
	inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	result |= inv_icm406xx_write_reg(s, MPUREG_ACCEL_CONFIG0, 1, &saved_regs->pwr_mgmt_0);
	result |= inv_icm406xx_write_reg(s, MPUREG_ACCEL_CONFIG0, 1, &saved_regs->accel_config0);
	result |= inv_icm406xx_write_reg(s, MPUREG_ACCEL_CONFIG1, 1, &saved_regs->accel_config1);
	result |= inv_icm406xx_write_reg(s, MPUREG_GYRO_CONFIG0, 1, &saved_regs->gyro_config0);
	result |= inv_icm406xx_write_reg(s, MPUREG_GYRO_CONFIG1, 1, &saved_regs->gyro_config1);
	result |=
	    inv_icm406xx_write_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &saved_regs->accel_gyro_config0);
	/* wait 200ms for gyro output to settle */
	inv_icm406xx_sleep_us(200000);

	return result;
}

static int do_average_on_sensor_output(struct inv_icm406xx *s, int sensor, int self_test_flag,
                                       int32_t sensor_result[3])
{
	int     result = 0;
	int     it = 0, sample_discarded = 0, timeout;
	uint8_t data, reg, ST_sensor_data[6];
	int16_t ST_buffer[3] = { 0 };
	int32_t ST_sum[3]    = { 0 };

	if (self_test_flag) {
		inv_icm406xx_read_reg(s, MPUREG_SELF_TEST_CONFIG, 1, &data);
		data |= self_test_flag;
		inv_icm406xx_write_reg(s, MPUREG_SELF_TEST_CONFIG, 1, &data);

		if (sensor == INV_ICM406XX_SENSOR_ON_MASK_GYRO)
			/* wait 200ms for the oscillation to stabilize */
			inv_icm406xx_sleep_us(200000);
		else
			/* wait for 25ms to allow output to settle */
			inv_icm406xx_sleep_us(25000);
	}

	if (sensor == INV_ICM406XX_SENSOR_ON_MASK_GYRO) {
		reg = MPUREG_GYRO_DATA_X1_UI;
		result |= inv_icm406xx_enable_gyro_low_noise_mode(s);
		/*  Workaround HW Bug 9863 : Wait 60ms to discard the
		invalid data, about 60ms after enable */
		inv_icm406xx_sleep_us(60000);
	} else if (sensor == INV_ICM406XX_SENSOR_ON_MASK_ACCEL) {
		reg = MPUREG_ACCEL_DATA_X1_UI;
		result |= inv_icm406xx_enable_accel_low_noise_mode(s);
		/* wait for 25ms to allow output to settle */
		inv_icm406xx_sleep_us(25000);
	} else
		return result;

	timeout = 200;
	do {
		uint8_t int_status;
		inv_icm406xx_read_reg(s, MPUREG_INT_STATUS, 1, &int_status);

		if (int_status & BIT_INT_STATUS_DRDY) {
			inv_icm406xx_read_reg(s, reg, 6, ST_sensor_data);
			ST_buffer[0] = (ST_sensor_data[1] << 8) + ST_sensor_data[0];
			ST_buffer[1] = (ST_sensor_data[3] << 8) + ST_sensor_data[2];
			ST_buffer[2] = (ST_sensor_data[5] << 8) + ST_sensor_data[4];
			if ((ST_buffer[0] != -32768) && (ST_buffer[1] != -32768) && (ST_buffer[2] != -32768)) {
				ST_sum[0] += ST_buffer[0];
				ST_sum[1] += ST_buffer[1];
				ST_sum[2] += ST_buffer[2];
			} else {
				sample_discarded++;
			}
			it++;
		}
		inv_icm406xx_sleep_us(1000);
		timeout--;
	} while ((it < 200) && (timeout > 0));

	result |= inv_icm406xx_disable_accel(s);
	result |= inv_icm406xx_disable_gyro(s);
	inv_icm406xx_sleep_us(150000);

	it -= sample_discarded;
	sensor_result[0] = (ST_sum[0] / it);
	sensor_result[1] = (ST_sum[1] / it);
	sensor_result[2] = (ST_sum[2] / it);

	if (self_test_flag) {
		data &= ~self_test_flag;
		inv_icm406xx_write_reg(s, MPUREG_SELF_TEST_CONFIG, 1, &data);
	}

	return result;
}

static int check_gyro_self_test(struct inv_icm406xx *s, struct recover_regs *saved_reg)
{
	int                            result = 1;
	int                            rc = 0, i = 0, otp_value_zero = 0;
	uint8_t                        data, bank, regs[3];
	int32_t                        ST_OFF_gyro[3], ST_ON_gyro[3];
	uint32_t                       ST_gyro_res[3], ST_gyro_otp[3];
	uint32_t                       gyro_sensitivity_1dps;
	ICM406XX_GYRO_CONFIG0_FS_SEL_t fsr;

	/** Check Gyro self-test response */

	/* set configuration values to set ODR to 1kHz and bandwith to 100Hz (ODR/10)*/

	data = DEFAULT_ST_GYRO_UI_FILT_ORD_3;
	rc |= inv_icm406xx_write_reg(s, MPUREG_GYRO_CONFIG1, 1, &data);

	data = DEFAULT_ST_GYRO_UI_FILT_BW_ODR_10;
	rc |= inv_icm406xx_write_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &data);

	data = DEFAULT_ST_GYRO_ODR_1_KHZ | DEFAULT_ST_GYRO_FSR_250_DPS;
	rc |= inv_icm406xx_write_reg(s, MPUREG_GYRO_CONFIG0, 1, &data);

	/* read average gyro digital output for each axis and store them as ST_OFF_{x,y,z} in lsb x 1000 */
	rc |= do_average_on_sensor_output(s, INV_ICM406XX_SENSOR_ON_MASK_GYRO, 0, ST_OFF_gyro);

	/* enable self-test for each axis */
	/* then read average gyro digital output for each axis and store them as ST_ON_{x,y,z} in lsb x 1000 */
	rc |= do_average_on_sensor_output(s, INV_ICM406XX_SENSOR_ON_MASK_GYRO,
	                                  (BIT_GYRO_X_ST_EN + BIT_GYRO_Y_ST_EN + BIT_GYRO_Z_ST_EN),
	                                  ST_ON_gyro);

	/* calculate the self-test response as ABS(ST_ON_{x,y,z} - ST_OFF_{x,y,z}) for each axis */
	for (i = 0; i < 3; i++) {
		ST_gyro_res[i] = INV_ABS(ST_ON_gyro[i] - ST_OFF_gyro[i]);
	}

	/** Trim Gyro self-test response */

	/* calculate ST results OTP based on the equation */
	bank = 1;
	inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);
	rc |= inv_icm406xx_read_reg(s, MPUREG_XG_ST_DATA_B1, 3, regs);
	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			ST_gyro_otp[i] = sSelfTestEquation[regs[i] - 1];
		} else {
			ST_gyro_otp[i] = 0;
			otp_value_zero = 1;
		}
	}
	bank = 0;
	inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	/** Check Gyro self-test delta */
	rc |= inv_icm406xx_rd_gyro_config0_fs_sel(s, &fsr);
	/* test if ST results match the expected results */
	gyro_sensitivity_1dps = 32768 / inv_icm406xx_reg_2_gyro_fsr(fsr);

	/* calculate the ST delta; the change in ST results respect to the stored ST results in % */
	for (i = 0; i < 3; i++) {
		if (!otp_value_zero) {
			float ratio;
			ratio = (float)ST_gyro_res[i] / (float)ST_gyro_otp[i];
			if (ratio <= 0.5)
				result = 0;
		} else {
			if (ST_gyro_res[i] < (MIN_DEFAULT_ST_GYRO_DPS * gyro_sensitivity_1dps))
				result = 0;
		}
	}

	/* stored the computed bias */
	for (i = 0; i < 3; i++) {
		s->gyro_st_bias[i] = ST_OFF_gyro[i];
		/* Check bias */
		if ((uint32_t)INV_ABS(s->gyro_st_bias[i]) >
		    (MAX_DEFAULT_ST_GYRO_OFFSET_DPS * gyro_sensitivity_1dps))
			result = 0;
	}

	return (rc | result);
}

static int check_accel_self_test(struct inv_icm406xx *s, struct recover_regs *saved_reg)
{
	int                             result = 1;
	int                             rc = 0, i, otp_value_zero = 0, axis, axis_sign;
	uint8_t                         data, bank, regs[3];
	int32_t                         ST_OFF_accel[3], ST_ON_accel[3];
	uint32_t                        ST_accel_res[3], ST_accel_otp[3];
	uint32_t                        acc_sensitivity_1g, gravity;
	ICM406XX_ACCEL_CONFIG0_FS_SEL_t fsr;

	/** Check Accel self-test */

	/* set configuration values to set ODR to 1kHz and bandwith to 100Hz (ODR/10)*/

	data = DEFAULT_ST_ACCEL_UI_FILT_BW_ODR_10;
	rc |= inv_icm406xx_write_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &data);

	data = DEFAULT_ST_ACCEL_UI_FILT_ORD_3;
	rc |= inv_icm406xx_write_reg(s, MPUREG_ACCEL_CONFIG1, 1, &data);

	data = DEFAULT_ST_ACCEL_ODR_1_KHZ | DEFAULT_ST_ACCEL_FSR_2_G;
	rc |= inv_icm406xx_write_reg(s, MPUREG_ACCEL_CONFIG0, 1, &data);

	/* read average gyro digital output for each axis and store them as ST_OFF_{x,y,z} in lsb x 1000 */
	rc |= do_average_on_sensor_output(s, INV_ICM406XX_SENSOR_ON_MASK_ACCEL, 0, ST_OFF_accel);

	/* enable self-test for each axis */
	/* then read average gyro digital output for each axis and store them as ST_ON_{x,y,z} in lsb x 1000 */
	rc |= do_average_on_sensor_output(
	    s, INV_ICM406XX_SENSOR_ON_MASK_ACCEL,
	    (BIT_ACCEL_X_ST_EN + BIT_ACCEL_Y_ST_EN + BIT_ACCEL_Z_ST_EN + BIT_ST_REGULATOR_EN),
	    ST_ON_accel);

	/* calculate the self-test response as ABS(ST_ON_{x,y,z} - ST_OFF_{x,y,z}) for each axis */
	/* outputs from this routine are in units of lsb and hence are dependent on the full-scale used on the DUT */
	for (i = 0; i < 3; i++) {
		ST_accel_res[i] = INV_ABS(ST_ON_accel[i] - ST_OFF_accel[i]);
	}

	/** Trim Accel self-test */

	/* calculate ST results OTP based on the equation */
	bank = 2;
	rc |= inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);
	rc |= inv_icm406xx_read_reg(s, MPUREG_XA_ST_DATA_B2, 3, regs);
	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			ST_accel_otp[i] = sSelfTestEquation[regs[i] - 1];
		} else {
			otp_value_zero = 1;
		}
	}
	bank = 0;
	rc |= inv_icm406xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	/** Check Accel self-test delta */

	rc |= inv_icm406xx_rd_accel_config0_fs_sel(s, &fsr);
	/* test if ST results match the expected results */
	acc_sensitivity_1g = 32768 / (inv_icm406xx_reg_2_accel_fsr(fsr) / 1000);

	/* calculate the ST delta; the change in ST results respect to the stored ST results in % */
	for (i = 0; i < 3; i++) {
		if (!otp_value_zero) {
			float ratio = (float)ST_accel_res[i] / (float)ST_accel_otp[i];
			if ((ratio <= 0.5) || (ratio >= 1.5))
				result = 0;
		} else {
			if ((ST_accel_res[i] < (MIN_DEFAULT_ST_ACCEL_MG * acc_sensitivity_1g / 1000)) ||
			    (ST_accel_res[i] > (MAX_DEFAULT_ST_ACCEL_MG * acc_sensitivity_1g / 1000)))
				result = 0;
		}
	}

	/* stored the computed offset */
	for (i = 0; i < 3; i++) {
		s->accel_st_bias[i] = ST_OFF_accel[i];
	}

	/* assume the largest data axis shows +1 or -1 gee for gravity */
	axis      = 0;
	axis_sign = 1;
	if (INV_ABS(s->accel_st_bias[1]) > INV_ABS(s->accel_st_bias[0]))
		axis = 1;
	if (INV_ABS(s->accel_st_bias[2]) > INV_ABS(s->accel_st_bias[axis]))
		axis = 2;
	if (s->accel_st_bias[axis] < 0)
		axis_sign = -1;

	gravity = acc_sensitivity_1g * axis_sign;
	s->accel_st_bias[axis] -= gravity;

	return (rc | result);
}

static void set_offsetreg(struct inv_icm406xx *s, uint8_t sensor)
{
	uint8_t val;

	/* Set offset registers sensor */
	if (sensor == INV_ICM406XX_SENSOR_ON_MASK_ACCEL) {
		// AX_H / GZ_H
		val = (-s->accel_st_bias[0] >> 4) & 0xf0;
		val |= ((-s->gyro_st_bias[2] >> 8) & 0x0f);
		inv_icm406xx_write_reg(s, MPUREG_GOS_USER4, 1, &val);

		// AX_L
		val = (-s->accel_st_bias[0]) & 0xff;
		inv_icm406xx_write_reg(s, MPUREG_GOS_USER5, 1, &val);

		// AY_L
		val = (-s->accel_st_bias[1]) & 0xff;
		inv_icm406xx_write_reg(s, MPUREG_GOS_USER6, 1, &val);

		// AZ_H / AY_H
		val = (-s->accel_st_bias[2] >> 4) & 0xf0;
		val |= ((-s->accel_st_bias[1] >> 8) & 0x0f);
		inv_icm406xx_write_reg(s, MPUREG_GOS_USER7, 1, &val);

		// AZ_L
		val = (-s->accel_st_bias[2]) & 0xff;
		inv_icm406xx_write_reg(s, MPUREG_GOS_USER8, 1, &val);

	} else if (sensor == INV_ICM406XX_SENSOR_ON_MASK_GYRO) {
		// GX_L
		val = (-s->gyro_st_bias[0]) & 0xff;
		inv_icm406xx_write_reg(s, MPUREG_GOS_USER0, 1, &val);

		// GY_H / GX_H
		val = (-s->gyro_st_bias[1] >> 4) & 0xf0;
		val |= ((-s->gyro_st_bias[0] >> 8) & 0x0f);
		inv_icm406xx_write_reg(s, MPUREG_GOS_USER1, 1, &val);

		// GY_L
		val = (-s->gyro_st_bias[1]) & 0xff;
		inv_icm406xx_write_reg(s, MPUREG_GOS_USER2, 1, &val);

		// GZ_L
		val = (-s->gyro_st_bias[2]) & 0xff;
		inv_icm406xx_write_reg(s, MPUREG_GOS_USER3, 1, &val);

		// AX_H / GZ_H
		val = (-s->accel_st_bias[0] >> 4) & 0xf0;
		val |= ((-s->gyro_st_bias[2] >> 8) & 0x0f);
		inv_icm406xx_write_reg(s, MPUREG_GOS_USER4, 1, &val);
	}
}

int inv_icm406xx_run_selftest(struct inv_icm406xx *s)
{
	int                 result = 0, gyro_result = 0, accel_result = 0;
	struct recover_regs saved_regs;

	result |= save_settings(s, &saved_regs);

	gyro_result = check_gyro_self_test(s, &saved_regs);
	if (gyro_result)
		set_offsetreg(s, INV_ICM406XX_SENSOR_ON_MASK_GYRO);

	accel_result = check_accel_self_test(s, &saved_regs);
	if (accel_result)
		set_offsetreg(s, INV_ICM406XX_SENSOR_ON_MASK_ACCEL);

	result |= recover_settings(s, &saved_regs);

	if (result)
		return result;
	else
		return (accel_result << 1) | gyro_result;
}

static int convert_bias(uint32_t current_FS, int current_bias, uint32_t FS)
{
	int bias;
	if (current_FS <= FS)
		bias = current_bias * (int)(FS / current_FS);
	else
		bias = current_bias / (int)(current_FS / FS);
	return bias;
}

void inv_icm406xx_get_st_bias(struct inv_icm406xx *s, int *st_bias)
{
	int                             i, t;
	int                             check;
	int                             scale;
	ICM406XX_GYRO_CONFIG0_FS_SEL_t  gfsr;
	uint32_t                        gyro_sensitivity_1dps;
	uint32_t                        ST_gyro_sensitivity_1dps = 32768 / DEFAULT_ST_GYRO_FSR_DPS;
	ICM406XX_ACCEL_CONFIG0_FS_SEL_t afsr;
	uint32_t                        acc_sensitivity_1g;
	uint32_t                        ST_acc_sensitivity_1g = 32768 / DEFAULT_ST_ACCEL_FSR_G;

	inv_icm406xx_rd_gyro_config0_fs_sel(s, &gfsr);
	gyro_sensitivity_1dps = 32768 / inv_icm406xx_reg_2_gyro_fsr(gfsr);

	inv_icm406xx_rd_accel_config0_fs_sel(s, &afsr);
	acc_sensitivity_1g = 32768 / (inv_icm406xx_reg_2_accel_fsr(afsr) / 1000);

	/* check bias there ? */
	check = 0;
	for (i = 0; i < 3; i++) {
		if (s->gyro_st_bias[i] != 0)
			check = 1;
		if (s->accel_st_bias[i] != 0)
			check = 1;
	}

	/* if no bias, return all 0 */
	if (check == 0) {
		for (i = 0; i < 6; i++)
			st_bias[i] = 0;
		return;
	}

	/* dps scaled by 2^16 */
	scale = 65536 / gyro_sensitivity_1dps;

	/* Gyro bias LN */
	t = 0;
	for (i = 0; i < 3; i++) {
		/* convert bias to the current FSR */
		st_bias[i + t] =
		    convert_bias(ST_gyro_sensitivity_1dps, s->gyro_st_bias[i], gyro_sensitivity_1dps);
		st_bias[i + t] *= scale;
	}

	/* Gyro bias LP : LP mode not yet supported */
	t += 3;
	for (i = 0; i < 3; i++) {
		/* convert bias to the current FSR */
		st_bias[i + t] =
		    convert_bias(ST_gyro_sensitivity_1dps, s->gyro_lp_st_bias[i], gyro_sensitivity_1dps);
		st_bias[i + t] *= scale;
	}

	/* gee scaled by 2^16 */
	scale = 65536 / acc_sensitivity_1g;

	/* Accel bias */
	t += 3;
	for (i = 0; i < 3; i++) {
		/* convert bias to the current FSR */
		st_bias[i + t] =
		    convert_bias(ST_acc_sensitivity_1g, s->accel_st_bias[i], acc_sensitivity_1g);
		st_bias[i + t] *= scale;
	}

	/* Accel bias LP : LP mode not yet supported */
	t += 3;
	for (i = 0; i < 3; i++) {
		/* convert bias to the current FSR */
		st_bias[i + t] =
		    convert_bias(ST_acc_sensitivity_1g, s->accel_lp_st_bias[i], acc_sensitivity_1g);
		st_bias[i + t] *= scale;
	}
}

void inv_icm406xx_set_st_bias(struct inv_icm406xx *s, int *st_bias)
{
	int                             i, t;
	int                             scale;
	ICM406XX_GYRO_CONFIG0_FS_SEL_t  gfsr;
	uint32_t                        gyro_sensitivity_1dps;
	uint32_t                        ST_gyro_sensitivity_1dps = 32768 / DEFAULT_ST_GYRO_FSR_DPS;
	ICM406XX_ACCEL_CONFIG0_FS_SEL_t afsr;
	uint32_t                        acc_sensitivity_1g;
	uint32_t                        ST_acc_sensitivity_1g = 32768 / DEFAULT_ST_ACCEL_FSR_G;

	inv_icm406xx_rd_gyro_config0_fs_sel(s, &gfsr);
	gyro_sensitivity_1dps = 32768 / inv_icm406xx_reg_2_gyro_fsr(gfsr);

	inv_icm406xx_rd_accel_config0_fs_sel(s, &afsr);
	acc_sensitivity_1g = 32768 / (inv_icm406xx_reg_2_accel_fsr(afsr) / 1000);

	/* dps scaled by 2^16 */
	scale = 65536 / gyro_sensitivity_1dps;

	/* Gyro normal mode */
	t = 0;
	for (i = 0; i < 3; i++) {
		/* convert bias to the current FSR */
		s->gyro_st_bias[i] =
		    convert_bias(gyro_sensitivity_1dps, st_bias[i + t], ST_gyro_sensitivity_1dps);
		s->gyro_st_bias[i] /= scale;
	}
	set_offsetreg(s, INV_ICM406XX_SENSOR_ON_MASK_GYRO);

	/* Gyro LP mode (not supported) */
	t += 3;

	/* gee scaled by 2^16 */
	scale = 65536 / acc_sensitivity_1g;

	/* Accel normal mode */
	t += 3;
	for (i = 0; i < 3; i++) {
		/* convert bias to the current FSR */
		s->accel_st_bias[i] =
		    convert_bias(acc_sensitivity_1g, st_bias[i + t], ST_acc_sensitivity_1g);
		s->accel_st_bias[i] /= scale;
	}

	/* Accel LP mode */
	t += 3;
	for (i = 0; i < 3; i++) {
		/* convert bias to the current FSR */
		s->accel_lp_st_bias[i] =
		    convert_bias(acc_sensitivity_1g, st_bias[i + t], ST_acc_sensitivity_1g);
		s->accel_lp_st_bias[i] /= scale;
	}

	set_offsetreg(s, INV_ICM406XX_SENSOR_ON_MASK_ACCEL);
}
