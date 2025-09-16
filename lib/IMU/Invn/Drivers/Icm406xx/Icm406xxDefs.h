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

#ifndef _INV_ICM406XX_DEFS_H_
#define _INV_ICM406XX_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @file Icm406xxDefs.h
 * File exposing the device register map
 */

#include <stdint.h>

/* forward declaration */
struct inv_icm406xx;

/* ----------------------------------------------------------------------------
 * Device Register map
 *
 * Next macros defines address for all icm406xx registers as listed by device 
 * datasheet.
 * Macros name is MPUREG_<REGISTER_NAME> with REGISTER_NAME being the name of
 * the corresponding register in datasheet.
 * Note that macro name is MPUREG_<REGISTER_NAME>_Bx with x being the bank 
 * number for registers that are in bank 1 and 2 (suffix is ommitted for 
 * bank0 registers)
 * ---------------------------------------------------------------------------- */

/* Bank 0 */
#define MPUREG_DEVICE_CONFIG      0x11
#define MPUREG_DRIVE_CONFIG       0x19
#define MPUREG_INT_CONFIG         0x14
#define MPUREG_FIFO_CONFIG        0x16
#define MPUREG_TEMP_DATA1_UI      0x1D
#define MPUREG_TEMP_DATA0_UI      0x1E
#define MPUREG_ACCEL_DATA_X1_UI   0x1F
#define MPUREG_ACCEL_DATA_X0_UI   0x20
#define MPUREG_ACCEL_DATA_Y1_UI   0x21
#define MPUREG_ACCEL_DATA_Y0_UI   0x22
#define MPUREG_ACCEL_DATA_Z1_UI   0x23
#define MPUREG_ACCEL_DATA_Z0_UI   0x24
#define MPUREG_GYRO_DATA_X1_UI    0x25
#define MPUREG_GYRO_DATA_X0_UI    0x26
#define MPUREG_GYRO_DATA_Y1_UI    0x27
#define MPUREG_GYRO_DATA_Y0_UI    0x28
#define MPUREG_GYRO_DATA_Z1_UI    0x29
#define MPUREG_GYRO_DATA_Z0_UI    0x2A
#define MPUREG_TMST_FSYNCH        0x2B
#define MPUREG_TMST_FSYNCL        0x2C
#define MPUREG_INT_STATUS         0x2D
#define MPUREG_FIFO_COUNTH        0x2E
#define MPUREG_FIFO_COUNTL        0x2F
#define MPUREG_FIFO_DATA          0x30
#define MPUREG_SIGNAL_PATH_RESET  0x4B
#define MPUREG_INTF_CONFIG0       0x4C
#define MPUREG_INTF_CONFIG1       0x4D
#define MPUREG_PWR_MGMT_0         0x4E
#define MPUREG_GYRO_CONFIG0       0x4F
#define MPUREG_ACCEL_CONFIG0      0x50
#define MPUREG_GYRO_CONFIG1       0x51
#define MPUREG_GYRO_ACCEL_CONFIG0 0x52
#define MPUREG_ACCEL_CONFIG1      0x53
#ifndef ICM40609D
#define MPUREG_ACCEL_WOM_X_THR    0x54
#define MPUREG_ACCEL_WOM_Y_THR    0x55
#define MPUREG_ACCEL_WOM_Z_THR    0x56
#endif
#define MPUREG_SMD_CONFIG         0x57
#ifdef ICM40609D
#define MPUREG_INT_STATUS2        0x37
#define MPUREG_TMST_CONFIG        0x54
#define MPUREG_WOM_CONFIG         0x57
#else
#define MPUREG_INT_STATUS2        0x59
#define MPUREG_TMST_CONFIG        0x5A
#endif
#define MPUREG_FIFO_CONFIG1       0x5F
#define MPUREG_FIFO_CONFIG2       0x60
#define MPUREG_FIFO_CONFIG3       0x61
#define MPUREG_FSYNC_CONFIG       0x62
#define MPUREG_INT_CONFIG0        0x63
#define MPUREG_INT_CONFIG1        0x64
#define MPUREG_INT_SOURCE0        0x65
#define MPUREG_INT_SOURCE1        0x66
#define MPUREG_INT_SOURCE2        0x67
#define MPUREG_INT_SOURCE3        0x68
#define MPUREG_INT_SOURCE4        0x69
#define MPUREG_INT_SOURCE5        0x6A
#define MPUREG_FIFO_LOST_PKT0     0x6C
#define MPUREG_FIFO_LOST_PKT1     0x6D
#define MPUREG_SELF_TEST_CONFIG   0x70
#define MPUREG_WHO_AM_I           0x75
#define MPUREG_REG_BANK_SEL       0x76
#define MPUREG_GOS_USER0          0x77
#define MPUREG_GOS_USER1          0x78
#define MPUREG_GOS_USER2          0x79
#define MPUREG_GOS_USER3          0x7A
#define MPUREG_GOS_USER4          0x7B
#define MPUREG_GOS_USER5          0x7C
#define MPUREG_GOS_USER6          0x7D
#define MPUREG_GOS_USER7          0x7E
#define MPUREG_GOS_USER8          0x7F

/* Bank 1 */
#define MPUREG_SENSOR_CONFIG0         0x03
#define MPUREG_GYRO_CONFIG_STATIC2_B1 0x0B
#define MPUREG_GYRO_CONFIG_STATIC3_B1 0x0C
#define MPUREG_GYRO_CONFIG_STATIC4_B1 0x0D
#define MPUREG_GYRO_CONFIG_STATIC5_B1 0x0E
#define MPUREG_GYRO_CONFIG_STATIC6_B1 0x0F
#define MPUREG_GYRO_CONFIG_STATIC7_B1 0x10
#define MPUREG_GYRO_CONFIG_STATIC8_B1 0x11
#define MPUREG_GYRO_CONFIG_STATIC9_B1 0x12
#define MPUREG_GYRO_CONFIG_STATIC10_B1 0x13
#define MPUREG_XG_ST_DATA_B1          0x5F
#define MPUREG_YG_ST_DATA_B1          0x60
#define MPUREG_ZG_ST_DATA_B1          0x61
#define MPUREG_TMST_VAL0_B1           0x62
#define MPUREG_TMST_VAL1_B1           0x63
#define MPUREG_TMST_VAL2_B1           0x64
#define MPUREG_INTF_CONFIG4_B1        0x7A

/* Bank 2 */
#define MPU_ACCEL_CONFIG_STATIC2_B2 0x03
#define MPU_ACCEL_CONFIG_STATIC3_B2 0x04
#define MPU_ACCEL_CONFIG_STATIC4_B2 0x05
#define MPUREG_XA_ST_DATA_B2 0x3B
#define MPUREG_YA_ST_DATA_B2 0x3C
#define MPUREG_ZA_ST_DATA_B2 0x3D

#define MPUREG_AUX1_CONFIG1_B2       0x44
#define MPUREG_AUX1_CONFIG2_B2       0x45
#define MPUREG_AUX1_CONFIG3_B2       0x46
#define MPUREG_TEMP_DATA0_AUX1_B2    0x47
#define MPUREG_TEMP_DATA1_AUX1_B2    0x48
#define MPUREG_ACCEL_DATA_X0_AUX1_B2 0x49
#define MPUREG_ACCEL_DATA_X1_AUX1_B2 0x4A
#define MPUREG_ACCEL_DATA_Y0_AUX1_B2 0x4B
#define MPUREG_ACCEL_DATA_Y1_AUX1_B2 0x4C
#define MPUREG_ACCEL_DATA_Z0_AUX1_B2 0x4D
#define MPUREG_ACCEL_DATA_Z1_AUX1_B2 0x4E
#define MPUREG_GYRO_DATA_X0_AUX1_B2  0x4F
#define MPUREG_GYRO_DATA_X1_AUX1_B2  0x50
#define MPUREG_GYRO_DATA_Y0_AUX1_B2  0x51
#define MPUREG_GYRO_DATA_Y1_AUX1_B2  0x52
#define MPUREG_GYRO_DATA_Z0_AUX1_B2  0x53
#define MPUREG_GYRO_DATA_Z1_AUX1_B2  0x54
#define MPUREG_TMSTVAL0_AUX1_B2      0x55
#define MPUREG_TMSTVAL1_AUX1_B2      0x56
#define MPUREG_INT_STATUS_AUX1_B2    0x57

#define MPUREG_AUX2_CONFIG1_B2       0x59
#define MPUREG_AUX2_CONFIG2_B2       0x5A
#define MPUREG_AUX2_CONFIG3_B2       0x5B
#define MPUREG_TEMP_DATA0_AUX2_B2    0x5C
#define MPUREG_TEMP_DATA1_AUX2_B2    0x5D
#define MPUREG_ACCEL_DATA_X0_AUX2_B2 0x5E
#define MPUREG_ACCEL_DATA_X1_AUX2_B2 0x5F
#define MPUREG_ACCEL_DATA_Y0_AUX2_B2 0x60
#define MPUREG_ACCEL_DATA_Y1_AUX2_B2 0x61
#define MPUREG_ACCEL_DATA_Z0_AUX2_B2 0x62
#define MPUREG_ACCEL_DATA_Z1_AUX2_B2 0x63
#define MPUREG_GYRO_DATA_X0_AUX2_B2  0x64
#define MPUREG_GYRO_DATA_X1_AUX2_B2  0x65
#define MPUREG_GYRO_DATA_Y0_AUX2_B2  0x66
#define MPUREG_GYRO_DATA_Y1_AUX2_B2  0x67
#define MPUREG_GYRO_DATA_Z0_AUX2_B2  0x68
#define MPUREG_GYRO_DATA_Z1_AUX2_B2  0x69
#define MPUREG_TMSTVAL0_AUX2_B2      0x6A
#define MPUREG_TMSTVAL1_AUX2_B2      0x6B
#define MPUREG_INT_STATUS_AUX2_B2    0x6C

#ifdef ICM40609D
/* Bank 4 */
#define MPUREG_ACCEL_WOM_X_THR    0x4A
#define MPUREG_ACCEL_WOM_Y_THR    0x4B
#define MPUREG_ACCEL_WOM_Z_THR    0x4C
#endif
#define MPUREG_OFFSET_USER0_B4 0x77
#define MPUREG_OFFSET_USER1_B4 0x78
#define MPUREG_OFFSET_USER2_B4 0x79
#define MPUREG_OFFSET_USER3_B4 0x7A
#define MPUREG_OFFSET_USER4_B4 0x7B
#define MPUREG_OFFSET_USER5_B4 0x7C
#define MPUREG_OFFSET_USER6_B4 0x7D
#define MPUREG_OFFSET_USER7_B4 0x7E
#define MPUREG_OFFSET_USER8_B4 0x7F

/* ----------------------------------------------------------------------------
 * Device features
 *
 * Next macros define some of the device features such as FIFO, sensor data
 * size or whoami value.
 * ---------------------------------------------------------------------------- */

#define ICM40609D_WHOAMI 0x3B
#define ICM40605_WHOAMI 0x33
#define ICM40604_WHOAMI 0x32
#define ICM40602_WHOAMI 0x31

#define ACCEL_DATA_SIZE 6
#define GYRO_DATA_SIZE  6

#define FIFO_HEADER_SIZE     1
#define FIFO_ACCEL_DATA_SIZE ACCEL_DATA_SIZE
#define FIFO_GYRO_DATA_SIZE  GYRO_DATA_SIZE
#define FIFO_TEMP_DATA_SIZE  1
#define FIFO_TS_FSYNC_SIZE   2

#define FIFO_16BYTES_PACKET_SIZE                                                                   \
	(FIFO_HEADER_SIZE + FIFO_ACCEL_DATA_SIZE + FIFO_GYRO_DATA_SIZE + FIFO_TEMP_DATA_SIZE +         \
	 FIFO_TS_FSYNC_SIZE)

#define FIFO_HEADER_ODR_ACCEL 0x01
#define FIFO_HEADER_ODR_GYRO  0x02
#define FIFO_HEADER_FSYNC     0x04
#define FIFO_HEADER_TMST      0x08
#define FIFO_HEADER_HEADER_20 0x10
#define FIFO_HEADER_GYRO      0x20
#define FIFO_HEADER_ACC       0x40
#define FIFO_HEADER_MSG       0x80

#define INVALID_VALUE_FIFO    ((int16_t)0x8000)
#define INVALID_VALUE_FIFO_1B ((int8_t)0x80)

typedef union {
	unsigned char Byte;
	struct {
		unsigned char gyro_20bit_ext : 1;
		unsigned char accel_20bit_ext : 1;
		unsigned char fsync_bit : 1;
		unsigned char timestamp_bit : 1;
		unsigned char twentybits_bit : 1;
		unsigned char gyro_bit : 1;
		unsigned char accel_bit : 1;
		unsigned char msg_bit : 1;
	} bits;
} fifo_header_t;

/* ----------------------------------------------------------------------------
 * Device registers description
 *
 * Next section defines some of the registers bitfield and declare corresponding
 * accessors.
 * Note that descriptors and accessors are not provided for all the registers 
 * but only for the most useful ones.
 * For all details on registers and bitfields functionalities please refer to 
 * the device datasheet.
 * ---------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
 * register bank 0 
 * ---------------------------------------------------------------------------- */

/*
 * MPUREG_DEVICE_CONFIG
 * Register Name : DEVICE_CONFIG
 */

/* SPI_MODE */
#define BIT_DEVICE_CONFIG_SPI_MODE_POS  1
#define BIT_DEVICE_CONFIG_SPI_MODE_MASK (0x1 << BIT_DEVICE_CONFIG_SPI_MODE_POS)

typedef enum {
	ICM406XX_DEVICE_CONFIG_SPI_MODE_1_2 = (0x1 << BIT_DEVICE_CONFIG_SPI_MODE_POS),
	ICM406XX_DEVICE_CONFIG_SPI_MODE_0_3 = (0x0 << BIT_DEVICE_CONFIG_SPI_MODE_POS),
} ICM406XX_DEVICE_CONFIG_SPI_MODE_t;

int inv_icm406xx_wr_device_config_spi_mode(struct inv_icm406xx *             s,
                                           ICM406XX_DEVICE_CONFIG_SPI_MODE_t new_value);

/* SOFT_RESET_CONFIG */
#define BIT_DEVICE_CONFIG_RESET_POS  0
#define BIT_DEVICE_CONFIG_RESET_MASK 0x01
typedef enum {
	ICM406XX_DEVICE_CONFIG_RESET_EN   = 0x01,
	ICM406XX_DEVICE_CONFIG_RESET_NONE = 0x00,
} ICM406XX_DEVICE_CONFIG_RESET_t;

int inv_icm406xx_wr_device_config_reset(struct inv_icm406xx *          s,
                                        ICM406XX_DEVICE_CONFIG_RESET_t new_value);

/*
 * MPUREG_INT_CONFIG
 * Register Name: INT_CONFIG
 */

/* INT1_DRIVE_CIRCUIT */
#define BIT_INT_CONFIG_INT1_DRIVE_CIRCUIT_POS  1
#define BIT_INT_CONFIG_INT1_DRIVE_CIRCUIT_MASK (0x01 << BIT_INT_CONFIG_INT1_DRIVE_CIRCUIT_POS)

typedef enum {
	ICM406XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_PP = (0x01 << BIT_INT_CONFIG_INT1_DRIVE_CIRCUIT_POS),
	ICM406XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_OD = (0x00 << BIT_INT_CONFIG_INT1_DRIVE_CIRCUIT_POS),
} ICM406XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_t;

int inv_icm406xx_wr_int_cfg_int1_drive_circuit(struct inv_icm406xx *                    s,
                                               ICM406XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_t new_value);

/* INT1_POLARITY */
#define BIT_INT_CONFIG_INT1_POLARITY_POS  0
#define BIT_INT_CONFIG_INT1_POLARITY_MASK 0x01

typedef enum {
	ICM406XX_INT_CONFIG_INT1_POLARITY_HIGH = 0x01,
	ICM406XX_INT_CONFIG_INT1_POLARITY_LOW  = 0x00,
} ICM406XX_INT_CONFIG_INT1_POLARITY_t;

int inv_icm406xx_wr_int_cfg_int1_polarity(struct inv_icm406xx *               s,
                                          ICM406XX_INT_CONFIG_INT1_POLARITY_t new_value);

/*
 * MPUREG_FIFO_CONFIG
 * Register Name: FIFO_CONFIG
 */

/* FIFO_MODE */
#define BIT_FIFO_CONFIG_MODE_POS  6
#define BIT_FIFO_CONFIG_MODE_MASK (0x03 << BIT_FIFO_CONFIG_MODE_POS)

typedef enum {
	ICM406XX_FIFO_CONFIG_MODE_SNAPSHOT = (0x02 << BIT_FIFO_CONFIG_MODE_POS),
	ICM406XX_FIFO_CONFIG_MODE_STREAM   = (0x01 << BIT_FIFO_CONFIG_MODE_POS),
	ICM406XX_FIFO_CONFIG_MODE_BYPASS   = (0x00 << BIT_FIFO_CONFIG_MODE_POS),
} ICM406XX_FIFO_CONFIG_MODE_t;

int inv_icm406xx_wr_fifo_config_mode(struct inv_icm406xx *s, ICM406XX_FIFO_CONFIG_MODE_t new_value);
int inv_icm406xx_rd_fifo_config_mode(struct inv_icm406xx *s, ICM406XX_FIFO_CONFIG_MODE_t *value);

/*
 * MPUREG_INT_STATUS
 * Register Name: INT_STATUS
 */
#define BIT_INT_STATUS_UI_FSYNC   0x40
#define BIT_INT_STATUS_PLL_RDY    0x20
#define BIT_INT_STATUS_RESET_DONE 0x10
#define BIT_INT_STATUS_DRDY       0x08
#define BIT_INT_STATUS_FIFO_THS   0x04
#define BIT_INT_STATUS_FIFO_FULL  0x02
#define BIT_INT_STATUS_AGC_RDY    0x01
int inv_icm406xx_rd_int_status(struct inv_icm406xx *s, uint8_t *value);

/*
 * MPUREG_FIFO_COUNTL - MPUREG_FIFO_COUNTH
 * Register Name: FIFO_COUNTL - FIFO_COUNTH
 */
int inv_icm406xx_rd_fifo_count(struct inv_icm406xx *s, uint8_t value[2]);

/*
 * MPUREG_FIFO_DATA
 * Register Name: FIFO_DATA_REG
 */
int inv_icm406xx_rd_fifo(struct inv_icm406xx *s, int len, uint8_t *value);

/*
 * MPUREG_SIGNAL_PATH_RESET
 * Register Name: SIGNAL_PATH_RESET
 */

/* TMST_STROBE */
#define BIT_SIGNAL_PATH_RESET_TMST_STROBE_POS  2
#define BIT_SIGNAL_PATH_RESET_TMST_STROBE_MASK (0x01 << BIT_SIGNAL_PATH_RESET_TMST_STROBE_POS)

typedef enum {
	ICM406XX_SIGNAL_PATH_RESET_TMST_STROBE_EN  = (0x01 << BIT_SIGNAL_PATH_RESET_TMST_STROBE_POS),
	ICM406XX_SIGNAL_PATH_RESET_TMST_STROBE_DIS = (0x00 << BIT_SIGNAL_PATH_RESET_TMST_STROBE_POS),
} ICM406XX_SIGNAL_PATH_RESET_TMST_STROBE_t;

int inv_icm406xx_wr_signal_path_rst_tmst_strobe(struct inv_icm406xx *                    s,
                                                ICM406XX_SIGNAL_PATH_RESET_TMST_STROBE_t new_value);

/* FIFO_FLUSH */
#define BIT_SIGNAL_PATH_RESET_FIFO_FLUSH_POS  1
#define BIT_SIGNAL_PATH_RESET_FIFO_FLUSH_MASK (0x01 << BIT_SIGNAL_PATH_RESET_FIFO_FLUSH_POS)

typedef enum {
	ICM406XX_SIGNAL_PATH_RESET_FIFO_FLUSH_EN  = (0x01 << BIT_SIGNAL_PATH_RESET_FIFO_FLUSH_POS),
	ICM406XX_SIGNAL_PATH_RESET_FIFO_FLUSH_DIS = (0x00 << BIT_SIGNAL_PATH_RESET_FIFO_FLUSH_POS),
} ICM406XX_SIGNAL_PATH_RESET_FIFO_FLUSH_t;

int inv_icm406xx_wr_signal_path_rst_fifo_flush(struct inv_icm406xx *                   s,
                                               ICM406XX_SIGNAL_PATH_RESET_FIFO_FLUSH_t new_value);

/*
 * MPUREG_INTF_CONFIG0
 * Register Name: INTF_CONFIG0
 */

/* FIFO_SREG_INVALID_IND */
#define BIT_FIFO_SREG_INVALID_IND_POS  7
#define BIT_FIFO_SREG_INVALID_IND_MASK (0x01 << BIT_FIFO_SREG_INVALID_IND_POS)

typedef enum {
	ICM406XX_INTF_CONFIG0_FIFO_SREG_INVALID_IND_DIS = (0x01 << BIT_FIFO_SREG_INVALID_IND_POS),
	ICM406XX_INTF_CONFIG0_FIFO_SREG_INVALID_IND_EN  = (0x00 << BIT_FIFO_SREG_INVALID_IND_POS),
} ICM406XX_INTF_CONFIG0_FIFO_SREG_INVALID_IND_t;

int inv_icm406xx_wr_intf_config0_fifo_sreg_invalid_ind(
    struct inv_icm406xx *s, ICM406XX_INTF_CONFIG0_FIFO_SREG_INVALID_IND_t new_value);

/* FIFO_COUNT_REC */
#define BIT_FIFO_COUNT_REC_POS  6
#define BIT_FIFO_COUNT_REC_MASK (0x01 << BIT_FIFO_COUNT_REC_POS)

typedef enum {
	ICM406XX_INTF_CONFIG0_FIFO_COUNT_REC_RECORD = (0x01 << BIT_FIFO_COUNT_REC_POS),
	ICM406XX_INTF_CONFIG0_FIFO_COUNT_REC_BYTE   = (0x00 << BIT_FIFO_COUNT_REC_POS),
} ICM406XX_INTF_CONFIG0_FIFO_COUNT_REC_t;

int inv_icm406xx_wr_intf_config0_fifo_count_rec(struct inv_icm406xx *                  s,
                                                ICM406XX_INTF_CONFIG0_FIFO_COUNT_REC_t new_value);

/* FIFO_COUNT_ENDIAN */
#define BIT_FIFO_COUNT_ENDIAN_POS  5
#define BIT_FIFO_COUNT_ENDIAN_MASK (0x01 << BIT_FIFO_COUNT_ENDIAN_POS)

typedef enum {
	ICM406XX_INTF_CONFIG0_FIFO_COUNT_BIG_ENDIAN    = (0x01 << BIT_FIFO_COUNT_ENDIAN_POS),
	ICM406XX_INTF_CONFIG0_FIFO_COUNT_LITTLE_ENDIAN = (0x00 << BIT_FIFO_COUNT_ENDIAN_POS),
} ICM406XX_INTF_CONFIG0_FIFO_COUNT_ENDIAN_t;

int inv_icm406xx_wr_intf_config0_fifo_count_endian(
    struct inv_icm406xx *s, ICM406XX_INTF_CONFIG0_FIFO_COUNT_ENDIAN_t new_value);

/* DATA_ENDIAN */
#define BIT_DATA_ENDIAN_POS  4
#define BIT_DATA_ENDIAN_MASK (0x01 << BIT_DATA_ENDIAN_POS)

typedef enum {
	ICM406XX_INTF_CONFIG0_DATA_BIG_ENDIAN    = (0x01 << BIT_DATA_ENDIAN_POS),
	ICM406XX_INTF_CONFIG0_DATA_LITTLE_ENDIAN = (0x00 << BIT_DATA_ENDIAN_POS),
} ICM406XX_INTF_CONFIG0_DATA_COUNT_ENDIAN_t;

int inv_icm406xx_wr_intf_config0_data_endian(struct inv_icm406xx *                     s,
                                             ICM406XX_INTF_CONFIG0_DATA_COUNT_ENDIAN_t new_value);
int inv_icm406xx_rd_intf_config0_data_endian(struct inv_icm406xx *                      s,
                                             ICM406XX_INTF_CONFIG0_DATA_COUNT_ENDIAN_t *value);

/* SPI_MODE_AUX2 */
#define BIT_SPI_MODE_OIS2_POS  3
#define BIT_SPI_MODE_OIS2_MASK (0x01 << BIT_SPI_MODE_OIS2_POS)

typedef enum {
	ICM406XX_INTF_CONFIG0_SPI_MODE_OIS2_1_2 = (0x01 << BIT_SPI_MODE_OIS2_POS),
	ICM406XX_INTF_CONFIG0_SPI_MODE_OIS2_0_3 = (0x00 << BIT_SPI_MODE_OIS2_POS),
} ICM406XX_INTF_CONFIG0_SPI_MODE_OIS2_t;

int inv_icm406xx_wr_intf_config0_spi_mode_ois2(struct inv_icm406xx *                 s,
                                               ICM406XX_INTF_CONFIG0_SPI_MODE_OIS2_t new_value);

/* SPI_MODE_AUX1 */
#define BIT_SPI_MODE_OIS1_POS  2
#define BIT_SPI_MODE_OIS1_MASK (0x01 << BIT_SPI_MODE_OIS1_POS)

typedef enum {
	ICM406XX_INTF_CONFIG0_SPI_MODE_OIS1_1_2 = (0x01 << BIT_SPI_MODE_OIS1_POS),
	ICM406XX_INTF_CONFIG0_SPI_MODE_OIS1_0_3 = (0x00 << BIT_SPI_MODE_OIS1_POS),
} ICM406XX_INTF_CONFIG0_SPI_MODE_OIS1_t;

int inv_icm406xx_wr_intf_config0_spi_mode_ois1(struct inv_icm406xx *                 s,
                                               ICM406XX_INTF_CONFIG0_SPI_MODE_OIS1_t new_value);

/*
 * MPUREG_INTF_CONFIG1
 * Register Name: INTF_CONFIG1
 */

/* FIFO_SREG_INVALID_IND */
#define BIT_ACCEL_LP_CLK_POS  3
#define BIT_ACCEL_LP_CLK_MASK (0x01 << BIT_ACCEL_LP_CLK_POS)

typedef enum {
	ICM406XX_INTF_CONFIG1_ACCEL_LP_CLK_SEL_RC = (0x01 << BIT_ACCEL_LP_CLK_POS),
	ICM406XX_INTF_CONFIG1_ACCEL_LP_CLK_SEL_WU  = (0x00 << BIT_ACCEL_LP_CLK_POS),
} ICM406XX_INTF_CONFIG1_ACCEL_LP_CLK_SEL_t;

int inv_icm406xx_wr_intf_config1_accel_lp_clk_sel(
    struct inv_icm406xx *s, ICM406XX_INTF_CONFIG1_ACCEL_LP_CLK_SEL_t new_value);

/*
 * MPUREG_PWR_MGMT_0
 * Register Name: PWR_MGMT_0
 */

/* TEMP_DIS */
#define BIT_PWR_MGMT_0_TEMP_POS  5
#define BIT_PWR_MGMT_0_TEMP_MASK (0x01 << BIT_PWR_MGMT_0_TEMP_POS)

typedef enum {
	ICM406XX_PWR_MGMT_0_TEMP_DIS = (0x01 << BIT_PWR_MGMT_0_TEMP_POS),
	ICM406XX_PWR_MGMT_0_TEMP_EN  = (0x00 << BIT_PWR_MGMT_0_TEMP_POS),
} ICM406XX_PWR_MGMT_0_TEMP_t;

int inv_icm406xx_wr_pwr_mgmt0_temp_en(struct inv_icm406xx *s, ICM406XX_PWR_MGMT_0_TEMP_t new_value);
int inv_icm406xx_rd_pwr_mgmt0_temp_en(struct inv_icm406xx *s, ICM406XX_PWR_MGMT_0_TEMP_t *value);

/* GYRO_MODE */
#define BIT_PWR_MGMT_0_GYRO_MODE_POS  2
#define BIT_PWR_MGMT_0_GYRO_MODE_MASK (0x03 << BIT_PWR_MGMT_0_GYRO_MODE_POS)

typedef enum {
	ICM406XX_PWR_MGMT_0_GYRO_MODE_LN  = (0x03 << BIT_PWR_MGMT_0_GYRO_MODE_POS),
	ICM406XX_PWR_MGMT_0_GYRO_MODE_OFF = (0x00 << BIT_PWR_MGMT_0_GYRO_MODE_POS),
} ICM406XX_PWR_MGMT_0_GYRO_MODE_t;

int inv_icm406xx_wr_pwr_mgmt0_gyro_mode(struct inv_icm406xx *           s,
                                        ICM406XX_PWR_MGMT_0_GYRO_MODE_t new_value);
int inv_icm406xx_rd_pwr_mgmt0_gyro_mode(struct inv_icm406xx *            s,
                                        ICM406XX_PWR_MGMT_0_GYRO_MODE_t *value);

/* ACCEL_MODE */
#define BIT_PWR_MGMT_0_ACCEL_MODE_POS  0
#define BIT_PWR_MGMT_0_ACCEL_MODE_MASK 0x03

typedef enum {
	ICM406XX_PWR_MGMT_0_ACCEL_MODE_LN  = 0x03,
	ICM406XX_PWR_MGMT_0_ACCEL_MODE_LP  = 0x02,
	ICM406XX_PWR_MGMT_0_ACCEL_MODE_OFF = 0x00,
} ICM406XX_PWR_MGMT_0_ACCEL_MODE_t;

int inv_icm406xx_wr_pwr_mgmt0_accel_mode(struct inv_icm406xx *            s,
                                         ICM406XX_PWR_MGMT_0_ACCEL_MODE_t new_value);
int inv_icm406xx_rd_pwr_mgmt0_accel_mode(struct inv_icm406xx *             s,
                                         ICM406XX_PWR_MGMT_0_ACCEL_MODE_t *value);

/*
 * MPUREG_GYRO_CONFIG0
 * Register Name: GYRO_CONFIG0
 */

/* GYRO_FS_SEL*/
#define BIT_GYRO_CONFIG0_FS_SEL_POS  5
#define BIT_GYRO_CONFIG0_FS_SEL_MASK (7 << BIT_GYRO_CONFIG0_FS_SEL_POS)

/** @brief Gyroscope FSR selection 
 */
typedef enum {
	ICM406XX_GYRO_CONFIG0_FS_SEL_16dps   = (7 << BIT_GYRO_CONFIG0_FS_SEL_POS), /*!< 16dps*/
	ICM406XX_GYRO_CONFIG0_FS_SEL_31dps   = (6 << BIT_GYRO_CONFIG0_FS_SEL_POS), /*!< 31dps*/
	ICM406XX_GYRO_CONFIG0_FS_SEL_62dps   = (5 << BIT_GYRO_CONFIG0_FS_SEL_POS), /*!< 62dps*/
	ICM406XX_GYRO_CONFIG0_FS_SEL_125dps  = (4 << BIT_GYRO_CONFIG0_FS_SEL_POS), /*!< 125dps*/
	ICM406XX_GYRO_CONFIG0_FS_SEL_250dps  = (3 << BIT_GYRO_CONFIG0_FS_SEL_POS), /*!< 250dps*/
	ICM406XX_GYRO_CONFIG0_FS_SEL_500dps  = (2 << BIT_GYRO_CONFIG0_FS_SEL_POS), /*!< 500dps*/
	ICM406XX_GYRO_CONFIG0_FS_SEL_1000dps = (1 << BIT_GYRO_CONFIG0_FS_SEL_POS), /*!< 1000dps*/
	ICM406XX_GYRO_CONFIG0_FS_SEL_2000dps = (0 << BIT_GYRO_CONFIG0_FS_SEL_POS), /*!< 2000dps*/
} ICM406XX_GYRO_CONFIG0_FS_SEL_t;

int inv_icm406xx_wr_gyro_config0_fs_sel(struct inv_icm406xx *          s,
                                        ICM406XX_GYRO_CONFIG0_FS_SEL_t new_value);
int inv_icm406xx_rd_gyro_config0_fs_sel(struct inv_icm406xx *           s,
                                        ICM406XX_GYRO_CONFIG0_FS_SEL_t *value);

/* GYRO_ODR */
#define BIT_GYRO_CONFIG0_ODR_POS  0
#define BIT_GYRO_CONFIG0_ODR_MASK 0x0F

/** @brief Gyroscope ODR selection 
 */
typedef enum {
#ifdef ICM40609D
	ICM406XX_GYRO_CONFIG0_ODR_500_HZ  = 0x0F, /*!< 500 Hz (2 ms)*/
#endif
	ICM406XX_GYRO_CONFIG0_ODR_6_25_HZ = 0x0C, /*!< 6.25 Hz (160 ms)*/
	ICM406XX_GYRO_CONFIG0_ODR_12_5_HZ = 0x0B, /*!< 12.5 Hz (80 ms)*/
	ICM406XX_GYRO_CONFIG0_ODR_25_HZ   = 0x0A, /*!< 25 Hz (40 ms)*/
	ICM406XX_GYRO_CONFIG0_ODR_50_HZ   = 0x09, /*!< 50 Hz (20 ms)*/
	ICM406XX_GYRO_CONFIG0_ODR_100_HZ  = 0x08, /*!< 100 Hz (10 ms)*/
	ICM406XX_GYRO_CONFIG0_ODR_200_HZ  = 0x07, /*!< 200 Hz (5 ms)*/
	ICM406XX_GYRO_CONFIG0_ODR_1_KHZ   = 0x06, /*!< 1 KHz (1 ms)*/
	ICM406XX_GYRO_CONFIG0_ODR_2_KHZ   = 0x05, /*!< 2 KHz (500 us)*/
	ICM406XX_GYRO_CONFIG0_ODR_4_KHZ   = 0x04, /*!< 4 KHz (250 us)*/
	ICM406XX_GYRO_CONFIG0_ODR_8_KHZ   = 0x03, /*!< 8 KHz (125 us)*/
	ICM406XX_GYRO_CONFIG0_ODR_16_KHZ  = 0x02, /*!< 16 KHz (62.5 us)*/
	ICM406XX_GYRO_CONFIG0_ODR_32_KHZ  = 0x01, /*!< 32 KHz (31.25 us)*/
#ifndef ICM40609D
	ICM406XX_GYRO_CONFIG0_ODR_64_KHZ  = 0x00, /*!< 64 KHz (15.625 us)*/
#endif
} ICM406XX_GYRO_CONFIG0_ODR_t;

int inv_icm406xx_wr_gyro_config0_odr(struct inv_icm406xx *s, ICM406XX_GYRO_CONFIG0_ODR_t new_value);
int inv_icm406xx_rd_gyro_config0_odr(struct inv_icm406xx *s, ICM406XX_GYRO_CONFIG0_ODR_t *value);

/*
 * MPUREG_ACCEL_CONFIG0
 * Register Name: ACCEL_CONFIG0
 */

/* ACCEL_FS_SEL */
#define BIT_ACCEL_CONFIG0_FS_SEL_POS  5
#define BIT_ACCEL_CONFIG0_FS_SEL_MASK (0x7 << BIT_ACCEL_CONFIG0_FS_SEL_POS)

/** @brief Accelerometer FSR selection 
 */
#ifdef ICM40609D
typedef enum {
	ICM406XX_ACCEL_CONFIG0_FS_SEL_4g  = (0x3 << BIT_ACCEL_CONFIG0_FS_SEL_POS), /*!< 4g*/
	ICM406XX_ACCEL_CONFIG0_FS_SEL_8g  = (0x2 << BIT_ACCEL_CONFIG0_FS_SEL_POS), /*!< 8g*/
	ICM406XX_ACCEL_CONFIG0_FS_SEL_16g = (0x1 << BIT_ACCEL_CONFIG0_FS_SEL_POS), /*!< 16g*/
	ICM406XX_ACCEL_CONFIG0_FS_SEL_32g = (0x0 << BIT_ACCEL_CONFIG0_FS_SEL_POS), /*!< 32g*/
} ICM406XX_ACCEL_CONFIG0_FS_SEL_t;
#else
typedef enum {
	ICM406XX_ACCEL_CONFIG0_FS_SEL_1g  = (0x4 << BIT_ACCEL_CONFIG0_FS_SEL_POS), /*!< 1g*/
	ICM406XX_ACCEL_CONFIG0_FS_SEL_2g  = (0x3 << BIT_ACCEL_CONFIG0_FS_SEL_POS), /*!< 2g*/
	ICM406XX_ACCEL_CONFIG0_FS_SEL_4g  = (0x2 << BIT_ACCEL_CONFIG0_FS_SEL_POS), /*!< 4g*/
	ICM406XX_ACCEL_CONFIG0_FS_SEL_8g  = (0x1 << BIT_ACCEL_CONFIG0_FS_SEL_POS), /*!< 8g*/
	ICM406XX_ACCEL_CONFIG0_FS_SEL_16g = (0x0 << BIT_ACCEL_CONFIG0_FS_SEL_POS), /*!< 16g*/
} ICM406XX_ACCEL_CONFIG0_FS_SEL_t;
#endif

int inv_icm406xx_wr_accel_config0_fs_sel(struct inv_icm406xx *           s,
                                         ICM406XX_ACCEL_CONFIG0_FS_SEL_t new_value);
int inv_icm406xx_rd_accel_config0_fs_sel(struct inv_icm406xx *            s,
                                         ICM406XX_ACCEL_CONFIG0_FS_SEL_t *value);

/* ACCEL_ODR */
#define BIT_ACCEL_CONFIG0_ODR_POS  0
#define BIT_ACCEL_CONFIG0_ODR_MASK 0x0F

/** @brief Accelerometer ODR selection 
 */
#ifdef ICM40609D
typedef enum {
	ICM406XX_ACCEL_CONFIG0_ODR_500_HZ = 0xF, /*!< 500 Hz (2 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_1_5625_HZ = 0xE, /*!< 1.5625 Hz (640 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_3_125_HZ = 0xD, /*!< 3.125 Hz (320 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_6_25_HZ = 0xC, /*!< 6.25 Hz (160 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_12_5_HZ = 0xB, /*!< 12.5 Hz (80 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_25_HZ   = 0xA, /*!< 25 Hz (40 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_50_HZ   = 0x9, /*!< 50 Hz (20 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_100_HZ  = 0x8, /*!< 100 Hz (10 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_200_HZ  = 0x7, /*!< 200 Hz (5 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_1_KHZ   = 0x6, /*!< 1 KHz (1 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_2_KHZ   = 0x5, /*!< 2 KHz (500 us)*/
	ICM406XX_ACCEL_CONFIG0_ODR_4_KHZ   = 0x4, /*!< 4 KHz (250 us)*/
	ICM406XX_ACCEL_CONFIG0_ODR_8_KHZ   = 0x3, /*!< 8 KHz (125 us)*/
	ICM406XX_ACCEL_CONFIG0_ODR_16_KHZ  = 0x2, /*!< 16 KHz (62.5 us)*/
	ICM406XX_ACCEL_CONFIG0_ODR_32_KHZ  = 0x1, /*!< 32 KHz (31.25 us)*/
} ICM406XX_ACCEL_CONFIG0_ODR_t;
#else
typedef enum {
	ICM406XX_ACCEL_CONFIG0_ODR_6_25_HZ = 0xC, /*!< 6.25 Hz (160 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_12_5_HZ = 0xB, /*!< 12.5 Hz (80 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_25_HZ   = 0xA, /*!< 25 Hz (40 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_50_HZ   = 0x9, /*!< 50 Hz (20 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_100_HZ  = 0x8, /*!< 100 Hz (10 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_200_HZ  = 0x7, /*!< 200 Hz (5 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_1_KHZ   = 0x6, /*!< 1 KHz (1 ms)*/
	ICM406XX_ACCEL_CONFIG0_ODR_2_KHZ   = 0x5, /*!< 2 KHz (500 us)*/
	ICM406XX_ACCEL_CONFIG0_ODR_4_KHZ   = 0x4, /*!< 4 KHz (250 us)*/
	ICM406XX_ACCEL_CONFIG0_ODR_8_KHZ   = 0x3, /*!< 8 KHz (125 us)*/
	ICM406XX_ACCEL_CONFIG0_ODR_16_KHZ  = 0x2, /*!< 16 KHz (62.5 us)*/
	ICM406XX_ACCEL_CONFIG0_ODR_32_KHZ  = 0x1, /*!< 32 KHz (31.25 us)*/
	ICM406XX_ACCEL_CONFIG0_ODR_64_KHZ  = 0x0, /*!< 64 KHz (15.625 us)*/
} ICM406XX_ACCEL_CONFIG0_ODR_t;
#endif

int inv_icm406xx_wr_accel_config0_odr(struct inv_icm406xx *        s,
                                      ICM406XX_ACCEL_CONFIG0_ODR_t new_value);
int inv_icm406xx_rd_accel_config0_odr(struct inv_icm406xx *s, ICM406XX_ACCEL_CONFIG0_ODR_t *value);

/*
 * MPUREG_GYRO_CONFIG1
 * Register Name: GYRO_CONFIG1
 */

/* TEMP_FILT_BW */
#define BIT_GYRO_CONFIG1_TEMP_FILT_BW_POS  5
#define BIT_GYRO_CONFIG1_TEMP_FILT_BW_MASK (0x7 << BIT_GYRO_CONFIG1_TEMP_FILT_BW_POS)

/* AVG_FILT_RATE */
#define BIT_GYRO_CONFIG1_AVG_FILT_RATE_POS  4
#define BIT_GYRO_CONFIG1_AVG_FILT_RATE_MASK (0x1 << BIT_GYRO_CONFIG1_AVG_FILT_RATE_POS)

typedef enum {
	ICM406XX_GYRO_CONFIG1_AVG_FILT_RATE_8KHz = (0x01 << BIT_GYRO_CONFIG1_AVG_FILT_RATE_POS),
	ICM406XX_GYRO_CONFIG1_AVG_FILT_RATE_1KHz = (0x00 << BIT_GYRO_CONFIG1_AVG_FILT_RATE_POS),
} ICM406XX_GYRO_CONFIG1_AVG_FILT_RATE_t;

int inv_icm406xx_wr_gyro_config1_avg_filt_rate(struct inv_icm406xx *                 s,
                                               ICM406XX_GYRO_CONFIG1_AVG_FILT_RATE_t new_value);

/* GYRO_UI_FILT_ORD */
#define BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_POS  2
#define BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_MASK (0x3 << BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_POS)

/* GYRO_DEC2_M2_ORD */
#define BIT_GYRO_CONFIG1_GYRO_DEC2_M2_ORD_POS  0
#define BIT_GYRO_CONFIG1_GYRO_DEC2_M2_ORD_MASK (0x3 << BIT_GYRO_CONFIG1_GYRO_DEC2_M2_ORD_POS)

/*
 * MPUREG_ACCEL_GYRO_CONFIG0
 * Register Name: GYRO_ACCEL_CONFIG0
 */

/* ACCEL_FILT */
#define BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS  4
#define BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK (0xF << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS)

typedef enum {
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_40 = (0x7 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_20 = (0x6 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_16 = (0x5 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_10 = (0x4 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_8  = (0x3 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_5  = (0x2 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_4  = (0x1 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_2  = (0x0 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
} ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_t;

int inv_icm406xx_wr_gyro_accel_config0_accel_filt_bw(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_t new_value);
int inv_icm406xx_rd_gyro_accel_config0_accel_filt_bw(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_t *value);

typedef enum {
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_128 = (0x9 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_64  = (0x8 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_32  = (0x7 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_16  = (0x6 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_8   = (0x5 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_4   = (0x4 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_3   = (0x3 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_2   = (0x2 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_1   = (0x1 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
} ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_t;
int inv_icm406xx_wr_gyro_accel_config0_accel_filt_avg(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_t new_value);
int inv_icm406xx_rd_gyro_accel_config0_accel_filt_avg(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_t *value);

/* GYRO_FILT */
#define BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_POS  0
#define BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_MASK 0x0F

typedef enum {
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_40 = 0x07,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_20 = 0x06,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_16 = 0x05,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_10 = 0x04,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_8  = 0x03,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_5  = 0x02,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_4  = 0x01,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_2  = 0x00,
} ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_t;
int inv_icm406xx_wr_gyro_accel_config0_gyro_filt_bw(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_t new_value);
int inv_icm406xx_rd_gyro_accel_config0_gyro_filt_bw(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_t *value);

typedef enum {
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_AVG_128 = 0x09,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_AVG_64  = 0x08,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_AVG_32  = 0x07,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_AVG_16  = 0x06,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_AVG_8   = 0x05,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_AVG_4   = 0x04,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_AVG_3   = 0x03,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_AVG_2   = 0x02,
	ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_AVG_1   = 0x01,
} ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_AVG_t;

int inv_icm406xx_wr_gyro_accel_config0_gyro_filt_avg(
    struct inv_icm406xx *s, ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_AVG_t new_value);

/*
 * MPUREG_ACCEL_CONFIG1
 * Register Name: ACCEL_CONFIG1
 */

/* ACCEL_UI_FILT_ORD */
#define BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_POS  3
#define BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_MASK (0x3 << BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_POS)

/* ACCEL_DEC2_M2_ORD */
#define BIT_ACCEL_CONFIG1_ACCEL_DEC2_M2_ORD_POS  1
#define BIT_ACCEL_CONFIG1_ACCEL_DEC2_M2_ORD_MASK (0x3 << BIT_ACCEL_CONFIG1_ACCEL_DEC2_M2_ORD_POS)

/* AVG_FILT_RATE */
#define BIT_ACCEL_CONFIG1_AVG_FILT_RATE_POS  0
#define BIT_ACCEL_CONFIG1_AVG_FILT_RATE_MASK 0x1

typedef enum {
	ICM406XX_ACCEL_CONFIG1_AVG_FILT_RATE_8KHz = 0x01,
	ICM406XX_ACCEL_CONFIG1_AVG_FILT_RATE_1KHz = 0x00,
} ICM406XX_ACCEL_CONFIG1_AVG_FILT_RATE_t;

int inv_icm406xx_wr_accel_config1_avg_filt_rate(struct inv_icm406xx *                  s,
                                                ICM406XX_ACCEL_CONFIG1_AVG_FILT_RATE_t new_value);
int inv_icm406xx_rd_accel_config1_avg_filt_rate(struct inv_icm406xx *                   s,
                                                ICM406XX_ACCEL_CONFIG1_AVG_FILT_RATE_t *value);

/*
 * MPUREG_ACCEL_WOM_X_THR
 * Register Name: ACCEL_WOM_X_THR
 */
int inv_icm406xx_wr_accel_wom_x_thr(struct inv_icm406xx *s, uint8_t new_value);

/*
 * MPUREG_ACCEL_WOM_Y_THR
 * Register Name: ACCEL_WOM_Y_THR
 */
int inv_icm406xx_wr_accel_wom_y_thr(struct inv_icm406xx *s, uint8_t new_value);

/* 
 * MPUREG_ACCEL_WOM_Z_THR
 * Register Name: ACCEL_WOM_Z_THR
 */
int inv_icm406xx_wr_accel_wom_z_thr(struct inv_icm406xx *s, uint8_t new_value);

/*
 * MPUREG_SMD_CONFIG
 * Register Name: SMD_CONFIG
 */

/* WOM_INT_MODE */
#define BIT_SMD_CONFIG_WOM_INT_MODE_POS  3
#define BIT_SMD_CONFIG_WOM_INT_MODE_MASK (0x1 << BIT_SMD_CONFIG_WOM_INT_MODE_POS)

typedef enum {
	ICM406XX_SMD_CONFIG_WOM_INT_MODE_ANDED = (0x01 << BIT_SMD_CONFIG_WOM_INT_MODE_POS),
	ICM406XX_SMD_CONFIG_WOM_INT_MODE_ORED  = (0x00 << BIT_SMD_CONFIG_WOM_INT_MODE_POS),
} ICM406XX_SMD_CONFIG_WOM_INT_MODE_t;

int inv_icm406xx_wr_smd_config_wom_int_mode(struct inv_icm406xx *              s,
                                            ICM406XX_SMD_CONFIG_WOM_INT_MODE_t new_value);
int inv_icm406xx_rd_smd_config_wom_int_mode(struct inv_icm406xx *               s,
                                            ICM406XX_SMD_CONFIG_WOM_INT_MODE_t *value);

/* WOM_MODE */
#define BIT_SMD_CONFIG_WOM_MODE_POS  2
#define BIT_SMD_CONFIG_WOM_MODE_MASK (0x1 << BIT_SMD_CONFIG_WOM_MODE_POS)

typedef enum {
	ICM406XX_SMD_CONFIG_WOM_MODE_CMP_PREV = (0x01 << BIT_SMD_CONFIG_WOM_MODE_POS),
	ICM406XX_SMD_CONFIG_WOM_MODE_CMP_INIT = (0x00 << BIT_SMD_CONFIG_WOM_MODE_POS),
} ICM406XX_SMD_CONFIG_WOM_MODE_t;

int inv_icm406xx_wr_smd_config_wom_mode(struct inv_icm406xx *          s,
                                        ICM406XX_SMD_CONFIG_WOM_MODE_t new_value);
int inv_icm406xx_rd_smd_config_wom_mode(struct inv_icm406xx *           s,
                                        ICM406XX_SMD_CONFIG_WOM_MODE_t *value);

/* SMD_MODE */
#define BIT_SMD_CONFIG_SMD_MODE_POS  0
#define BIT_SMD_CONFIG_SMD_MODE_MASK 0x3

typedef enum {
	ICM406XX_SMD_CONFIG_SMD_MODE_LONG     = 0x03,
	ICM406XX_SMD_CONFIG_SMD_MODE_SHORT    = 0x02,
	ICM406XX_SMD_CONFIG_SMD_MODE_WOM      = 0x01,
	ICM406XX_SMD_CONFIG_SMD_MODE_DISABLED = 0x00,
} ICM406XX_SMD_CONFIG_SMD_MODE_t;

int inv_icm406xx_wr_smd_config_smd_mode(struct inv_icm406xx *          s,
                                        ICM406XX_SMD_CONFIG_SMD_MODE_t new_value);
int inv_icm406xx_rd_smd_config_smd_mode(struct inv_icm406xx *           s,
                                        ICM406XX_SMD_CONFIG_SMD_MODE_t *value);

/*
 * MPUREG_INT_STATUS2
 * Register Name: INT_STATUS2
 */
#define BIT_INT_STATUS2_SMD_INT   0x08
#define BIT_INT_STATUS2_WOM_Z_INT 0x04
#define BIT_INT_STATUS2_WOM_Y_INT 0x02
#define BIT_INT_STATUS2_WOM_X_INT 0x01
int inv_icm406xx_rd_int_status2(struct inv_icm406xx *s, uint8_t *value);

/*
 * MPUREG_TMST_CONFIG
 * Register Name: TMST_CONFIG
 */

/* TMST_TO_REGS */
#define BIT_TMST_CONFIG_TMST_TO_REGS_EN_POS  4
#define BIT_TMST_CONFIG_TMST_TO_REGS_EN_MASK (0x1 << BIT_TMST_CONFIG_TMST_TO_REGS_EN_POS)

typedef enum {
	ICM406XX_TMST_CONFIG_TMST_TO_REGS_EN  = (0x1 << BIT_TMST_CONFIG_TMST_TO_REGS_EN_POS),
	ICM406XX_TMST_CONFIG_TMST_TO_REGS_DIS = (0x0 << BIT_TMST_CONFIG_TMST_TO_REGS_EN_POS),
} ICM406XX_TMST_CONFIG_TMST_TO_REGS_EN_t;

int inv_icm406xx_wr_tmst_config_tmst_to_reg(struct inv_icm406xx *                  s,
                                            ICM406XX_TMST_CONFIG_TMST_TO_REGS_EN_t new_value);

/* TMST_RES */
#define BIT_TMST_CONFIG_RESOL_POS  3
#define BIT_TMST_CONFIG_RESOL_MASK (0x1 << BIT_TMST_CONFIG_RESOL_POS)

typedef enum {
	ICM406XX_TMST_CONFIG_RESOL_16us = (0x01 << BIT_TMST_CONFIG_RESOL_POS),
	ICM406XX_TMST_CONFIG_RESOL_1us  = (0x00 << BIT_TMST_CONFIG_RESOL_POS),
} ICM406XX_TMST_CONFIG_RESOL_t;

int inv_icm406xx_wr_tmst_config_resolution(struct inv_icm406xx *        s,
                                           ICM406XX_TMST_CONFIG_RESOL_t new_value);

/* TMST_FSYNC */
#define BIT_TMST_CONFIG_TMST_FSYNC_POS  1
#define BIT_TMST_CONFIG_TMST_FSYNC_MASK (0x1 << BIT_TMST_CONFIG_TMST_FSYNC_POS)

typedef enum {
	ICM406XX_TMST_CONFIG_TMST_FSYNC_EN  = (0x01 << BIT_TMST_CONFIG_TMST_FSYNC_POS),
	ICM406XX_TMST_CONFIG_TMST_FSYNC_DIS = (0x00 << BIT_TMST_CONFIG_TMST_FSYNC_POS),
} ICM406XX_TMST_CONFIG_TMST_FSYNC_EN_t;

int inv_icm406xx_wr_tmst_config_fsync_en(struct inv_icm406xx *                s,
                                         ICM406XX_TMST_CONFIG_TMST_FSYNC_EN_t new_value);
int inv_icm406xx_rd_tmst_config_fsync_en(struct inv_icm406xx *                 s,
                                         ICM406XX_TMST_CONFIG_TMST_FSYNC_EN_t *value);

/* TMST_EN */
#define BIT_TMST_CONFIG_TMST_EN_POS  0
#define BIT_TMST_CONFIG_TMST_EN_MASK 0x1

typedef enum {
	ICM406XX_TMST_CONFIG_TMST_EN  = 0x01,
	ICM406XX_TMST_CONFIG_TMST_DIS = 0x00,
} ICM406XX_TMST_CONFIG_TMST_EN_t;

int inv_icm406xx_wr_tmst_config_en(struct inv_icm406xx *          s,
                                   ICM406XX_TMST_CONFIG_TMST_EN_t new_value);

/*
 * MPUREG_FIFO_CONFIG1
 * Register Name: FIFO_CONFIG1
 */

/* FIFO_WM_GT_TH */
#define BIT_FIFO_CONFIG1_WM_GT_TH_POS  5
#define BIT_FIFO_CONFIG1_WM_GT_TH_MASK (0x1 << BIT_FIFO_CONFIG1_WM_GT_TH_POS)

typedef enum {
	ICM406XX_FIFO_CONFIG1_WM_GT_TH_EN  = (0x1 << BIT_FIFO_CONFIG1_WM_GT_TH_POS),
	ICM406XX_FIFO_CONFIG1_WM_GT_TH_DIS = (0x0 << BIT_FIFO_CONFIG1_WM_GT_TH_POS),
} ICM406XX_FIFO_CONFIG1_WM_GT_t;

int inv_icm406xx_wr_fifo_config1_wm_gt_th(struct inv_icm406xx *         s,
                                          ICM406XX_FIFO_CONFIG1_WM_GT_t new_value);

/* FIFO_HIRES_EN */
#define BIT_FIFO_CONFIG1_HIRES_POS  4
#define BIT_FIFO_CONFIG1_HIRES_MASK (0x1 << BIT_FIFO_CONFIG1_HIRES_POS)

typedef enum {
	ICM406XX_FIFO_CONFIG1_HIRES_EN  = (0x1 << BIT_FIFO_CONFIG1_HIRES_POS),
	ICM406XX_FIFO_CONFIG1_HIRES_DIS = (0x0 << BIT_FIFO_CONFIG1_HIRES_POS),
} ICM406XX_FIFO_CONFIG1_HIRES_t;

int inv_icm406xx_wr_fifo_config1_hires(struct inv_icm406xx *         s,
                                       ICM406XX_FIFO_CONFIG1_HIRES_t new_value);

/* FIFO_TMST_FSYNC_EN */
#define BIT_FIFO_CONFIG1_TMST_FSYNC_POS  3
#define BIT_FIFO_CONFIG1_TMST_FSYNC_MASK (0x1 << BIT_FIFO_CONFIG1_TMST_FSYNC_POS)

typedef enum {
	ICM406XX_FIFO_CONFIG1_TMST_FSYNC_EN  = (0x1 << BIT_FIFO_CONFIG1_TMST_FSYNC_POS),
	ICM406XX_FIFO_CONFIG1_TMST_FSYNC_DIS = (0x0 << BIT_FIFO_CONFIG1_TMST_FSYNC_POS),
} ICM406XX_FIFO_CONFIG1_TMST_FSYNC_t;

int inv_icm406xx_wr_fifo_config1_tmst_fsync(struct inv_icm406xx *              s,
                                            ICM406XX_FIFO_CONFIG1_TMST_FSYNC_t new_value);

/* FIFO_TEMP_EN */
#define BIT_FIFO_CONFIG1_TEMP_POS  2
#define BIT_FIFO_CONFIG1_TEMP_MASK (0x1 << BIT_FIFO_CONFIG1_TEMP_POS)

typedef enum {
	ICM406XX_FIFO_CONFIG1_TEMP_EN  = (0x1 << BIT_FIFO_CONFIG1_TEMP_POS),
	ICM406XX_FIFO_CONFIG1_TEMP_DIS = (0x0 << BIT_FIFO_CONFIG1_TEMP_POS),
} ICM406XX_FIFO_CONFIG1_TEMP_t;

int inv_icm406xx_wr_fifo_config1_temp_en(struct inv_icm406xx *        s,
                                         ICM406XX_FIFO_CONFIG1_TEMP_t new_value);

/* FIFO_GYRO_EN */
#define BIT_FIFO_CONFIG1_GYRO_POS  1
#define BIT_FIFO_CONFIG1_GYRO_MASK (0x1 << BIT_FIFO_CONFIG1_GYRO_POS)

typedef enum {
	ICM406XX_FIFO_CONFIG1_GYRO_EN  = (0x1 << BIT_FIFO_CONFIG1_GYRO_POS),
	ICM406XX_FIFO_CONFIG1_GYRO_DIS = (0x0 << BIT_FIFO_CONFIG1_GYRO_POS),
} ICM406XX_FIFO_CONFIG1_GYRO_t;

int inv_icm406xx_wr_fifo_config1_gyro_en(struct inv_icm406xx *        s,
                                         ICM406XX_FIFO_CONFIG1_GYRO_t new_value);

/* FIFO_ACCEL_EN*/
#define BIT_FIFO_CONFIG1_ACCEL_POS  0
#define BIT_FIFO_CONFIG1_ACCEL_MASK 0x1

typedef enum {
	ICM406XX_FIFO_CONFIG1_ACCEL_EN  = 0x01,
	ICM406XX_FIFO_CONFIG1_ACCEL_DIS = 0x00,
} ICM406XX_FIFO_CONFIG1_ACCEL_t;

int inv_icm406xx_wr_fifo_config1_accel_en(struct inv_icm406xx *         s,
                                          ICM406XX_FIFO_CONFIG1_ACCEL_t new_value);

/*
 * MPUREG_FIFO_CONFIG2
 * Register Name: FIFO_CONFIG2
 */
int inv_icm406xx_wr_fifo_config2(struct inv_icm406xx *s, uint8_t new_value);

/*
 * MPUREG_FSYNC_CONFIG
 * Register Name: FSYNC_CONFIG
 */

/* FSYNC_UI_SEL */
#define BIT_FSYNC_CONFIG_UI_SEL_POS  4
#define BIT_FSYNC_CONFIG_UI_SEL_MASK (0x7 << BIT_FSYNC_CONFIG_UI_SEL_POS)

typedef enum {
	ICM406XX_FSYNC_CONFIG_UI_SEL_NO      = (0x0 << BIT_FSYNC_CONFIG_UI_SEL_POS),
	ICM406XX_FSYNC_CONFIG_UI_SEL_TEMP    = (0x1 << BIT_FSYNC_CONFIG_UI_SEL_POS),
	ICM406XX_FSYNC_CONFIG_UI_SEL_GYRO_X  = (0x2 << BIT_FSYNC_CONFIG_UI_SEL_POS),
	ICM406XX_FSYNC_CONFIG_UI_SEL_GYRO_Y  = (0x3 << BIT_FSYNC_CONFIG_UI_SEL_POS),
	ICM406XX_FSYNC_CONFIG_UI_SEL_GYRO_Z  = (0x4 << BIT_FSYNC_CONFIG_UI_SEL_POS),
	ICM406XX_FSYNC_CONFIG_UI_SEL_ACCEL_X = (0x5 << BIT_FSYNC_CONFIG_UI_SEL_POS),
	ICM406XX_FSYNC_CONFIG_UI_SEL_ACCEL_Y = (0x6 << BIT_FSYNC_CONFIG_UI_SEL_POS),
	ICM406XX_FSYNC_CONFIG_UI_SEL_ACCEL_Z = (0x7 << BIT_FSYNC_CONFIG_UI_SEL_POS),
} ICM406XX_FSYNC_CONFIG_UI_SEL_t;

int inv_icm406xx_fsync_config_ui_sel(struct inv_icm406xx *          s,
                                     ICM406XX_FSYNC_CONFIG_UI_SEL_t new_value);

/*
 * MPUREG_INT_CONFIG1
 * Register Name: INT_CONFIG1
 */

/* ASY_RESET_DISABLE */
#define BIT_INT_CONFIG1_ASY_RST_POS  4
#define BIT_INT_CONFIG1_ASY_RST_MASK (0x1 << BIT_INT_CONFIG1_ASY_RST_POS)

typedef enum {
	ICM406XX_INT_CONFIG1_ASY_RST_DISABLED = (0x1 << BIT_INT_CONFIG1_ASY_RST_POS),
	ICM406XX_INT_CONFIG1_ASY_RST_ENABLED  = (0x0 << BIT_INT_CONFIG1_ASY_RST_POS),
} ICM406XX_INT_CONFIG1_ASY_RST_t;

int inv_icm406xx_wr_int_config1_asy_rst_dis(struct inv_icm406xx *          s,
                                            ICM406XX_INT_CONFIG1_ASY_RST_t new_value);

/*
 * MPUREG_INT_SOURCE0
 * Register Name: INT_SOURCE0
 */
#define BIT_INT_SOURCE0_UI_FSYNC_INT1_EN   0x40
#define BIT_INT_SOURCE0_PLL_RDY_INT1_EN    0x20
#define BIT_INT_SOURCE0_RESET_DONE_INT1_EN 0x10
#define BIT_INT_SOURCE0_UI_DRDY_INT1_EN    0x08
#define BIT_INT_SOURCE0_FIFO_THS_INT1_EN   0x04
#define BIT_INT_SOURCE0_FIFO_FULL_INT1_EN  0x02
#define BIT_INT_SOURCE0_UI_AGC_RDY_INT1_EN 0x01
int inv_icm406xx_wr_int_source0(struct inv_icm406xx *s, uint8_t new_value);
int inv_icm406xx_rd_int_source0(struct inv_icm406xx *s, uint8_t *value);

/*
 * MPUREG_INT_SOURCE1
 * Register Name: INT_SOURCE1
 */
#define BIT_INT_SOURCE1_SMD_INT1_EN   0x08
#define BIT_INT_SOURCE1_WOM_Z_INT1_EN 0x04
#define BIT_INT_SOURCE1_WOM_Y_INT1_EN 0x02
#define BIT_INT_SOURCE1_WOM_X_INT1_EN 0x01
int inv_icm406xx_wr_int_source1(struct inv_icm406xx *s, uint8_t new_value);
int inv_icm406xx_rd_int_source1(struct inv_icm406xx *s, uint8_t *value);

/*
 * MPUREG_INT_SOURCE2
 * Register Name: INT_SOURCE2
 */
#define BIT_INT_SOURCE2_OIS2_AGC_RDY_INT1_EN 0x20
#define BIT_INT_SOURCE2_OIS2_FSYNC_INT1_EN   0x10
#define BIT_INT_SOURCE2_OIS2_DRDY_INT1_EN    0x08
#define BIT_INT_SOURCE2_OIS1_AGC_RDY_INT1_EN 0x04
#define BIT_INT_SOURCE2_OIS1_FSYNC_INT1_EN   0x02
#define BIT_INT_SOURCE2_OIS1_DRDY_INT1_EN    0x01

/*
 * MPUREG_INT_SOURCE3
 * Register Name: INT_SOURCE3
 */
#define BIT_INT_SOURCE3_UI_FSYNC_INT2_EN   0x40
#define BIT_INT_SOURCE3_PLL_RDY_INT2_EN    0x20
#define BIT_INT_SOURCE3_RESET_DONE_INT2_EN 0x10
#define BIT_INT_SOURCE3_UI_DRDY_INT2_EN    0x08
#define BIT_INT_SOURCE3_FIFO_THS_INT2_EN   0x04
#define BIT_INT_SOURCE3_FIFO_FULL_INT2_EN  0x02
#define BIT_INT_SOURCE3_UI_AGC_RDY_INT2_EN 0x01

/*
 * MPUREG_INT_SOURCE4
 * Register Name: INT_SOURCE4
 */
#define BIT_INT_SOURCE4_SMD_INT2_EN   0x08
#define BIT_INT_SOURCE4_WOM_Z_INT2_EN 0x04
#define BIT_INT_SOURCE4_WOM_Y_INT2_EN 0x02
#define BIT_INT_SOURCE4_WOM_X_INT2_EN 0x01

/*
 * MPUREG_INT_SOURCE5
 * Register Name: INT_SOURCE5
 */
#define BIT_INT_SOURCE5_OIS2_AGC_RDY_INT2_EN 0x20
#define BIT_INT_SOURCE5_OIS2_FSYNC_INT2_EN   0x10
#define BIT_INT_SOURCE5_OIS2_DRDY_INT2_EN    0x08
#define BIT_INT_SOURCE5_OIS1_AGC_RDY_INT2_EN 0x04
#define BIT_INT_SOURCE5_OIS1_FSYNC_INT2_EN   0x02
#define BIT_INT_SOURCE5_OIS1_DRDY_INT2_EN    0x01

/*
 * MPUREG_SELF_TEST_CONFIG
 * Register Name: SELF_TEST_CONFIG
*/
#define BIT_ST_REGULATOR_EN 0x40
#define BIT_ACCEL_Z_ST_EN   0x20
#define BIT_ACCEL_Y_ST_EN   0x10
#define BIT_ACCEL_X_ST_EN   0x08
#define BIT_GYRO_Z_ST_EN    0x04
#define BIT_GYRO_Y_ST_EN    0x02
#define BIT_GYRO_X_ST_EN    0x01

/*
 * MPUREG_WHO_AM_I
 * Register Name: WHO_AM_I
 */
int inv_icm406xx_rd_who_am_i(struct inv_icm406xx *s, uint8_t *value);

/*
 * MPUREG_REG_BANK_SEL
 * Register Name: REG_BANK_SEL
 */
int inv_icm406xx_wr_reg_bank_sel(struct inv_icm406xx *s, uint8_t new_value);

/* ----------------------------------------------------------------------------
 * Register bank 1
 * ---------------------------------------------------------------------------- */

/*
 * MPUREG_TMST_VAL0_B1
 * MPUREG_TMST_VAL1_B1
 * MPUREG_TMST_VAL2_B1
 *
 * Register Name: TMST_VAL0_B1
 * Register Name: TMST_VAL1_B1
 * Register Name: TMST_VAL2_B1
 */
int inv_icm406xx_rd_tmst_val(struct inv_icm406xx *s, uint8_t value[3]);

/*
 * MPUREG_INTF_CONFIG4_B1
 * Register Name: INTF_CONFIG4_B1
 */

/* SPI_AUX1_4WIRE */
#define BIT_INTF_CONFIG4_AUX_SPI_POS  2
#define BIT_INTF_CONFIG4_AUX_SPI_MASK (0x1 << BIT_INTF_CONFIG4_AUX_SPI_POS)

typedef enum {
	ICM406XX_INTF_CONFIG4_AUX_SPI4W = (0x1 << BIT_INTF_CONFIG4_AUX_SPI_POS),
	ICM406XX_INTF_CONFIG4_AUX_SPI3W = (0x0 << BIT_INTF_CONFIG4_AUX_SPI_POS),
} ICM406XX_INTF_CONFIG4_AUX_SPI_t;

int inv_icm406xx_wr_intf_config4_aux_spi(struct inv_icm406xx *           s,
                                         ICM406XX_INTF_CONFIG4_AUX_SPI_t new_value);
int inv_icm406xx_rd_intf_config4_aux_spi(struct inv_icm406xx *            s,
                                         ICM406XX_INTF_CONFIG4_AUX_SPI_t *value);

/* SPI_AP_4WIRE */
#define BIT_INTF_CONFIG4_AP_SPI_POS  1
#define BIT_INTF_CONFIG4_AP_SPI_MASK (0x1 << BIT_INTF_CONFIG4_AP_SPI_POS)

typedef enum {
	ICM406XX_INTF_CONFIG4_AP_SPI4W = (0x1 << BIT_INTF_CONFIG4_AP_SPI_POS),
	ICM406XX_INTF_CONFIG4_AP_SPI3W = (0x0 << BIT_INTF_CONFIG4_AP_SPI_POS),
} ICM406XX_INTF_CONFIG4_AP_SPI_t;

int inv_icm406xx_wr_intf_config4_ap_spi(struct inv_icm406xx *          s,
                                        ICM406XX_INTF_CONFIG4_AP_SPI_t new_value);
int inv_icm406xx_rd_intf_config4_ap_spi(struct inv_icm406xx *           s,
                                        ICM406XX_INTF_CONFIG4_AP_SPI_t *value);

/* ----------------------------------------------------------------------------
 * Register bank 2
 * ---------------------------------------------------------------------------- */

/*
 * MPUREG_AUX1_CONFIG1_B2
 * Register Name: AUX1_CONFIG1
 */

/* AUX1_DEC */
#define BIT_OIS1_CONFIG1_DEC_POS  2
#define BIT_OIS1_CONFIG1_DEC_MASK (0x7 << BIT_OIS1_CONFIG1_DEC_POS)

typedef enum {
	ICM406XX_OIS1_CONFIG1_DEC_1  = (0x0 << BIT_OIS1_CONFIG1_DEC_POS),
	ICM406XX_OIS1_CONFIG1_DEC_2  = (0x1 << BIT_OIS1_CONFIG1_DEC_POS),
	ICM406XX_OIS1_CONFIG1_DEC_4  = (0x2 << BIT_OIS1_CONFIG1_DEC_POS),
	ICM406XX_OIS1_CONFIG1_DEC_8  = (0x3 << BIT_OIS1_CONFIG1_DEC_POS),
	ICM406XX_OIS1_CONFIG1_DEC_16 = (0x4 << BIT_OIS1_CONFIG1_DEC_POS),
	ICM406XX_OIS1_CONFIG1_DEC_32 = (0x5 << BIT_OIS1_CONFIG1_DEC_POS),
} ICM406XX_OIS1_CONFIG1_DEC_t;

int inv_icm406xx_wr_ois1_config1_dec(struct inv_icm406xx *s, ICM406XX_OIS1_CONFIG1_DEC_t new_value);

/* GYRO_AUX1_EN */
#define BIT_OIS1_CONFIG1_GYRO_EN_POS  1
#define BIT_OIS1_CONFIG1_GYRO_EN_MASK (0x1 << BIT_OIS1_CONFIG1_GYRO_EN_POS)

typedef enum {
	ICM406XX_OIS1_CONFIG1_GYRO_EN  = (0x1 << BIT_OIS1_CONFIG1_GYRO_EN_POS),
	ICM406XX_OIS1_CONFIG1_GYRO_DIS = (0x0 << BIT_OIS1_CONFIG1_GYRO_EN_POS),
} ICM406XX_OIS1_CONFIG1_GYRO_EN_t;

int inv_icm406xx_wr_ois1_config1_gyro_en(struct inv_icm406xx *           s,
                                         ICM406XX_OIS1_CONFIG1_GYRO_EN_t new_value);

/*
 * MPUREG_AUX1_CONFIG2_B2
 * Register Name: AUX1_CONFIG2
 */

/* GYRO_AUX1_FS_SEL */
#define BIT_OIS1_CONFIG2_GYRO_FS_SEL_POS  3
#define BIT_OIS1_CONFIG2_GYRO_FS_SEL_MASK (0x7 << BIT_OIS1_CONFIG2_GYRO_FS_SEL_POS)

typedef enum {
	ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_16dps   = (0x7 << BIT_OIS1_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_31dps   = (0x6 << BIT_OIS1_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_62dps   = (0x5 << BIT_OIS1_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_125dps  = (0x4 << BIT_OIS1_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_250dps  = (0x3 << BIT_OIS1_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_500dps  = (0x2 << BIT_OIS1_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_1000dps = (0x1 << BIT_OIS1_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_2000dps = (0x0 << BIT_OIS1_CONFIG2_GYRO_FS_SEL_POS),
} ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_t;

int inv_icm406xx_wr_ois1_config2_gyro_fs_sel(struct inv_icm406xx *               s,
                                             ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_t new_value);
int inv_icm406xx_rd_ois1_config2_gyro_fs_sel(struct inv_icm406xx *                s,
                                             ICM406XX_OIS1_CONFIG2_GYRO_FS_SEL_t *value);

/*
 * MPUREG_GYRO_DATA_X0_AUX1_B2
 * MPUREG_GYRO_DATA_Y0_AUX1_B2
 * MPUREG_GYRO_DATA_Z0_AUX1_B2
 * Register Name: GYRO_DATA_X0_AUX1
 * Register Name: GYRO_DATA_Y0_AUX1
 * Register Name: GYRO_DATA_Z0_AUX1
 */
int inv_icm406xx_rd_gyro_data_ois1(struct inv_icm406xx *s, uint8_t value[GYRO_DATA_SIZE]);

/*
 * MPUREG_INT_STATUS_AUX1_B2
 * Register Name: INT_STATUS_AUX1
 */
#define BIT_INT_STATUS_AUX1_FSYNC   0x04
#define BIT_INT_STATUS_AUX1_DRDY    0x02
#define BIT_INT_STATUS_AUX1_AGC_RDY 0x01
int inv_icm406xx_rd_int_status_aux1(struct inv_icm406xx *s, uint8_t *value);

/*
 * MPUREG_AUX2_CONFIG1_B2
 * Register Name: AUX2_CONFIG1
 */

/* AUX2_DEC */
#define BIT_OIS2_CONFIG1_DEC_POS  2
#define BIT_OIS2_CONFIG1_DEC_MASK (0x7 << BIT_OIS2_CONFIG1_DEC_POS)

typedef enum {
	ICM406XX_OIS2_CONFIG1_DEC_1  = (0x0 << BIT_OIS2_CONFIG1_DEC_POS),
	ICM406XX_OIS2_CONFIG1_DEC_2  = (0x1 << BIT_OIS2_CONFIG1_DEC_POS),
	ICM406XX_OIS2_CONFIG1_DEC_4  = (0x2 << BIT_OIS2_CONFIG1_DEC_POS),
	ICM406XX_OIS2_CONFIG1_DEC_8  = (0x3 << BIT_OIS2_CONFIG1_DEC_POS),
	ICM406XX_OIS2_CONFIG1_DEC_16 = (0x4 << BIT_OIS2_CONFIG1_DEC_POS),
	ICM406XX_OIS2_CONFIG1_DEC_32 = (0x5 << BIT_OIS2_CONFIG1_DEC_POS),
} ICM406XX_OIS2_CONFIG1_DEC_t;

int inv_icm406xx_wr_ois2_config1_dec(struct inv_icm406xx *s, ICM406XX_OIS2_CONFIG1_DEC_t new_value);

/* GYRO_AUX2_EN */
#define BIT_OIS2_CONFIG1_GYRO_EN_POS  1
#define BIT_OIS2_CONFIG1_GYRO_EN_MASK (0x1 << BIT_OIS2_CONFIG1_GYRO_EN_POS)

typedef enum {
	ICM406XX_OIS2_CONFIG1_GYRO_EN  = (0x1 << BIT_OIS2_CONFIG1_GYRO_EN_POS),
	ICM406XX_OIS2_CONFIG1_GYRO_DIS = (0x0 << BIT_OIS2_CONFIG1_GYRO_EN_POS),
} ICM406XX_OIS2_CONFIG1_GYRO_EN_t;

int inv_icm406xx_wr_ois2_config1_gyro_en(struct inv_icm406xx *           s,
                                         ICM406XX_OIS2_CONFIG1_GYRO_EN_t new_value);

/*
 * MPUREG_AUX2_CONFIG2_B2
 * Register Name: AUX2_CONFIG2
 */

/* GYRO_AUX2_FS_SEL */
#define BIT_OIS2_CONFIG2_GYRO_FS_SEL_POS  3
#define BIT_OIS2_CONFIG2_GYRO_FS_SEL_MASK (0x7 << BIT_OIS2_CONFIG2_GYRO_FS_SEL_POS)

typedef enum {
	ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_16dps   = (0x7 << BIT_OIS2_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_31dps   = (0x6 << BIT_OIS2_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_62dps   = (0x5 << BIT_OIS2_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_125dps  = (0x4 << BIT_OIS2_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_250dps  = (0x3 << BIT_OIS2_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_500dps  = (0x2 << BIT_OIS2_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_1000dps = (0x1 << BIT_OIS2_CONFIG2_GYRO_FS_SEL_POS),
	ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_2000dps = (0x0 << BIT_OIS2_CONFIG2_GYRO_FS_SEL_POS),
} ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_t;

int inv_icm406xx_wr_ois2_config2_gyro_fs_sel(struct inv_icm406xx *               s,
                                             ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_t new_value);
int inv_icm406xx_rd_ois2_config2_gyro_fs_sel(struct inv_icm406xx *                s,
                                             ICM406XX_OIS2_CONFIG2_GYRO_FS_SEL_t *value);

/*
 * MPUREG_GYRO_DATA_X0_AUX2_B2
 * MPUREG_GYRO_DATA_Y0_AUX2_B2
 * MPUREG_GYRO_DATA_Z0_AUX2_B2
 * Register Name: GYRO_DATA_X0_AUX2
 * Register Name: GYRO_DATA_Y0_AUX2
 * Register Name: GYRO_DATA_Z0_AUX2
 */
int inv_icm406xx_rd_gyro_data_ois2(struct inv_icm406xx *s, uint8_t value[GYRO_DATA_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _INV_ICM406XX_DEFS_H_ */
