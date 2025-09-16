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

/** @defgroup DriverIcm406xxTransport Icm406xx driver transport
 *  @brief    Low-level Icm406xx register access
 *  @ingroup  DriverIcm406xx
 *  @{
 */

/** @file Icm406xxTransport.h
 * Low-level Icm406xx register access
 */

#ifndef _INV_ICM406XX_TRANSPORT_H_
#define _INV_ICM406XX_TRANSPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* forward declaration */
struct inv_icm406xx;

/** @brief enumeration  of serial interfaces available on icm406xx */
typedef enum {
	ICM406XX_UI_I2C,
	ICM406XX_UI_SPI4,
	ICM406XX_UI_SPI3,
	ICM406XX_AUX1_SPI3,
	ICM406XX_AUX2_SPI3

} ICM406XX_SERIAL_IF_TYPE_t;

/** @brief basesensor serial interface
 */
struct inv_icm406xx_serif {
	void *context;
	int (*read_reg)(void *context, uint8_t reg, uint8_t *buf, uint32_t len);
	int (*write_reg)(void *context, uint8_t reg, const uint8_t *buf, uint32_t len);
	uint32_t                  max_read;
	uint32_t                  max_write;
	ICM406XX_SERIAL_IF_TYPE_t serif_type;
};

/** @brief Reads data from a register on Icm406xx.
 * @param[in] reg    register address to be read
 * @param[in] len    number of byte to be read
 * @param[out] buf   output data from the register
 * @return            0 in case of success, -1 for any error
 */
int inv_icm406xx_read_reg(struct inv_icm406xx *s, uint8_t reg, uint32_t len, uint8_t *buf);

/** @brief Writes data to a register on Icm406xx.
 * @param[in] reg    register address to be written
 * @param[in] len    number of byte to be written
 * @param[in] buf    input data to write
 * @return            0 in case of success, -1 for any error
 */
int inv_icm406xx_write_reg(struct inv_icm406xx *s, uint8_t reg, uint32_t len, const uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM406XX_TRANSPORT_H_ */

/** @} */
