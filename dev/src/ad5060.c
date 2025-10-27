/*!****************************************************************************
 * @file		ad5060.c
 * @author		d_el
 * @version		V1.0
 * @date		Oct 23, 2025
 * @copyright	License (MIT). Copyright (c) 2025 Storozhenko Roman
 * @brief		16 bit 1 LSB INL DAC
 */

/*!****************************************************************************
 * Include
 */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <spi.h>
#include <gpio.h>
#include "ad5060.h"

/*!****************************************************************************
 * MEMORY
 */
static ad5060_spi_t spi;

/*!****************************************************************************
 * @brief
 */
static bool ad5060_writepack(uint8_t n, uint16_t val){
	uint8_t rx[3] = {};
	val = __builtin_bswap16(val);
	uint8_t tx[3] = { 0, val & 0xFF, val >> 8 };
	gppin_reset(n == 0 ? GP_NSS2 : GP_NSS3);
	bool res = spi(rx, tx, 3);
	gppin_set(n == 0 ? GP_NSS2 : GP_NSS3);
	return res;
}

/*!****************************************************************************
 * @brief
 */
void ad5060_init(ad5060_spi_t _spi){
	spi = _spi;
}

/*!****************************************************************************
 * @brief
 */
void ad5060_set_a(uint16_t val){
	ad5060_writepack(1, val);
}

/*!****************************************************************************
 * @brief
 */
void ad5060_set_b(uint16_t val){
	ad5060_writepack(0, val);
}

/******************************** END OF FILE ********************************/
