/*!****************************************************************************
 * @file		crc32.h
 * @author		d_el
 * @version		V1.0
 * @date		Apr 27, 2020
 * @brief
 * @copyright	Copyright (C) 2017 Storozhenko Roman
 *				All rights reserved
 *				This software may be modified and distributed under the terms
 *				of the BSD license.	 See the LICENSE file for details
 */

#ifndef crc32_H
#define crc32_H

/*!****************************************************************************
 * Include
 */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f3xx.h"

/*!****************************************************************************
 * Define
 */

/*!****************************************************************************
 * Enumeration
 */

/*!****************************************************************************
 * Typedef
 */

/*!****************************************************************************
 * Exported variables
 */

/*!****************************************************************************
 * Macro functions
 */
static inline void crc32_reset(void){
	CRC->CR = CRC_CR_RESET; // Reset CRC generator
	__DSB();
}

static inline void crc32_init(void){
	RCC->AHBENR |= RCC_AHBENR_CRCEN;
}

static inline uint32_t crc32_CalcCRC(uint32_t data){
	CRC->DR = data;
	return CRC->DR;
}

/*!****************************************************************************
 * Algorithm	Poly		Init		RefIn	RefOut	XorOut
 * CRC-32		0x04C11DB7	0xFFFFFFFF	true	true	0xFFFFFFFF
 */
static inline uint32_t crc32_posix(const void *buf, size_t len, bool clear){
	if(clear){
		CRC->POL = 0x04C11DB7;
		CRC->INIT = 0xFFFFFFFF;
		CRC->CR = CRC_CR_REV_OUT | CRC_CR_REV_IN;
		CRC->CR |= CRC_CR_RESET;
	}

	uint32_t *p = (uint32_t*) buf;

	while(len >= 4){
		CRC->DR = *p++;
		len -= 4;
	}

	return ~CRC->DR;
}

#endif //crc32_H
/******************************** END OF FILE ********************************/
