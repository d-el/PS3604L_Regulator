/*!****************************************************************************
 * @file		flash.c
 * @author		Storozhenko Roman - D_EL
 * @version		V1.1
 * @date		17.08.2023
 * @copyright	The MIT License (MIT). Copyright (c) 2023 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <string.h>
#include "stm32f3xx.h"
#include "flash.h"

/*!****************************************************************************
* Memory
*/

/*!****************************************************************************
* @brief    Unlock flash
*/
void flash_unlock(void){
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
}

/*!****************************************************************************
* @brief    Lock flash
*/
void flash_lock(void){
	FLASH->CR |= FLASH_CR_LOCK;
}

/*!****************************************************************************
 * @brief    Returned not 0 if flash busy
 */
uint32_t flash_busy(void){
	return (FLASH->SR & FLASH_SR_BSY);
}

/*!****************************************************************************
* @brief    Erase all pages
*/
void flash_eraseAllPages(void){
    FLASH->CR |= FLASH_CR_MER;
    FLASH->CR |= FLASH_CR_STRT;
    while(flash_busy());
    FLASH->CR &= FLASH_CR_MER;
}

/*!****************************************************************************
* @brief    	Erase one page
* @param[in]    addr - address allocable in page
*/
bool flash_erasePage(void *addr){
	while(flash_busy());
	FLASH->CR|= FLASH_CR_PER;
	FLASH->AR = (uint32_t)addr;
	FLASH->CR|= FLASH_CR_STRT;
	__NOP(); // Wait one CPU cycle
	while(flash_busy());
	FLASH->CR&= ~FLASH_CR_PER;
	if(FLASH->SR & FLASH_SR_EOP){
		FLASH->SR &= ~FLASH_SR_EOP;
		return true;
	}
	return false;
}

/*!****************************************************************************
* @brief    Write data
* @param    dst[in] - destination
* @param    src[in] - source
* @param    num[in] - number half word (2 byte)
*/
bool flash_write(void *dst, uint16_t *src, uint32_t num){
	while(flash_busy());
	FLASH->CR |= FLASH_CR_PG;
	while(flash_busy());

	uint16_t* pRd     = src;
	uint16_t* pWr     = dst;
	uint16_t* pEnd    = pWr + num;

	while(pWr < pEnd){
		*pWr++ = *pRd++;
		__DSB();
		while(flash_busy());
	}

	FLASH->CR &= ~FLASH_CR_PG;
	if(FLASH->SR & FLASH_SR_EOP){
		FLASH->SR &= ~FLASH_SR_EOP;
		return true;
	}
	return false;
}

/******************************** END OF FILE ********************************/
