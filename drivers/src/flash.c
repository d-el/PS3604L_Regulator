﻿/*!****************************************************************************
 * @file    	flash.c
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	07.02.2015
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
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
void flash_erasePage(void *addr){
    FLASH->CR|= FLASH_CR_PER;
    FLASH->AR = (uint32_t)addr;
    FLASH->CR|= FLASH_CR_STRT;
    while(flash_busy());
    FLASH->CR&= ~FLASH_CR_PER;
}

/*!****************************************************************************
* @brief    Write data
* @param    dst[in] - destination
* @param    src[in] - source
* @param    num[in] - number half word (2 byte)
*/
flashState_type flash_write(void *dst, uint16_t *src, uint32_t num){
    uint16_t    *pRd;
    uint16_t    *pWr;
    uint16_t    *pEnd;

    FLASH->CR |= FLASH_CR_PG;
    while(flash_busy());

    pRd     = src;
    pWr     = dst;
    pEnd    = pWr + num;

    while(pWr < pEnd){
        *pWr++ = *pRd++;
        __DSB();
        while(flash_busy());
    }

    FLASH->CR &= ~FLASH_CR_PG;
    return flash_ok;
}

/******************************** END OF FILE ********************************/
