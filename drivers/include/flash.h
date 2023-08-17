﻿/*!****************************************************************************
 * @file		flash.h
 * @author		Storozhenko Roman - D_EL
 * @version		V1.1
 * @date		17.08.2023
 * @copyright	The MIT License (MIT). Copyright (c) 2023 Storozhenko Roman
 */
#ifndef FLASH_H
#define FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include <stdint.h>
#include <stdbool.h>

/*!****************************************************************************
* Prototypes for the functions
*/
void flash_unlock(void);
void flash_lock(void);
void flash_eraseAllPages(void);
bool flash_erasePage(void *address);
bool flash_write(void *dst, uint16_t *src, uint32_t num);

#ifdef __cplusplus
}
#endif

#endif //FLASH_H
/******************************** END OF FILE ********************************/
