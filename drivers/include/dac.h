/*!****************************************************************************
 * @file    	dac.h
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	02.05.2016
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */
#ifndef dac_H
#define dac_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include "stm32f3xx.h"

/*!****************************************************************************
* User define
*/
#define DAC_MAX_VALUE	4095

/*!****************************************************************************
* Prototypes for the functions
*/
void dac_init(void);
void dac_ch1(uint16_t val);
void dac_ch2(uint16_t val);

#ifdef __cplusplus
}
#endif

#endif //dac_H
/******************************** END OF FILE ********************************/
