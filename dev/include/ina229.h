/*!****************************************************************************
 * @file    	ina229.h
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	14.12.2016
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */
#ifndef INA229_H
#define INA229_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include <stdbool.h>

typedef bool (*ina229_spi_t)(void* dst, void* src, uint16_t len);

/*!****************************************************************************
* Prototypes for the functions
*/
bool ina229_init(ina229_spi_t spi);
bool ina229_trig(void);
bool ina229_readShuntVoltage(int32_t *v);
bool ina229_readCNVRF(bool *c);

#ifdef __cplusplus
}
#endif

#endif //INA229_H
/******************************** END OF FILE ********************************/
