/*!****************************************************************************
 * @file    	ina226.h
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	14.12.2016
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */
#ifndef INA226_H
#define INA226_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include <stdbool.h>

/*!****************************************************************************
* External variables
*/

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
bool ina229_init(void);
bool ina229_trig(void);
bool ina229_readShuntVoltage(int32_t *v);
bool ina229_readCNVRF(bool *c);

#ifdef __cplusplus
}
#endif

#endif //INA226_H
/******************************** END OF FILE ********************************/
