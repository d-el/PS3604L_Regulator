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

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* User typedef
*/

/*!****************************************************************************
* External variables
*/

/*!****************************************************************************
* Macro functions
*/
#define setDacCh1(u16val)	DAC->DHR12R1 = (u16val)
#define setDacCh2(u16val)	DAC->DHR12R2 = (u16val)

/*!****************************************************************************
* Prototypes for the functions
*/
void dac_init(void);

#ifdef __cplusplus
}
#endif

#endif //dac_H
/******************************** END OF FILE ********************************/
