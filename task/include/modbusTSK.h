﻿/*!****************************************************************************
 * @file		modbusTSK.h
 * @author		d_el
 * @version		V1.2
 * @date		12.12.2020
 * @brief		connect interface with regulator
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */
#ifndef modbusTSK_H
#define modbusTSK_H

/*!****************************************************************************
* Include
*/
#include <stdint.h>

/*!****************************************************************************
* User define
*/

/*!****************************************************************************
* User typedef
*/

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* External variables
*/

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
void modbusTSK(void *pPrm);
bool modbus_needSave(bool clear);

#endif //modbusTSK_H
/******************************** END OF FILE ********************************/
