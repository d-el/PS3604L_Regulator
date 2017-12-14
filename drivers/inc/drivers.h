/*!****************************************************************************
* @file    		drivers.c
* @author  		Storozhenko Roman - D_EL
* @version 		V1.0
* @date    		05-07-2013
* @copyright 	GNU Public License
*/
#ifndef DRIVERS_H
#define DRIVERS_H

/*!****************************************************************************
* Include
*/
#include "stdint.h"
#include "systemTSK.h"
#include "IQmathLib.h"
#include "clock.h"
#include "dac.h"
#include "pwm.h"
#include "i2c.h"
#include "ina226.h"
#include "board.h"
#include "flash.h"
#include "systemTSK.h"
#include "delay.h"

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
* Extern viriables
*/

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
void Init_Hard(void);
void prmInitDef(void);

#endif //DRIVERS_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/
