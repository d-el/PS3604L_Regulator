/*!****************************************************************************
* @file    		drivers.c
* @author  		Storozhenko Roman - D_EL
* @version 		V1.0
* @date    		05-07-2013
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include <string.h>
#include "board.h"
#include "clock.h"
#include "uart.h"
#include "adc.h"
#include "dac.h"
#include "pwm.h"
#include "i2c.h"
#include "flash.h"
#include "drivers.h"

/*!****************************************************************************
* MEMORY
*/

/*!****************************************************************************
*
*/
void hardInit(void){
    clock_init();
    gpio_init();
    pwmFan_init();
    adc_init();
    dac_init();
    externalInterrupt_CcCv_init();
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/
