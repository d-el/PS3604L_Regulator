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
#include "string.h"
#include "board.h"
#include "clock.h"
#include "uart.h"
#include "adc.h"
#include "dac.h"
#include "pwm.h"
#include "i2c.h"
#include "ina226.h"
#include "flash.h"
#include "drivers.h"
#include "prmSystem.h"
#include "systemTSK.h"

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
    sdadc_init();
    dac_init();
    externalInterrupt_CcCv_init();
    i2c_init(i2c1);
    uart_init(uart1, BR38400);  //Connect
    uart_init(uart3, BR9600);   //1WIRE

    prm_state_type res = prm_load(SYSFLASHADR, prmFlash);
    if(res != prm_ok){
    	prm_loadDefault(prmFlash);
		rg.tf.state.bit.noCalibration = 1;
	}
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/
