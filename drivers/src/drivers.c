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
#include "delay.h"
#include "drivers.h"

/*!****************************************************************************
* MEMORY
*/

/*!****************************************************************************
*
*/
void Init_Hard(void){
    clock_init();
    gpio_init();
    pwmFan_init();
    delTim_init();
    sdadc_init();
    dac_init();
    externalInterrupt_CcCv_init();
    i2c_init(i2c1);
    uart_init(uart1, BR38400);  //Connect
    uart_init(uart3, BR9600);   //1WIRE

    nvMem_state_type nvMemState = nvMem_error;
    nvMemState = nvMem_init();
    nvMemState = nvMem_loadPrm(nvMem.nvMemBase);
    if(nvMemState != nvMem_ok){
        prmInitDef();
        rg.tf.state.bit.noCalibration = 1;
    }
}

/*!****************************************************************************
* Set settings by default
*/
void prmInitDef(void){
    static const regSetting_type defSettings = {
    	.pU = {
    		{.qu  = _IQ(0.0101), 	.adc = 414, 	.dac = 25,},
			{.qu  = _IQ(0.1050), 	.adc = 580, 	.dac = 35,},
			{.qu  = _IQ(19.0000), 	.adc = 33230, 	.dac = 2021,},
			{.qu  = _IQ(30.0000),	.adc = 52292, 	.dac = 3178,},
    	},
		.pI = {
			{.qi  = _IQ(0.00966), 	.adc = 562, 	.dac = 35,},
			{.qi  = _IQ(0.01007), 	.adc = 1980, 	.dac = 121,},
			{.qi  = _IQ(1.5000), 	.adc = 24539, 	.dac = 1491,},
			{.qi  = _IQ(3.0000), 	.adc = 48647, 	.dac = 2958,},
		},
		.pIEx = {
			{.qi  = _IQ(0.00966), 	.adc = 50, 		.dac = 35,},
			{.qi  = _IQ(0.01007), 	.adc = 340, 	.dac = 121,},
			{.qi  = _IQ(1.5000), 	.adc = 3214, 	.dac = 1491,},
			{.qi  = _IQ(3.0000), 	.adc = 32767, 	.dac = 2958,},
		},
    };

    memcpy(&rg.rgSet, &defSettings, sizeof(regSetting_type));
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/
