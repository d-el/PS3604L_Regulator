/*!****************************************************************************
 * @file    	pwm.h
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	02.05.2016
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */
#ifndef pwm_H
#define pwm_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include <stdint.h>

/*!****************************************************************************
* User define
*/
#define PWM_FAN_FREQ    40000   //[Hz]
#define PWM_LED_FREQ    2000    //[Hz]

/*!****************************************************************************
* Prototypes for the functions
*/
void pwmFan_init(void);
void FanPwmSet(uint16_t dc);
void Init_PwmLed(void);
void LedPwmSet(uint16_t dc);

#ifdef __cplusplus
}
#endif

#endif //pwm_H
/******************************** END OF FILE ********************************/
