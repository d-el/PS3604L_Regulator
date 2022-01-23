/*!****************************************************************************
 * @file    	systemTSK.h
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	14-09-2015
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */
#ifndef systemTSK_H
#define systemTSK_H

/*!****************************************************************************
* Include
*/

/*!****************************************************************************
* User define
*/
#define SYSTEM_TSK_PERIOD	(4)		///<[ms]
#define CUR_OFF_TIME		(1000)	///<[ms]
#define MAX_WAIT_RxRequest	(20)	///<[ms]

#define COOLER_PWM_START	(0.3)	///<[k PWM]
#define TEMP_FAN_OFF		(35.0)	///<[°C]
#define TEMP_FAN_ON			(40.0)	///<[°C]
#define TEMP_FAN_MAX		(60.0)	///<[°C]
#define TEMP_DISABLE		(80.0)	///<[°C]

#define VTASK_FILTER_K		(3)

#define MIN_VIN_VOLTAGE		(35.0)		//[V]

/*!****************************************************************************
* Prototypes for the functions
*/
void OSinit(void);

#endif //systemTSK_H
/******************************** END OF FILE ********************************/
