/*!****************************************************************************
 * @file    	adcTSK.h
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0.0
 * @date    	07-01-2015
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */
#ifndef ADC_TSK_H
#define ADC_TSK_H

/*!****************************************************************************
 * Include
 */
#include "IQmathLib.h"

/*!****************************************************************************
 * User define
 */
#define AdcVref							3.3		//[V]
#define UDC_Rh							47		//[kOhm]
#define UDC_Rl							2		//[kOhm]

#define REVERSE_VOLTAGE_THRESHOLD		10		//[adc lsb with oversampling]
#define CURRENT_SENSOR_THRESHOLD_UP		100		//[X_XXX A]
#define CURRENT_SENSOR_THRESHOLD_DOWN	90		//[X_XXX A]

/*!****************************************************************************
 * User typedef
 */
typedef enum {
	adcCurrentSensorInternal,
	adcCcurrentSensorExternal
} adcCurrentSensor_type;

typedef struct {
	struct {
		uint16_t uin;
		uint16_t u;
		uint16_t i;
		int32_t iex;
	}filtered;
	bool externalSensorOk;
} adcTaskStct_type;

/*!****************************************************************************
 * External variables
 */
extern adcTaskStct_type adcTaskStct;

/*!****************************************************************************
 * Prototypes for the functions
 */
void adcTSK(void *pPrm);

#endif //ADC_TSK_H
/******************************** END OF FILE ********************************/
