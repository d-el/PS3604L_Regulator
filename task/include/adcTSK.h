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
#define ADC_NUM_CH                  (3)
#define MA_FILTER_MAX_WITH          (128)

#define AdcVref                     3.3         //[V]
#define UDC_Rh                      47          //[kOhm]
#define UDC_Rl                      2           //[kOhm]

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
	uint16_t adcDefVal;
	uint16_t oversampling;
	uint16_t recursiveK;
	uint16_t MA_filter_WITH;

	uint32_t recursiveFilterCumul;
	uint16_t recursiveFilterOut;
	uint16_t MA_filterMas[MA_FILTER_MAX_WITH];
	uint16_t MA_filterIndex;
	uint32_t MA_accumulator;
} adcFilt_type;

typedef struct {
	adcFilt_type	adcFilt[ADC_NUM_CH];
	uint16_t 		filtered[ADC_NUM_CH];
	int16_t 		adcIna229;

	/*******************************/
	_iq 				udc;
	_iq 				voltage;        		//[V]
	_iq 				current;  				//[A]
	_iq 				currentInt;     		//[A]
	_iq 				currentExt;  		//[A]
	_iq14 				outPower;       		//[W]
	_iq14 				radPower;       		//[W]
	_iq14 				resistens;      		//[Ohm]
	uint32_t 			capacity;       		//[mAh]
	adcCurrentSensor_type	currentSensor;
	uint8_t 			externalSensorOk :1;
	uint8_t 			reverseVoltage :1;
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
