/*!****************************************************************************
 * @file    		adcTSK.h
 * @author  		Storozhenko Roman - D_EL
 * @version 		V1.0.0
 * @date    		07-01-2015
 * @copyright 	GNU Public License
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
#warning
#define MIN_VIN_VOLTAGE				10.0		//[V]
#define REVERSE_VOLTAGE_THRESHOLD	10			//[adc lsb with oversampling]

/*!****************************************************************************
 * User typedef
 */
typedef enum {
	internalCurrentSensor, externalCurrentSensor,
} currentSensor_type;

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
	int16_t 		adcIna226;

	/*******************************/
	_iq 				udc;
	_iq 				voltage;        		//[V]
	_iq 				current;  				//[A]
	_iq 				currentInt;     		//[A]
	_iq 				currentIna226;  		//[A]
	_iq14 				outPower;       		//[W]
	_iq14 				radPower;       		//[W]
	_iq14 				resistens;      		//[Ohm]
	uint32_t 			capacity;       		//[mA/h]
	currentSensor_type	currentSensor;
	uint8_t 			externalSensorOk :1;
	uint8_t 			reverseVoltage :1;
	uint8_t 			lowInputVoltage :1;
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
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/
