/*!****************************************************************************
 * @file		sdAdc.h
 * @author		Storozhenko Roman - D_EL
 * @version		V1.1
 * @date		08.10.2024
 * @copyright	The MIT License (MIT). Copyright (c) 2024 Storozhenko Roman
 */
#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include "stm32f3xx.h"

/*!****************************************************************************
* User define
*/
#define SDADC_DR_TO_LSB_ADD			32768

enum{
	CH_UINADC,
	CH_VREFM,
	CH_NUMBER
};

/*!****************************************************************************
* User typedef
*/
typedef struct adcStct{
	uint16_t		sampleRate;			// [us]
	int16_t			adcdr[CH_NUMBER];
	uint16_t		adcreg[CH_NUMBER];
	void (*tcHoock)(struct adcStct *adc);
}adcStct_type;

typedef void (*adcCallback_type)(adcStct_type *adc);

/*!****************************************************************************
* Prototypes for the functions
*/
void adc_init(void);
void adc_startSampling(void);
void adc_stopSampling(void);
void adc_setSampleRate(uint16_t us);
void adc_setCallback(adcCallback_type tcHoock);

#ifdef __cplusplus
}
#endif

#endif //ADC_H
/******************************** END OF FILE ********************************/
