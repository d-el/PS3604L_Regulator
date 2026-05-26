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
	A1CH_TSH2,
	A1CH_TSH1,
	A1CH_IFAN,
	A1CH_V2,
	A1CH_V1,
	A1CH_NUMBER
};

enum{
	A3CH_VG,
	A3CH_V4,
	A3CH_V3,
	A3CH_NUMBER
};

/*!****************************************************************************
* User typedef
*/
typedef struct adcStct{
	uint16_t		sampleRate;			// [us]
	int16_t			adcdr1[A1CH_NUMBER];
	uint16_t		adcreg1[A1CH_NUMBER];
	int16_t			adcdr3[A3CH_NUMBER];
	uint16_t		adcreg3[A3CH_NUMBER];
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
