/*!****************************************************************************
* @file    		ina226.c
* @author  		Storozhenko Roman - D_EL
* @version 		V1.0
* @date    		14.12.2016
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <spi.h>
#include "ina229.h"
#include <gpio.h>

/*!****************************************************************************
* MEMORY
*/
SemaphoreHandle_t spiSem;

/*!****************************************************************************
 * @brief	I2C callback
 */
static void spiTC_Hook(spi_type *spix){
	(void)spix;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(spiSem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

bool ina229_spi(void *dst, void *src, uint16_t len){
	gppin_reset(GP_SPI3_NSS);
	spi_transfer(spi3, dst, src, len);
	BaseType_t res = xSemaphoreTake(spiSem, pdMS_TO_TICKS(portMAX_DELAY));
	gppin_set(GP_SPI3_NSS);
	return res == pdTRUE && spi3->state == spiTCSuccess;
}

bool ina229_readReg16s(ina229_registers_t reg, int16_t *val){
	uint8_t tx[3] = { (reg << 2) | 1 };
	uint8_t rx[3] = {};
	if(!ina229_spi(rx, tx, 3)) return false;
	*val = (int16_t) rx[1] << 8 | rx[2];
	return true;
}

bool ina229_readReg24s(ina229_registers_t reg, int32_t *val){
	uint8_t tx[4] = { (reg << 2) | 1 };
	uint8_t rx[4] = {};
	if(!ina229_spi(rx, tx, 4)) return false;
	int32_t signedval = (int32_t) rx[1] << 16 | rx[2] << 8 | rx[3];
	if(signedval & 0x800000) signedval |= 0xFF000000;
	*val = signedval;
	return true;
}

bool ina229_writeReg16u(ina229_registers_t reg, uint16_t val){
	uint8_t tx[3] = { (reg << 2) | 0, val >> 8, val & 0xFF };
	uint8_t rx[3] = {};
	if(!ina229_spi(rx, tx, 3)) return false;
	return true;
}

/*!****************************************************************************
* @brief
* @retval
*/
bool ina229_init(void){
	vSemaphoreCreateBinary(spiSem);
	xSemaphoreTake(spiSem, portMAX_DELAY);

	spi_init(spi3, spiDiv4);
	spi_setCallback(spi3, spiTC_Hook);

	int16_t val;
	if(!ina229_readReg16s(DEVICE_ID, &val)){
		return false;
	}

	typedef struct{
		uint16_t REV_ID :4;
		uint16_t DIEID :12;
	}DEVICE_ID_t;

	DEVICE_ID_t* DEVICE_ID = (DEVICE_ID_t*)&val;

	const uint16_t deviceIdentification = 0x229;
	if(DEVICE_ID->DIEID != deviceIdentification){
		return false;
	}

	ina229_writeReg16u(CONFIG, 1 << 4/*Shunt full scale range selection across IN+ and IN–. 1h = ±40.96 mV*/);

	return true;
}

/*!****************************************************************************
* @brief
* @retval
*/
bool ina229_trig(void){
//	uint16_t adcconfig =	0xA /*Continuous shunt voltage only*/ << 12 |
//							7 /*4120 µs*/ << 6 |
//							0 /*averaging 1*/ << 0;

	uint16_t adcconfig =	2 /*Triggered shunt voltage triggered, single shot*/ << 12 |
							2 /*150 µs*/ << 6 |
							1 /*averaging 4*/ << 0;
	return ina229_writeReg16u(ADC_CONFIG, adcconfig);
}

/*!****************************************************************************
* @brief
* @retval
*/
bool ina229_readShuntVoltage(int32_t *v){
	int32_t vshunt;
	if(!ina229_readReg24s(VSHUNT, &vshunt)){ // bit 23-4, 3-0 reserved
		return false;
	}
	*v = vshunt / 16;
	return true;
}

bool ina229_readCNVRF(bool *c){
	int16_t cnvrf;
	if(!ina229_readReg16s(DIAG_ALRT, &cnvrf)){ // bit 23-4, 3-0 reserved
		return false;
	}
	*c = cnvrf & 0x02;
	return true;
}

/******************************** END OF FILE ********************************/
