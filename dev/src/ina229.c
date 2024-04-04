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
#include "ina229.h"
#include <gpio.h>

/*!****************************************************************************
* User define
*/
#define ina226Addr		(0x80)

/*!****************************************************************************
* User typedef
*/
typedef enum{
	CONFIG = 0x00,          // Configuration 16
	ADC_CONFIG = 0x01,      // ADC Configuration 16
	SHUNT_CAL = 0x02,       // Shunt Calibration 16
	SHUNT_TEMPCO = 0x03,    // Shunt Temperature Coefficient 16
	VSHUNT = 0x04,          // Shunt Voltage Measurement 24
	VBUS = 0x05,            // Bus Voltage Measurement 24
	DIETEMP = 0x06,         // Temperature Measurement 16
	CURRENT = 0x07,         // Current Result 24
	POWER = 0x08,           // Power Result 24
	ENERGY = 0x09,          // Energy Result 40
	CHARGE = 0x0A,          // Charge Result 40
	DIAG_ALRT = 0x0B,       // Diagnostic Flags and Alert 16
	SOVL = 0x0C,            // Shunt Overvoltage Threshold 16
	SUVL = 0x0D,            // Shunt Undervoltage Threshold 16
	BOVL = 0x0E,            // Bus Overvoltage Threshold 16
	BUVL = 0x0F,            // Bus Undervoltage Threshold 16
	TEMP_LIMIT = 0x10,      // Temperature Over-Limit Threshold 16
	PWR_LIMIT = 0x11,       // Power Over-Limit Threshold 16
	MANUFACTURER_ID = 0x3E,	// Manufacturer ID 16
	DEVICE_ID = 0x3F		// ID Device ID 16	1
}ina229_registers_t;

typedef union{
	struct{
		uint16_t AVG :3; // ADC sample averaging count.
		uint16_t VTCT :3; // Conversion time of the temperature measurement
		uint16_t VSHCT :3; // Conversion time of the shunt voltage measurement
		uint16_t VBUSCT :3; // Conversion time of the bus voltage measurement
		uint16_t MODE :4;
	}bit;
	uint16_t all;
}ADC_CONFIG_t;

enum mode{
	MODE_Shutdown = 0x0, // Shutdown
	MODE_v_single = 0x1, // Triggered bus voltage, single shot
	MODE_i_single = 0x2, // Triggered shunt voltage, single shot
	MODE_iv_single = 0x3, // Triggered shunt voltage and bus voltage, single shot
	MODE_t_single = 0x4, // Triggered temperature, single shot
	MODE_tv_single = 0x5, // Triggered temperature and bus voltage, single shot
	MODE_ti_single = 0x6, // Triggered temperature and shunt voltage, single shot
	MODE_vit_single = 0x7, // Triggered bus voltage, shunt voltage and temperature, single shot
	MODE_Shutdown1 = 0x8, // Shutdown
	MODE_v_continuous = 0x9, // Continuous bus voltage only
	MODE_i_continuous = 0xA, // Continuous shunt voltage only
	MODE_iv_continuous = 0xB, // Continuous shunt and bus voltage
	MODE_t_continuous = 0xC, // Continuous temperature only
	MODE_vt_continuous = 0xD, // Continuous bus voltage and temperature
	MODE_ti_continuous = 0xE, // Continuous temperature and shunt voltage
	MODE_vit_continuous = 0xF // Continuous bus voltage, shunt voltage and temperature
};

enum conversionTime{
	CT_50µs = 0,
	CT_84µs = 1,
	CT_150µs = 2,
	CT_280µs = 3,
	CT_540µs = 4,
	CT_1052µs = 5,
	CT_2074µs = 6,
	CT_4120µs = 7
};

enum averagingCount{
	AVG_1 = 0,
	AVG_4 = 1,
	AVG_16 = 2,
	AVG_64 = 3,
	AVG_128 = 4,
	AVG_256 = 5,
	AVG_512 = 6,
	AVG_1024 = 7
};

/*!****************************************************************************
 * MEMORY
 */
static ina229_spi_t spi;

/*!****************************************************************************
 * @brief
 */
static bool ina229_spi_loc(void *dst, void *src, uint16_t len){
	gppin_reset(GP_SPI3_NSS);
	bool res = spi(dst, src, len);
	gppin_set(GP_SPI3_NSS);
	return res;
}

/*!****************************************************************************
 * @brief
 */
static bool ina229_readReg16s(ina229_registers_t reg, int16_t *val){
	uint8_t tx[3] = { (reg << 2) | 1 };
	uint8_t rx[3] = {};
	if(!ina229_spi_loc(rx, tx, 3)) return false;
	*val = (int16_t) rx[1] << 8 | rx[2];
	return true;
}

/*!****************************************************************************
 * @brief
 */
static bool ina229_readReg24s(ina229_registers_t reg, int32_t *val){
	uint8_t tx[4] = { (reg << 2) | 1 };
	uint8_t rx[4] = {};
	if(!ina229_spi_loc(rx, tx, 4)) return false;
	int32_t signedval = (int32_t) rx[1] << 16 | rx[2] << 8 | rx[3];
	if(signedval & 0x800000) signedval |= 0xFF000000;
	*val = signedval;
	return true;
}

/*!****************************************************************************
 * @brief
 */
static bool ina229_writeReg16u(ina229_registers_t reg, uint16_t val){
	uint8_t tx[3] = { (reg << 2) | 0, val >> 8, val & 0xFF };
	uint8_t rx[3] = {};
	if(!ina229_spi_loc(rx, tx, 3)) return false;
	return true;
}

/*!****************************************************************************
* @brief
* @retval
*/
bool ina229_init(ina229_spi_t _spi){
	spi = _spi;

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
	ADC_CONFIG_t adcconfig = {
			.bit.AVG = AVG_4,
			.bit.VSHCT = CT_2074µs,
			.bit.MODE = MODE_iv_continuous
	};
	return ina229_writeReg16u(ADC_CONFIG, adcconfig.all);
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

/*!****************************************************************************
 * @brief
 */
bool ina229_readCNVRF(bool *c){
	int16_t cnvrf;
	if(!ina229_readReg16s(DIAG_ALRT, &cnvrf)){
		return false;
	}
	*c = cnvrf & 0x02 ? true : false;
	return true;
}

/******************************** END OF FILE ********************************/
