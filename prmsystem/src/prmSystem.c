/*!****************************************************************************
 * @file		prmSystem.c
 * @author		d_el - Storozhenko Roman
 * @version		V1.1
 * @date		05.11.2020
 * @copyright	The MIT License (MIT). Copyright (c) 2017 Storozhenko Roman
 * @brief		Parameters system
 */

/*!****************************************************************************
 * Include
 */
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <crc.h>
#include "prmSystem.h"
#include "prmSystemCallback.h"

/*!****************************************************************************
 * MEMORY
 */
//! Parameters data
static prmData_type prmData;
#include "parametresobj.h"

//! Type size table
static const uint8_t sizePrm[] = {
		sizeof(bool),
		sizeof(char),
		sizeof(uint8_t),
		sizeof(int8_t),
		sizeof(uint16_t),
		sizeof(int16_t),
		sizeof(uint32_t),
		sizeof(int32_t),
		sizeof(float),
		sizeof(uint32_t),
		sizeof(uint32_t),
		sizeof(uint32_t),
		sizeof(uint8_t) * 4
};

/*!****************************************************************************
 * @brief	initialize parameter system
 */
bool prm_init(void){
	return true;
}

/*!****************************************************************************
 * @brief	get parameter size
 * @param[in]	parameter type
 * @retval		size in bytes
 */
uint8_t prm_getSize(const prmHandle_type *const prmHandle){
	return sizePrm[prmHandle->type];
}

const prmHandle_type* prm_getHandler(uint16_t index){
	if(index < endOfNumberPrm){
		return &prmh[index];
	}
	else{
		return NULL;
	}
}

/*!****************************************************************************
 * @brief	call callback if it exist
 */
void prm_callback(const prmHandle_type *prmHandle, void *arg){
	if(prmHandle->pCallback != NULL){
		prmHandle->pCallback(prmHandle, arg);
	}
}

/*!****************************************************************************
 * @brief	Write value to parameter
 */
void prm_writeVal(const prmHandle_type *const prmHandle, const prmval_type prmval, void *arg){
	switch(prmHandle->type){
		case boolFrmt:
			prmHandle->prm->t_boolFrmt = prmval.t_boolFrmt;
			break;
		case charFrmt:
			prmHandle->prm->t_charFrmt = prmval.t_charFrmt;
			break;
		case u8Frmt:
			prmHandle->prm->t_s8Frmt = prmval.t_u8Frmt;
			break;
		case s8Frmt:
			prmHandle->prm->t_s8Frmt = prmval.t_s8Frmt;
			break;
		case u16Frmt:
			prmHandle->prm->t_u16Frmt = prmval.t_u16Frmt;
			break;
		case s16Frmt:
			prmHandle->prm->t_s16Frmt = prmval.t_s16Frmt;
			break;
		case u32Frmt:
		case unixTimeFrmt:
		case unixDateFrmt:
		case ipAdrFrmt:
		case bytesFmt:
			prmHandle->prm->t_u32Frmt = prmval.t_u32Frmt;
			break;
		case s32Frmt:
			prmHandle->prm->t_s32Frmt = prmval.t_s32Frmt;
			break;
		case floatFrmt:
			prmHandle->prm->t_floatFrmt = prmval.t_floatFrmt;
			break;
	}
	prm_callback(prmHandle, arg);
}

prmval_type prm_readVal(const prmHandle_type *const prmHandle){
	prm_callback(prmHandle, NULL);
	return *prmHandle->prm;
}

prmval_type prm_nreadVal(parametresNum_type parametres){
	if(parametres < endOfNumberPrm){
		return *(prmh[parametres].prm);
	}
	return (prmval_type){};
}

/*!****************************************************************************
 * @brief	Load to all parameters default value
 */
void prm_loadDefault(prmNvSave_type prmNvSave){
	for(uint32_t iterator = 0; iterator < endOfNumberPrm; iterator++){
		if(prmh[iterator].save == prmNvSave){
			switch(prmh[iterator].type){
				case boolFrmt:
					prmh[iterator].prm->t_boolFrmt = prmh[iterator].def.t_boolFrmt;
					break;
				case charFrmt:
					prmh[iterator].prm->t_charFrmt = prmh[iterator].def.t_charFrmt;
					break;
				case u8Frmt:
					prmh[iterator].prm->t_u8Frmt = prmh[iterator].def.t_u8Frmt;
					break;
				case s8Frmt:
					prmh[iterator].prm->t_s8Frmt = prmh[iterator].def.t_s8Frmt;
					break;
				case u16Frmt:
					prmh[iterator].prm->t_u16Frmt = prmh[iterator].def.t_u16Frmt;
					break;
				case s16Frmt:
					prmh[iterator].prm->t_s16Frmt = prmh[iterator].def.t_s16Frmt;
					break;
				case u32Frmt:
				case unixTimeFrmt:
				case unixDateFrmt:
				case ipAdrFrmt:
				case bytesFmt:
					prmh[iterator].prm->t_u32Frmt = prmh[iterator].def.t_u32Frmt;
					break;
				case s32Frmt:
					prmh[iterator].prm->t_s32Frmt = prmh[iterator].def.t_s32Frmt;
					break;
				case floatFrmt:
					prmh[iterator].prm->t_floatFrmt = prmh[iterator].def.t_floatFrmt;
					break;
			}
		}
	}
}

/*!****************************************************************************
 * @brief	Calculate size packet
 */
size_t prm_size(prmNvSave_type prmNvSave){
	uint16_t signature = 0x2805;
	uint16_t crc;
	size_t size;
	size = sizeof(signature);
	for(uint32_t iterator = 0; iterator < endOfNumberPrm; iterator++){
		if(prmh[iterator].save == prmNvSave){
			size += prm_getSize(&prmh[iterator]);
		}
	}
	size += sizeof(crc);
	return size;
}

/*!****************************************************************************
 * @brief	Store all parameters
 */
prm_state_type prm_serialize(void *pMemory, size_t *size, prmNvSave_type prmNvSave){
	uint16_t signature = 0x2805;
	uint16_t crc;
	uint8_t *pbuf = pMemory;

	// Copy data to signature
	memcpy(pbuf, &signature, sizeof(signature));
	*size = sizeof(signature);

	// Copy data to buffer
	for(uint32_t iterator = 0; iterator < endOfNumberPrm; iterator++){
		if(prmh[iterator].save == prmNvSave){
			memcpy(pbuf + *size, prmh[iterator].prm, prm_getSize(&prmh[iterator]));
			*size += prm_getSize(&prmh[iterator]);
		}
	}

	// Calculate CRC
	crc = crc16Calc(&crcModBus, pbuf, *size);

	// Copy CRC
	memcpy(pbuf + *size, &crc, sizeof(crc));
	*size += sizeof(crc);
	return prm_ok;
}

/*!****************************************************************************
 * @brief	Load all parameters
 */
prm_state_type prm_deserialize(void *pMemory, prmNvSave_type prmNvSave){
	uint8_t *pbuf = pMemory;
	size_t size = prm_size(prmNvSave);

	// Check signature
	uint16_t signature;
	memcpy(&signature, pbuf, sizeof(signature));
	if(signature != 0x2805){
		return prm_signatureError;
	}

	// Check CRC
	uint16_t crc = crc16Calc(&crcModBus, pbuf, size);
	if(crc != 0){
		return prm_crcError;
	}

	// Copy data
	pbuf += sizeof(signature);
	for(uint32_t iterator = 0; iterator < endOfNumberPrm; iterator++){
		if(prmh[iterator].save == prmNvSave){
			memcpy(prmh[iterator].prm, pbuf, prm_getSize(&prmh[iterator]));
			pbuf += prm_getSize(&prmh[iterator]);
		}
	}

	return prm_ok;
}

/*!****************************************************************************
 * @brief
 */
static size_t printUsigVar(char *string, size_t size, const prmHandle_type *prmHandler, uint32_t var){
	static const int32_t pows[] = { 1, 10, 100, 1000, 10000, 100000, 1000000 };

	if(prmHandler->power == 0){
		return snprintf(string, size, "%"PRIu32, var);
	}else{
		uint32_t a = var / pows[prmHandler->power];
		uint32_t b = var % pows[prmHandler->power];
		return snprintf(string, size, "%"PRIu32".%0*"PRIu32, a, prmHandler->power, b);
	}
}

/*!****************************************************************************
 * @brief
 */
static size_t printSigVar(char *string, size_t size, const prmHandle_type *prmHandler, int32_t var){
	static const int32_t pows[] = { 1, 10, 100, 1000, 10000, 100000, 1000000 };

	if(prmHandler->power == 0){
		return snprintf(string, size, "%"PRIi32, var);
	}else{
		uint32_t a = var / pows[prmHandler->power];
		uint32_t b = abs(var) % pows[prmHandler->power];
		return snprintf(string, size, "%"PRIi32".%0*"PRIu32, a, prmHandler->power, b);
	}
}

/*!****************************************************************************
 * @brief
 */
static size_t printFloatVar(char *string, size_t size, const prmHandle_type *prmHandler){
	return snprintf(string, size, /*"%.*f"*/"%f", /*prmHandler->power,*/ prmHandler->prm->t_floatFrmt);
}

size_t prm_toString(char *string, size_t size, const prmHandle_type *prmHandler){
	switch(prmHandler->type){
		case boolFrmt:
			return snprintf(string, size, prmHandler->prm->t_boolFrmt ? "true" : "false");
		case charFrmt:
			return snprintf(string, size, "%c", prmHandler->prm->t_charFrmt);
		case u8Frmt:
			return printUsigVar(string, size, prmHandler, prmHandler->prm->t_u8Frmt);
		case s8Frmt:
			return printSigVar(string, size, prmHandler, prmHandler->prm->t_s8Frmt);
		case u16Frmt:
			return printUsigVar(string, size, prmHandler, prmHandler->prm->t_u16Frmt);
		case s16Frmt:
			return printSigVar(string, size, prmHandler, prmHandler->prm->t_s16Frmt);
		case u32Frmt:
			return printUsigVar(string, size, prmHandler, prmHandler->prm->t_u32Frmt);
		case s32Frmt:
			return printSigVar(string, size, prmHandler, prmHandler->prm->t_s32Frmt);
		case floatFrmt:
			return printFloatVar(string, size, prmHandler);
		case ipAdrFrmt:
			return snprintf(string, size, "%"PRIu8 ".%"PRIu8 ".%"PRIu8 ".%"PRIu8,
					prmHandler->prm->bytes[0], prmHandler->prm->bytes[1], prmHandler->prm->bytes[2], prmHandler->prm->bytes[3]);
		default:
			return snprintf(string, size, "Error");
	}
}

void prm_toPrm(const char *string, const prmHandle_type *prmHandler){
	static const int32_t pows[] = { 1, 10, 100, 1000, 10000, 100000, 1000000 };
	int32_t ia = 0;
	uint32_t ua = 0;
	uint32_t b = 0;

	switch(prmHandler->type){
		case boolFrmt:
			if(strcmp("true", string) == 0){
				prmHandler->prm->t_boolFrmt = true;
			}else{
				prmHandler->prm->t_boolFrmt = false;
			}
			return;
		case charFrmt:
			sscanf(string, "%c", &prmHandler->prm->t_charFrmt);
			return;
		case u8Frmt:
			sscanf(string, "%"PRIu32".%"PRIu32, &ua, &b);
			prmHandler->prm->t_u8Frmt = ua * pows[prmHandler->power] + b;
			return;
		case s8Frmt:
			sscanf(string, "%"PRIi32".%"PRIu32, &ia, &b);
			prmHandler->prm->t_s8Frmt = ia * pows[prmHandler->power] + b;
			return;
		case u16Frmt:
			sscanf(string, "%"PRIu32".%"PRIu32, &ua, &b);
			prmHandler->prm->t_u16Frmt = ua * pows[prmHandler->power] + b;
			return;
		case s16Frmt:
			sscanf(string, "%"PRIi32".%"PRIu32, &ia, &b);
			prmHandler->prm->t_s16Frmt = ia * pows[prmHandler->power] + b;
			return;
		case u32Frmt:
			sscanf(string, "%"PRIu32".%"PRIu32, &ua, &b);
			prmHandler->prm->t_u32Frmt = ua * pows[prmHandler->power] + b;
			return;
		case s32Frmt:
			sscanf(string, "%"PRIi32".%"PRIu32, &ia, &b);
			prmHandler->prm->t_s32Frmt = ia * pows[prmHandler->power] + b;
			return;
		case floatFrmt:
			sscanf(string, "%f", &prmHandler->prm->t_floatFrmt);
			return;
		case unixDateFrmt:
		case unixTimeFrmt:
		case ipAdrFrmt:
		default:
			return;
	}
}

bool prm_greaterThan(const prmHandle_type *const ph, prmval_type a, prmval_type b){
	switch(ph->type){
		case boolFrmt:
			return a.t_boolFrmt != false && b.t_boolFrmt == false;
		case charFrmt:
			return a.t_charFrmt > b.t_charFrmt;
		case u8Frmt:
			return a.t_u8Frmt > b.t_u8Frmt;
		case s8Frmt:
			return a.t_s8Frmt > b.t_s8Frmt;
		case u16Frmt:
			return a.t_u16Frmt > b.t_u16Frmt;
		case s16Frmt:
			return a.t_s16Frmt > b.t_s16Frmt;
		case u32Frmt:
		case unixDateFrmt:
		case unixTimeFrmt:
		case ipAdrFrmt:
			return a.t_u32Frmt > b.t_u32Frmt;
		case s32Frmt:
			return a.t_s32Frmt > b.t_s32Frmt;
		case floatFrmt:
			return a.t_floatFrmt > b.t_floatFrmt;
		default:
			return false;
	}
}

bool prm_lessThan(const prmHandle_type *const ph, prmval_type a, prmval_type b){
	switch(ph->type){
		case boolFrmt:
			return a.t_boolFrmt == false && b.t_boolFrmt != false;
		case charFrmt:
			return a.t_charFrmt < b.t_charFrmt;
		case u8Frmt:
			return a.t_u8Frmt < b.t_u8Frmt;
		case s8Frmt:
			return a.t_s8Frmt < b.t_s8Frmt;
		case u16Frmt:
			return a.t_u16Frmt < b.t_u16Frmt;
		case s16Frmt:
			return a.t_s16Frmt < b.t_s16Frmt;
		case u32Frmt:
		case unixDateFrmt:
		case unixTimeFrmt:
		case ipAdrFrmt:
			return a.t_u32Frmt < b.t_u32Frmt;
		case s32Frmt:
			return a.t_s32Frmt < b.t_s32Frmt;
		case floatFrmt:
			return a.t_floatFrmt < b.t_floatFrmt;
		default:
			return false;
	}
}

bool prm_equal(const prmHandle_type *const ph, prmval_type a, prmval_type b){
	switch(ph->type){
		case boolFrmt:
			return a.t_boolFrmt == b.t_boolFrmt;
		case charFrmt:
			return a.t_charFrmt == b.t_charFrmt;
		case u8Frmt:
			return a.t_u8Frmt == b.t_u8Frmt;
		case s8Frmt:
			return a.t_s8Frmt == b.t_s8Frmt;
		case u16Frmt:
			return a.t_u16Frmt == b.t_u16Frmt;
		case s16Frmt:
			return a.t_s16Frmt == b.t_s16Frmt;
		case u32Frmt:
		case unixDateFrmt:
		case unixTimeFrmt:
		case ipAdrFrmt:
			return a.t_u32Frmt == b.t_u32Frmt;
		case s32Frmt:
			return a.t_s32Frmt == b.t_s32Frmt;
		case floatFrmt:
			return (a.t_floatFrmt - b.t_floatFrmt) < 10e-6;
		default:
			return false;
	}
}

/******************************** END OF FILE ********************************/
