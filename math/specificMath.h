/*!****************************************************************************
 * @file		specificMath.h
 * @author		Storozhenko Roman - D_EL
 * @version		V1.1
 * @date		02-01-2024
 * @copyright	The MIT License (MIT). Copyright (c) 2024 Storozhenko Roman
 */
#ifndef specificMath_H
#define specificMath_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include "stdint.h"
#include "IQmathLib.h"

/*!****************************************************************************
* IQ to integer with scale
*/
static inline
int32_t IQtoInt(_iq val, int32_t scale, uint8_t N = GLOBAL_Q){
	int64_t a = (int64_t)val * scale + (1 << N) / 2;
	return a / (1 << N);
}

/*!****************************************************************************
* Integer with scale to IQ
*/
static inline
_iq IntToIQ(int32_t val, int32_t scale, uint8_t N = GLOBAL_Q){
	int64_t a = (int64_t)val * (1 << N) + scale / 2;
	return a / scale;
}

/*!****************************************************************************
* Linear extrapolation
*/
static inline
_iq iq_lerp(_iq x1, _iq y1, _iq x2, _iq y2, _iq x){
	_iq df = y2 - y1;
	_iq dx = x2 - x1;
	if(dx == 0){
		return MAX_IQ_POS;
	}
	return y1 + ((int64_t)df * ((x - x1)) + dx / 2) / dx;
}


/*!****************************************************************************
* Linear extrapolation
*/
static inline
_iq s32iq_lerp(int32_t x1, _iq y1, int32_t x2, _iq y2, int32_t x){
	_iq df = y2 - y1;
	int32_t dx = x2 - x1;
	if(dx == 0){
		return MAX_IQ_POS;
	}
	return y1 + ((int64_t)df * ((x - x1)) + dx / 2) / dx;
}

#ifdef __cplusplus
}
#endif

#endif //specificMath_H
/******************************** END OF FILE ********************************/
