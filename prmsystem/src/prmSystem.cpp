/*!****************************************************************************
 * @file		prmSystem.cpp
 * @author		d_el - Storozhenko Roman
 * @version		V2.1
 * @date		25.01.2021
 * @copyright	The MIT License (MIT). Copyright (c) 2021 Storozhenko Roman
 * @brief		Parameters system
 */

/*!****************************************************************************
 * Include
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <crc.h>
#include "prmSystem.h"
#include "prmSystemCallback.h"

namespace Prm {

using crc_t = uint16_t;
constexpr uint16_t magic = 0x2805;
static size_t currentIndex;

#include "parameter.def"

template<class T>
bool Val<T>::deserialize(const void *src){
	T v = 0;
	memcpy(&v, src, sizeof(v));
	if constexpr(std::is_same_v<T, float>){
		if(std::isnan(v)){
			return false;
		}
	}
	if(v > handler.max || v < handler.min){
		return false;
	}
	val = v;
	return true;
}

template<class T>
size_t Val<T>::tostring(char *string, size_t size) const {
	constexpr int32_t pows[7] = { 1, 10, 100, 1000, 10000, 100000, 1000000 };

	if(handler.text){
		const char *s = handler.text->get(val);
		if(s) strncpy(string, s, size);
		return strlen(string);
	}

	if constexpr(std::is_unsigned<T>::value){
		if(handler.power == 0){
			uint32_t v = val;
			return snprintf(string, size, "%" PRIu32, v);
		}else{
			uint32_t a = val / pows[handler.power];
			uint32_t b = val % pows[handler.power];
			return snprintf(string, size, "%" PRIu32 ".%0*" PRIu32, a, handler.power, b);
		}
	}

	if constexpr(std::is_signed<T>::value){
		if(handler.power == 0){
			int32_t v = val;
			return snprintf(string, size, "%" PRIi32, v);
		}else{
			int32_t v = val;
			int32_t a = v / pows[handler.power];
			if(v < 0) v = -v;
			uint32_t b = v % pows[handler.power];
			return snprintf(string, size, "%" PRIi32 ".%0*" PRIu32, a, handler.power, b);
		}
	}
}

template<>
size_t Val<bool>::tostring(char *string, size_t size) const{
	strncpy(string, val ? "true" : "false", size);
	return strlen(string);
};

template<>
size_t Val<char>::tostring(char *string, size_t size) const{
	if(size < 2){
		return 0;
	}
	string[0] = val;
	string[1] = '\0';
	return 1;
};

template<>
size_t Val<float>::tostring(char *string, size_t size) const{
	return snprintf(string, size, "%f", val);
}

IVal *getbyaddress(uint16_t address){
	currentIndex = 0;
	for(auto *p : valuearray){
		if(address == p->getaddress()){
			return p;
		}
		currentIndex++;
	}
	return nullptr;
}

IVal *getNext(){
	currentIndex++;
	return valuearray[currentIndex];
}

size_t getSerialSize(Save save){
	size_t size = 0;
	for(auto *p : valuearray){
		if(save == p->getsave()){
			size += p->getsize();
		}
	}
	return size + sizeof(magic) + sizeof(crc_t);
}

bool serialize(Save save, uint8_t *dst){
	const uint8_t *dstentry = dst;
	memcpy(dst, &magic, sizeof(magic));
	dst += sizeof(magic);
	for(auto *p : valuearray){
		if(save == p->getsave()){
			p->serialize(dst);
			dst += p->getsize();
		}
	}
	crc_t crc = crc16Calc(&crcModBus, dstentry, dst - dstentry);
	memcpy(dst, &crc, sizeof(crc));
	return true;
}

bool deserialize(Save save, const uint8_t *src, size_t size){
	std::remove_cv_t<decltype(magic)> readmagic;
	memcpy(&readmagic, src, sizeof(readmagic));
	if(readmagic != magic) return false;
	if(crc16Calc(&crcModBus, src, size)) return false;
	src += sizeof(magic);
	for(auto *p : valuearray){
		if(save == p->getsave()){
			p->deserialize(src);
			src += p->getsize();
		}
	}
	return true;
}

} // namespace Prm

/******************************** END OF FILE ********************************/
