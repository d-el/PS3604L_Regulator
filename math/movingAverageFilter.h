/*!****************************************************************************
 * @file		movingAverageFilter.h
 * @author		Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date		22-01-2022
 * @copyright 	The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman
 */

#ifndef MOVINGAVERAGEFILTER_H
#define MOVINGAVERAGEFILTER_H

#include <stdint.h>
#include <stddef.h>

template<class T, size_t MaxSize>
class MovingAverageFilter {
public:
	MovingAverageFilter(T _defValue=0, size_t _size = MaxSize):
		size(_size), index(0), acc(0)
	{
		for(size_t i = 0; i < size; i++){
			buffer[i] = _defValue;
			acc += _defValue;
		}
	};

	T proc(T val){
		acc -= buffer[index];
		buffer[index] = val;
		acc += buffer[index];

		index++;
		if(index >= size){
			index = 0;
		}

		return acc / (int32_t)size;
	}

	void setsize(size_t _size){
		if(_size <= sizeof(buffer)/sizeof(buffer[0])){
			size = _size;
			acc = 0;
			auto lastval = buffer[index];
			for(size_t i = 0; i < size; i++){
				buffer[i] = lastval;
				acc += lastval;
			}
		}
	}

	size_t getsize(){
		return size;
	}

private:
	size_t size;
	size_t index = 0;
	T buffer[MaxSize];
	decltype(buffer[0] * buffer[0]) acc;
};

#endif /* MOVINGAVERAGEFILTER_H */

/******************************** END OF FILE ********************************/
