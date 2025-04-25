/*
 * imageheader.h
 *
 *  Created on: Mar 12, 2021
 *      Author: del
 */

#ifndef IMAGEHEADER_H
#define IMAGEHEADER_H

#include <stdint.h>

static const uint32_t _header_magic = 0x36232587;		/// Firmware header magic. See memory.ld

typedef struct imageHeader{
	uint32_t valueCRC;
	uint32_t size; // Size image include this header
	uint32_t magic;
	uint32_t versionMajor;
	uint32_t versionMinor;
	uint32_t versionPach;
	uint32_t exeOffset; // Offset execute from start image
}imageHeader_t;


static inline const imageHeader_t* getImageHeader(){
	extern const struct imageHeader _imageheader_start;
	return &_imageheader_start;
}

static inline const uint8_t* getImageStartAddress(){
	extern const struct imageHeader _imageheader_start;
	return (uint8_t*)&_imageheader_start;
}

#endif /* IMAGEHEADER_H */
