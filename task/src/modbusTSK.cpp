/*!****************************************************************************
 * @file		uartTSK.c
 * @author		d_el
 * @version		V1.2
 * @date		13.12.2020
 * @brief		connect interface with regulator
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <string.h>
#include <assert.h>
#include <FreeRTOS.h>
#include <task.h>
#include <mb.h>
#include <prmSystem.h>
#include <plog.h>
#include <flash.h>

/*!****************************************************************************
* MEMORY
*/
#define LOG_LOCAL_LEVEL P_LOG_NONE
static const char *logTag = "modbusTSK";
static bool needSave;

extern const uint8_t *const _fwstorage_flash_start;	/// See memory.ld
extern const uint8_t _fwstorage_flash_size;			/// See memory.ld

/*!****************************************************************************
* @brief	Connect program task
*/
void modbusTSK(void *pPrm){
	(void)pPrm;

	eMBInit(MB_RTU, 0x01, 0, 921600, MB_PAR_NONE);
	eMBEnable();

	while(1){
		eMBPoll();
	}
}

/*!****************************************************************************
 * @brief
 */
eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode){
	usAddress--;
	switch(eMode){
		/* Pass current register values to the protocol stack. */
		case MB_REG_READ:{
			Prm::IVal *ph = Prm::getbyaddress(usAddress);
			while(usNRegs > 0){
				if(ph == nullptr){
					P_LOGW(logTag, "r: illegal register address [%04X], regs %u", usAddress, usNRegs);
					return MB_ENOREG;
				}

				auto prmsize = ph->getsize();
				if(prmsize == 4 && usNRegs < 2){
					P_LOGW(logTag, "r: [%04X] usNRegs < 2 in 4 Byte parameter", usAddress);
					return MB_EINVAL;
				}

				(*ph)(true, nullptr);

				if(LOG_LOCAL_LEVEL >= P_LOG_DEBUG){
					char string[32];
					ph->tostring(string, sizeof(string));
					P_LOGD(logTag, "r: [%04X] %u %s: %s %s", usAddress, usNRegs, ph->getlabel(), string, ph->getunit());
				}

				uint8_t buffer[4] = {};
				ph->serialize(buffer);
				switch(prmsize){
					case 1:
					case 2:
						*pucRegBuffer++ = buffer[1];
						*pucRegBuffer++ = buffer[0];
						usAddress++;
						usNRegs--;
						break;
					case 4:
						*pucRegBuffer++ = buffer[1];
						*pucRegBuffer++ = buffer[0];
						*pucRegBuffer++ = buffer[3];
						*pucRegBuffer++ = buffer[2];
						usAddress += 2;
						usNRegs -= 2;
						break;
				}
				ph = Prm::getNext();
			}
		}
		break;

		/* Update current register values with new values from the protocol stack. */
		case MB_REG_WRITE:{
			Prm::IVal *ph = Prm::getbyaddress(usAddress);
			while(usNRegs > 0){
				if(ph == nullptr){
					P_LOGW(logTag, "w: illegal register address [%04X]", usAddress);
					return MB_ENOREG;
				}

				auto prmsize = ph->getsize();
				if(prmsize == 4 && usNRegs < 2){
					P_LOGW(logTag, "w: [%04X] usNRegs < 2 in 4 Byte parameter", usAddress);
					return MB_EINVAL;
				}

				uint8_t buffer[4];
				switch(prmsize){
					case 1:
					case 2:
						buffer[1] = *pucRegBuffer++;
						buffer[0] = *pucRegBuffer++;
						usAddress++;
						usNRegs--;
						break;

					case 4:
						buffer[1] = *pucRegBuffer++;
						buffer[0] = *pucRegBuffer++;
						buffer[3] = *pucRegBuffer++;
						buffer[2] = *pucRegBuffer++;
						usAddress += 2;
						usNRegs -= 2;
						break;
				}

				if(!ph->deserialize(buffer)){
					P_LOGW(logTag, "w [%04X]: out of range", usAddress);
					return MB_EINVAL;
				}

				(*ph)(false, nullptr);

				if(LOG_LOCAL_LEVEL >= P_LOG_DEBUG){
					char string[32];
					ph->tostring(string, sizeof(string));
					P_LOGD(logTag, "w: [%04X] %u %s: %s %s", usAddress, usNRegs, ph->getlabel(), string, ph->getunit());
				}

				if(ph->getsave() == Prm::savesys){
					needSave = true;
				}
				ph = Prm::getNext();
			}
		}
		break;
	}
	return MB_ENOERR;
}

/*!****************************************************************************
 * @brief
 */
eMBErrorCode eMBFileRecordCB(UCHAR* pucDataBuffer, USHORT usFile, USHORT usRecord, USHORT usLen, eMBRegisterMode eMode){
	(void)usFile;
	size_t offset = usRecord * 128;
	const uint8_t* fwstorage_flash_start = (uint8_t*)&_fwstorage_flash_start;
	size_t fwstorage_flash_size = (size_t)&_fwstorage_flash_size;

	if(usFile != 1){
		return MB_EINVAL;
	}
	if(offset >= fwstorage_flash_size){
		return MB_EINVAL;
	}

	if(eMode == MB_REG_WRITE){
		uint8_t m[128];
		for(size_t i = 0; i < usLen; i++){
				m[i*2 + 0] = *pucDataBuffer++;
				m[i*2 + 1] = *pucDataBuffer++;
		}
		flash_unlock();

		const uint32_t* pFlash = (uint32_t*)&fwstorage_flash_start[offset];
		if((offset % FLASH_PAGE_SIZE) == 0){
			for(size_t i = 0; i < FLASH_PAGE_SIZE / sizeof(uint32_t); i++){
				if(pFlash[i] != 0xFFFFFFFF){
					flash_erasePage((void*)pFlash);
					break;
				}
			}
		}

		flash_write((void*)pFlash, (void*)m, usLen);
		flash_lock();
	}
	return MB_ENOERR;
}

/*!****************************************************************************
 * @brief
 */
bool modbus_needSave(bool clear){
	bool cuurentState = needSave;
	if(clear){
		needSave = false;
	}
	return cuurentState;
}

/******************************** END OF FILE ********************************/
