﻿/*!****************************************************************************
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

/*!****************************************************************************
* MEMORY
*/
#define LOG_LOCAL_LEVEL P_LOG_NONE
static const char *logTag = "modbusTSK";
static bool needSave;

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
					P_LOGW(logTag, "read: illegal register address [%04X], regs %u", usAddress, usNRegs);
					return MB_ENOREG;
				}

				auto prmsize = ph->getsize();
				if(prmsize == 4 && usNRegs < 2){
					P_LOGW(logTag, "read: [%04X] usNRegs < 2 in 4 Byte parameter", usAddress);
					return MB_EINVAL;
				}

				(*ph)(true, nullptr);

				char string[64];
				ph->tostring(string, sizeof(string));
				P_LOGD(logTag, "read: [%04X] %u %s: %s %s", usAddress, usNRegs, ph->getlabel(), string, ph->getunit());

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
					P_LOGW(logTag, "write: illegal register address [%04X]", usAddress);
					return MB_ENOREG;
				}

				auto prmsize = ph->getsize();
				if(prmsize == 4 && usNRegs < 2){
					P_LOGW(logTag, "write: [%04X] usNRegs < 2 in 4 Byte parameter", usAddress);
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
					P_LOGW(logTag, "write [%04X]: out of range", usAddress);
					return MB_EINVAL;
				}

				(*ph)(false, nullptr);

				char string[64];
				ph->tostring(string, sizeof(string));
				P_LOGD(logTag, "write: [%04X] %u %s: %s %s", usAddress, usNRegs, ph->getlabel(), string, ph->getunit());

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
bool modbus_needSave(bool clear){
	bool cuurentState = needSave;
	if(clear){
		needSave = false;
	}
	return cuurentState;
}

/******************************** END OF FILE ********************************/
