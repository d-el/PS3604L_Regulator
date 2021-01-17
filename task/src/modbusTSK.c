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
#include <semphr.h>
#include <uart.h>
#include <crc.h>
#include <mb.h>
#include <prmSystem.h>
#include <board.h>

#define PIECE_BUF_RX        256	//[bytes]
#define connectUart			uart1

/*!****************************************************************************
* MEMORY
*/
static SemaphoreHandle_t connUartRxSem;
static bool needSave;

/******************************************************************************
 * Local function declaration
 */
static void uartTskHook(uart_type *puart);

/*!****************************************************************************
* @brief	Connect program task
*/
void modbusTSK(void *pPrm){
	(void)pPrm;

	// Create Semaphore for UART
	vSemaphoreCreateBinary(connUartRxSem);
	xSemaphoreTake(connUartRxSem, portMAX_DELAY);
	assert(connUartRxSem != NULL);

	uart_init(uart1, 115200);
	uart_setCallback(connectUart, uartTskHook, uartTskHook);
	eMBInit(MB_RTU, 0x01, 0, 115200, MB_PAR_NONE);
	eMBEnable();

    while(1){
        uart_read(connectUart, connectUart->pRxBff, PIECE_BUF_RX);
        BaseType_t res = xSemaphoreTake(connUartRxSem, portMAX_DELAY);
        size_t numRx = PIECE_BUF_RX - uartGetRemainRx(connectUart);

        if((numRx != 0)&&(res == pdTRUE)){
        	LED_ON();
        	mbSlaveSetReceive(connectUart->pRxBff, numRx);
        	if(eMBPoll() == MB_ENOERR){
				uint8_t txsize = mbSlaveGetTransmit(connectUart->pTxBff);
				if(txsize > 0){
					uart_write(connectUart, connectUart->pTxBff, txsize);
					xSemaphoreTake(connUartRxSem, pdMS_TO_TICKS(100));
				}
        	}
        	LED_OFF();
        }
    }
}

/*!****************************************************************************
 * @brief
 */
static const prmHandle_type *getHandlerByAddress(uint16_t address){
	size_t i = 0;
	while(1){
		const prmHandle_type *ph = prm_getHandler(i++);
		if(ph == NULL){
			break;
		}
		if(ph->addr == address){
			return ph;
		}
	}

	return NULL;
}

/*!****************************************************************************
 * @brief
 */
#define P_LOGW(...)
#define P_LOGD(...)

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode){
	usAddress--;
	switch(eMode){
		/* Pass current register values to the protocol stack. */
		case MB_REG_READ:
			while(usNRegs > 0){
				const prmHandle_type *ph = getHandlerByAddress(usAddress);
				if(ph == NULL){
					P_LOGW(logTag, "read: illegal register address [%04X], regs %u", usAddress, usNRegs);
					return MB_ENOREG;
				}

				uint8_t prmsize = prm_getSize(ph);

				if(prmsize == 4 && usNRegs < 2){
					P_LOGW(logTag, "read: [%04X] usNRegs < 2 in 4 Byte parameter", usAddress);
					return MB_EINVAL;
				}

				prmval_type prmval = prm_readVal(ph);
				char string[32];
				prm_toString(string, sizeof(string), ph);
				P_LOGD(logTag, "read: [%04X] %u %s %s %s", usAddress, usNRegs, ph->label, string, ph->units);

				switch(prmsize){
					case 1:
						*pucRegBuffer++ = 0;
						*pucRegBuffer++ = prmval.bytes[0];
						usAddress++;
						usNRegs--;
						break;
					case 2:{
						*pucRegBuffer++ = prmval.bytes[1];
						*pucRegBuffer++ = prmval.bytes[0];
						usAddress++;
						usNRegs--;
						}break;
					case 4:
						*pucRegBuffer++ = prmval.bytes[1];
						*pucRegBuffer++ = prmval.bytes[0];
						*pucRegBuffer++ = prmval.bytes[3];
						*pucRegBuffer++ = prmval.bytes[2];
						usAddress += 2;
						usNRegs -= 2;
						break;
				}
			}
			break;

			/* Update current register values with new values from the
			 * protocol stack. */
		case MB_REG_WRITE:
			while(usNRegs > 0){
				const prmHandle_type *ph = getHandlerByAddress(usAddress);

				if(ph == NULL){
					P_LOGW(logTag, "write: illegal register address [%04X]", usAddress);
					return MB_ENOREG;
				}

				if(ph->chmod != chmodRW){
					P_LOGW(logTag, "write: [%04X] register only for read", usAddress);
					return MB_EINVAL;
				}

				uint8_t prmsize = prm_getSize(ph);
				if(prmsize == 4 && usNRegs < 2){
					P_LOGW(logTag, "write: [%04X] usNRegs < 2 in 4 Byte parameter", usAddress);
					return MB_EINVAL;
				}

				prmval_type prmval = {};
				switch(prmsize){
					case 1:
						pucRegBuffer++;
						prmval.bytes[0] = *pucRegBuffer++;
						usAddress++;
						usNRegs--;
						break;
					case 2:
						prmval.bytes[1] = *pucRegBuffer++;
						prmval.bytes[0] = *pucRegBuffer++;
						usAddress++;
						usNRegs--;
						break;

					case 4:
						prmval.bytes[1] = *pucRegBuffer++;
						prmval.bytes[0] = *pucRegBuffer++;
						prmval.bytes[3] = *pucRegBuffer++;
						prmval.bytes[2] = *pucRegBuffer++;
						usAddress += 2;
						usNRegs -= 2;
						break;
				}

				if(prm_greaterThan(ph, ph->min, prmval) || prm_lessThan(ph, ph->max, prmval)){
					P_LOGW(logTag, "write [%04X]: out of range", usAddress);
					return MB_EINVAL;
				}
				bool writeaction = true;
				prm_writeVal(ph, prmval, &writeaction);
				char string[32];
				prm_toString(string, sizeof(string), ph);
				P_LOGD(logTag, "write: [%04X] %u %s %s %s", usAddress, usNRegs, ph->label, string, ph->units);

				if(ph->save == prmSaveSys){
					needSave = true;
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

/*!****************************************************************************
 * @brief	uart RX TX callback
 */
static void uartTskHook(uart_type *puart){
	(void)puart;
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(connUartRxSem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/******************************** END OF FILE ********************************/
