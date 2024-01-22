/* ----------------------- Standard includes --------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <uart.h>
#include <board.h>

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbconfig.h"

/* ----------------------- Defines  -----------------------------------------*/
#define connectUart		uart1
#define BUF_SIZE		UART1_RxBffSz         /* must hold a complete RTU frame. */

/* ----------------------- Static variables ---------------------------------*/
static bool bRxEnabled, bTxEnabled;
static SemaphoreHandle_t connUartTcSem;
static size_t rxSize = 0;

/*!****************************************************************************
 * @brief	uart RX TX callback
 */
static void uartTskHook(uart_type *puart){
	(void)puart;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(connUartTcSem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

int getReceiveData(void *data){
	memcpy(data, connectUart->pRxBff, rxSize);
	return rxSize;
}

int setTransmitData(void *data, int len){
	uart_write(connectUart, data, len);
	xSemaphoreTake(connUartTcSem, pdMS_TO_TICKS(100));
	LED_OFF();
	return 0;
}

/* ----------------------- Begin implementation -----------------------------*/
void vMBPortSerialEnable(BOOL bEnableRx, BOOL bEnableTx){
	/* it is not allowed that both receiver and transmitter are enabled. */
	assert( !bEnableRx || !bEnableTx );

	if(bEnableRx){
		bRxEnabled = true;
	}else{
		bRxEnabled = false;
	}

	if(bEnableTx){
		bTxEnabled = true;
	}else{
		bTxEnabled = false;
	}
}

BOOL xMBPortSerialInit(UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity){
	(void)ucPort;
	(void)ucDataBits;
	(void)eParity;

	// Create Semaphore for UART
	vSemaphoreCreateBinary(connUartTcSem);
	xSemaphoreTake(connUartTcSem, portMAX_DELAY);
	assert(connUartTcSem != NULL);

	uart_init(connectUart, ulBaudRate);
	uart_setCallback(connectUart, uartTskHook, uartTskHook);

	vMBPortSerialEnable( FALSE, FALSE );
	return TRUE;
}

void vMBPortClose(void){

}

BOOL xMBPortSerialPoll(){
	BOOL bStatus = FALSE;
	if(bRxEnabled){
#warning "timeout"
		uart_read(connectUart, connectUart->pRxBff, BUF_SIZE);
		BaseType_t res = xSemaphoreTake(connUartTcSem, portMAX_DELAY);
		rxSize = BUF_SIZE - uartGetRemainRx(connectUart);
		if((rxSize != 0)&&(res == pdTRUE)){
			LED_ON();
			xMBPortEventPost(EV_FRAME_RECEIVED);
			bStatus = TRUE;
		}
		pxMBFrameCBByteReceived();
	}
	return bStatus;
}
