/*!****************************************************************************
 * @file		uartTSK.c
 * @author		d_el
 * @version		V1.2
 * @date		13.12.2017
 * @brief		connect interface with regulator
 * @copyright	Copyright (C) 2017 Storozhenko Roman
 *				All rights reserved
 *				This software may be modified and distributed under the terms
 *				of the BSD license.	 See the LICENSE file for details
 */

/*!****************************************************************************
* Include
*/
#include "uartTSK.h"

/*!****************************************************************************
* MEMORY
*/
uartTsk_type				uartTsk;
static SemaphoreHandle_t	connUartRxSem;

/******************************************************************************
 * Local function declaration
 */
static void uartTskHook(uart_type *puart);

/*!****************************************************************************
* @brief	Connect program task
*/
void uartTSK(void *pPrm){
	BaseType_t  res;
	uint16_t    crc;
	uint8_t     numRx = 0;

	// Create Semaphore for UART
	vSemaphoreCreateBinary(connUartRxSem);
	assert(connUartRxSem != NULL);

	uart_setCallback(connectUart, uartTskHook, uartTskHook);

    while(1){
        uart_read(connectUart, connectUart->pRxBff, PIECE_BUF_RX);       //Запуск нового приема
        res = xSemaphoreTake(connUartRxSem, pdMS_TO_TICKS(maxWait_ms));
        numRx += PIECE_BUF_RX - uartGetRemainRx(connectUart);

        if((numRx != 0)&&(res == pdTRUE)){
            crc = crc16Calc(&crcModBus, connectUart->pRxBff, sizeof(task_type) + sizeof(uint16_t));
            if(crc == 0){
                //Вынимаем принятые данные
                memcpy(&rg.tf.task, connectUart->pRxBff, sizeof(task_type));
                xSemaphoreGive(rxRequest);  //Отдаем семафор
                uartTsk.successfulRequest++;
                numRx = 0;

                //Отправка
                memcpy(connectUart->pTxBff, &rg.tf.state, sizeof(psState_type) + sizeof(meas_type));
                crc = crc16Calc(&crcModBus, connectUart->pTxBff, sizeof(psState_type) + sizeof(meas_type));
                *(uint16_t*)(connectUart->pTxBff + sizeof(psState_type) + sizeof(meas_type)) = crc;
                uart_write(connectUart, connectUart->pTxBff, sizeof(psState_type) + sizeof(meas_type) + sizeof(uint16_t));
                xSemaphoreTake(connUartRxSem, pdMS_TO_TICKS(100));

                uartTsk.state = uartConnect;
            }
            else{
                uartTsk.errorRequest++;
                uartTsk.state = uartNoConnect;
            }
        }
    }
}

/*!****************************************************************************
 * @brief	uart RX TX callback
 */
static void uartTskHook(uart_type *puart){
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(connUartRxSem, &xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken != pdFALSE){
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}

/***************** Copyright (C) Storozhenko Roman ******* END OF FILE *******/
