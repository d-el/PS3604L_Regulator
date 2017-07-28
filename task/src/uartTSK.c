/*!****************************************************************************
* @file    		uartTSK.c
* @author  		D_EL
* @version 		V1.1
* @date    		2015-20-11
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include "uartTSK.h"

/*!****************************************************************************
* MEMORY
*/
uartTsk_type        uartTsk;

/*!****************************************************************************
* @brief	Connect programm task
*/
void uartTSK(void *pPrm){
	BaseType_t  res;
	uint16_t    crc;
	uint8_t     numRx = 0;
    
    while(1){
        uart_read(connectUart, connectUart->pRxBff, PieceBufRx);       //Запуск нового приема
        res = xSemaphoreTake(uart1Sem, pdMS_TO_TICKS(maxWait_ms));
        numRx += PieceBufRx - uartGetRemainRx(connectUart);

        if((numRx != 0)&&(res == pdTRUE)){
            crc = GetCrc(connectUart->pRxBff, sizeof(task_type) + sizeof(uint16_t));
            if(crc == 0){
                //Вынимаем принятые данные
                memcpy(&rg.tf.task, connectUart->pRxBff, sizeof(task_type));
                xSemaphoreGive(rxRequest);  //Отдаем семафор
                uartTsk.successfulRequest++;
                numRx = 0;
                
                //Отправка
                memcpy(connectUart->pTxBff, &rg.tf.state, sizeof(psState_type) + sizeof(meas_type));
                crc = GetCrc(connectUart->pTxBff, sizeof(psState_type) + sizeof(meas_type));
                *(uint16_t*)(connectUart->pTxBff + sizeof(psState_type) + sizeof(meas_type)) = crc;
                uart_write(connectUart, connectUart->pTxBff, sizeof(psState_type) + sizeof(meas_type) + sizeof(uint16_t));
                xSemaphoreTake(uart1Sem, pdMS_TO_TICKS(100));

                uartTsk.state = uartConnect;
            }
            else{
                uartTsk.errorRequest++;
                uartTsk.state = uartNoConnect;
            }
        }
    }
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/
