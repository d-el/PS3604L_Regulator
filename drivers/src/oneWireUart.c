/*!****************************************************************************
* @file    		oneWireUart.c
* @author  		d_el
* @version 		V1.0
* @date    		21.07.2016, Storozhenko Roman
* @brief   		driver 1Wire on UART
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include "oneWireUart.h"

/*!****************************************************************************
* MEMORY
*/

/*!****************************************************************************
* @brief    Set One Wire pin to push-pull output
*/
void ow_setOutHi(void){
	gppin_init(pinsMode[GP_DS18B20].p, pinsMode[GP_DS18B20].npin, alternateFunctionPushPull,
			pullDisable, pinsMode[GP_DS18B20].iniState, 0);
}

/*!****************************************************************************
* @brief    Set One Wire pin to Open Drain output
*/
void ow_setOutOpenDrain(void){
	gppin_init(pinsMode[GP_DS18B20].p, pinsMode[GP_DS18B20].npin, pinsMode[GP_DS18B20].mode,
			pullDisable, pinsMode[GP_DS18B20].iniState, UART1_PINAFRX);
}

/*!****************************************************************************
* @brief    Reset pulse and presence detect
* @param    None
* @retval   owSt_type
*/
owSt_type ow_init(void){
    BaseType_t  res __attribute((unused));

    if(gppin_get(GP_DS18B20) == 0){
        return owShortCircle;       //Check on the Short Circle bus
    }

    uart_setBaud(OW_UART, BR9600);
    OW_UART->pTxBff[0] = 0xF0;
    res = xSemaphoreTake(uart3Sem, pdMS_TO_TICKS(OW_TIMEOUT));

    uart_read(OW_UART, OW_UART->pRxBff, 1);
    uart_write(OW_UART, OW_UART->pTxBff, 1);
    res = xSemaphoreTake(uart3Sem, pdMS_TO_TICKS(OW_TIMEOUT));

    if(((OW_UART->pRxBff[0] >= 0x90)&&(OW_UART->pRxBff[0] <= 0xE0))||(OW_UART->pRxBff[0] == 0)){
        return owOk;
    }else{
        return owNotFound;
    }
}

/*!***************************************************************************
* @brief  Write data
* @param  src - pointer to source buffer
* @param  len - number bytes for transmit
* @retval None
*/
void ow_write(const void *src, uint8_t len){
    uint8_t *pSrc       = (uint8_t*)src;
    uint8_t *pSrcEnd    = pSrc + len;
    uint8_t *pBff       = OW_UART->pTxBff;
    uint8_t mask, byteTrans = len << 3;

    while(pSrc < pSrcEnd){
        for(mask = 1; mask != 0; mask <<= 1){
            if((*pSrc & mask) != 0){
                *pBff++ = 0xFF;
            }else{
                *pBff++ = 0x00;
            }
        }
        pSrc++;
    }

    uart_setBaud(OW_UART, BR115200);
    uart_read(OW_UART, OW_UART->pRxBff, byteTrans);
    uart_write(OW_UART, OW_UART->pTxBff, byteTrans);
    xSemaphoreTake(uart3Sem, pdMS_TO_TICKS(OW_TIMEOUT));
}

/*!***************************************************************************
* @brief  Read data
* @param  dst - pointer to destination buffer
* @param  len - number bytes for receive
* @retval None
*/
void ow_read(void *dst, uint8_t len){
	BaseType_t  res __attribute((unused));
    uint8_t 	*pDst       = dst;
    uint8_t 	*pDstEnd    = pDst + len;
    uint8_t 	*pBff       = OW_UART->pRxBff + 0;
    uint8_t 	mask, byteTrans = len << 3;

    memset(OW_UART->pTxBff, 0xFF, byteTrans);

    uart_setBaud(OW_UART, BR115200);
    uart_read(OW_UART, OW_UART->pRxBff, byteTrans);
    uart_write(OW_UART, OW_UART->pTxBff, byteTrans);
    res = xSemaphoreTake(uart3Sem, pdMS_TO_TICKS(OW_TIMEOUT));

    while(pDst < pDstEnd){
        *pDst = 0;
        for(mask = 1; mask != 0; mask <<= 1){
            if(*pBff++ == 0xFF){
                *pDst |= mask;     //Read '1'
            }
        }
        pDst++;
    }
}

/*!***************************************************************************
* @brief  Подсчет CRC-8-Dallas/Maxim
* @param  Указатель на массив, колличество элементов
* @retval Result: the value of the received data
*/
uint8_t ow_crc8(uint8_t *mas, uint8_t n){
    uint8_t j , i, tmp, data, crc = 0;

    for(i = 0; i < n; i++){
        data = *mas;
        for(j = 0; j < 8; j++){
            tmp = (crc ^ data) & 0x01;
            if (tmp == 0x01) crc = crc ^ 0x18;
            crc = (crc >> 1) & 0x7F;
            if (tmp == 0x01) crc = crc | 0x80;
            data = data >> 1;
        }
        mas++;
    }
    return crc;
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/
