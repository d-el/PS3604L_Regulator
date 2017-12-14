/*!****************************************************************************
* @file         ds18b20.c
* @author       d_el
* @version      V2.1
* @date         30.03.2014
* @date         04.08.2014  remade for 1wire on uart
* @brief        ds18b20 driver
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include "ds18b20.h"

/*!****************************************************************************
* MEMORY
*/
volatile    tmpr_type   tem;

/*!****************************************************************************
* @brief    Настройка датчиков, сразу дает команду на конвертирование температуры
* @retval   0 - устройство обнаружено, и является ds18b20,
*           1 - не обнаружено 1-wire устройств,
*           2 - к.з. на линии,
*           3 - ошибка CRC,
*           4 - устройство не является ds18b20.
*/
uint8_t ds18b20Init(void){
    uint8_t result, buff[8];
    uint8_t crc = 0;

    result = ow_reset();
    if(result != 0){
        return result;
    }
    buff[0] = READ_ROM;
    ow_write(buff, 1);
    ow_read(buff, 8);
    crc = ow_crc8(buff, 7);
    if(crc != buff[7]){
        return 3;
    }
    if(buff[0] != 0x28){
        return 4;
    }

    //Set TH, TL, Resolution
    buff[0] = SKIP_ROM;
    buff[1] = WRITE_SCRATCHPAD;
    buff[2] = 127;              //TH
    buff[3] = 0;                //TL
    buff[4] = 0x7F;         	//12bit 750ms   0.0625
    ow_reset();
    ow_write(buff, 5);

    buff[0] = SKIP_ROM;
    buff[1] = CONVERT_T;
    ow_reset();
    ow_write(buff, 2);

    return 0;
}

/*!****************************************************************************
* @param    rl - low temperature register
* @param    rh - hight temperature register
* @retval   temperature X_XX
*/
uint16_t reg2tmpr(uint8_t rl, uint8_t rh){
    union{
        struct{
            uint8_t     rl;
            uint8_t     rh;
        }byte;
        uint16_t    word;
    }scratchpad;

    scratchpad.byte.rl = rl;
    scratchpad.byte.rh = rh;

    return (scratchpad.word * 10U + (16/2)) / 16;   //Деление с округлением
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/
