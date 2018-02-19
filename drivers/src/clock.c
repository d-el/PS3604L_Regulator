/*!****************************************************************************
* @file    clock.c
* @author  d_el
* @version V1.0
* @date    09.01.2016, by d_el
* @copyright GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include "stm32f3xx.h"
#include "clock.h"

/*!****************************************************************************
* MEMORY
*/
clock_type      clock;

/*!****************************************************************************
*
*/
void clock_init(void){
    uint32_t    	timeOut;
    useGen_type		useGen;

    RCC->CFGR &=~(  RCC_CFGR_PLLSRC     |
                    RCC_CFGR_PLLXTPRE   |
					RCC_CFGR_PLLMUL)   	;                       //Предочистка

    /**************************************
    * Пытаемся запустить генератор HSE
    */
    RCC->CR     |= RCC_CR_HSEON;                                //HSE oscillator ON
    timeOut     = RCC_WAIN_TIMEOUT;
    while(((RCC->CR & RCC_CR_HSERDY) == 0) && (--timeOut != 0));

    //Запустился HSE
    if(timeOut != 0){                                           //HSE is ready
        RCC->CFGR   |= RCC_CFGR_PLLSRC_HSE_PREDIV;         		//Тактировать PLL от HSE
        RCC->CFGR2  = HSE_PLL_PREDIV - 1;                       //Делитель
        RCC->CFGR   |= ((HSE_PLL_MUL - 2) << 18);               //Умножитель
        useGen = clock_useHse;
    }

    //Запускаем HSI
    else{                                                       //HSE is not ready
        RCC->CR     &= ~RCC_CR_HSEON;                           //HSE oscillator OFF
        RCC->CR     |= RCC_CR_HSION;                            //HSI oscillator ON
        while((RCC->CR & RCC_CR_HSION) == 0);
        RCC->CFGR   |= RCC_CFGR_PLLSRC_HSI_DIV2;                //Тактировать PLL от HSI/2
        RCC->CFGR2  = HSI_PLL_PREDIV - 1;                       //Делитель
        RCC->CFGR   |=((HSI_PLL_MUL - 2) << 18);                //Умножитель
        useGen = clock_useHsi;
    }

    /**************************************
	* Включаем PLL
	*/
    RCC->CR     |= RCC_CR_PLLON;                                //Запустить PLL
    while((RCC->CR & RCC_CR_PLLRDY) == 0);                   	//Ожидание готовности PLL
    RCC->CFGR   &= ~RCC_CFGR_SW;                                //Очистить биты SW0, SW1
    RCC->CFGR   |= RCC_CFGR_SW_PLL;                             //Тактирование с выхода PLL
    while((RCC->CFGR & RCC_CFGR_SWS) != 0x08);                  //Ожидание переключения на PLL

    clock.useGen = useGen;
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/
