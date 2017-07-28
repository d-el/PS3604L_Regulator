/*!****************************************************************************
* @file    		timeMeas
* @author  		d_el
* @version 		V1.0
* @date    		04.10.2015, by d_el
* @brief   		debug
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include "timeMeas.h"

unsigned long long    measTime[NUMTIMERS];
//xFreeBytesRemaining

/*!****************************************************************************
* @brief
*/
void timMeasInit(void){
    RCC->APB1ENR  |= RCC_APB1ENR_TIM2EN;                        //Включили тактирование Timer
    MEASTIM->PSC     =  (MEASTIM_FREQ / 1000000) - 1;

}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/
