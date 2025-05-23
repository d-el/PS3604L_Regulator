﻿/*!****************************************************************************
 * @file		FreeRTOSConfig.h
 * @author		d_el - Storozhenko Roman
 * @version		V1.0
 * @date		06.08.2017
 * @copyright	GNU Lesser General Public License v3
 * @brief		FreeRTOS configuration file
 * 				http://www.freertos.org/a00110.html
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ISR */
#define xPortSysTickHandler SysTick_Handler
#define xPortPendSVHandler 	PendSV_Handler
#define vPortSVCHandler 	SVC_Handler

#define configUSE_PREEMPTION                    ( 1 )
#define configUSE_PORT_OPTIMISED_TASK_SELECTION ( 0 )
#define configUSE_TICKLESS_IDLE                 ( 0 )
#define configCPU_CLOCK_HZ                      ( 64000000 )
#define configTICK_RATE_HZ                      ( 1000 )
#define configMAX_PRIORITIES                    ( 6 )
#define configMINIMAL_STACK_SIZE                ( 512 )
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_16_BIT_TICKS                  ( 0 )
#define configIDLE_SHOULD_YIELD                 ( 1 )
#define configUSE_TASK_NOTIFICATIONS            ( 0 )
#define configUSE_MUTEXES                       ( 1 )
#define configUSE_RECURSIVE_MUTEXES             ( 0 )
#define configUSE_COUNTING_SEMAPHORES           ( 1 )
#define configUSE_ALTERNATIVE_API               ( 0 ) /* Deprecated! */
#define configQUEUE_REGISTRY_SIZE               ( 10 )
#define configUSE_QUEUE_SETS                    ( 0 )
#define configUSE_TIME_SLICING                  ( 0 )
#define configUSE_NEWLIB_REENTRANT              ( 0 )
#define configENABLE_BACKWARD_COMPATIBILITY     ( 0 )
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS ( 2 )

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION         ( 0 )
#define configSUPPORT_DYNAMIC_ALLOCATION        ( 1 )
#define configTOTAL_HEAP_SIZE                   ( 12 * 1024 )
#define configAPPLICATION_ALLOCATED_HEAP        ( 0 )

/* Hook function related definitions. */
#define configUSE_IDLE_HOOK                     ( 0 )
#define configUSE_TICK_HOOK                     ( 0 )
#define configCHECK_FOR_STACK_OVERFLOW          ( 2 )
#define configUSE_MALLOC_FAILED_HOOK            ( 1 )
#define configUSE_DAEMON_TASK_STARTUP_HOOK      ( 0 )

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS           ( 0 )
#define configUSE_TRACE_FACILITY                ( 1 )
#define configUSE_STATS_FORMATTING_FUNCTIONS    ( 0 )

/* Co-routine related definitions. */
#define configUSE_CO_ROUTINES                   ( 0 )
#define configMAX_CO_ROUTINE_PRIORITIES         ( 1 )

/* Software timer related definitions. */
#define configUSE_TIMERS                        ( 0 )
#define configTIMER_TASK_PRIORITY               ( 3 )
#define configTIMER_QUEUE_LENGTH                ( 10 )
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE )

/* Cortex-M specific definitions. */
#define configPRIO_BITS         4

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Optional functions - most linkers will remove unused functions anyway. */
#define INCLUDE_vTaskPrioritySet                ( 1 )
#define INCLUDE_uxTaskPriorityGet               ( 1 )
#define INCLUDE_vTaskDelete                     ( 1 )
#define INCLUDE_vTaskSuspend                    ( 1 )
#define INCLUDE_xResumeFromISR                  ( 0 )
#define INCLUDE_vTaskDelayUntil                 ( 1 )
#define INCLUDE_vTaskDelay                      ( 1 )
#define INCLUDE_xTaskGetSchedulerState          ( 0 )
#define INCLUDE_xTaskGetCurrentTaskHandle       ( 0 )
#define INCLUDE_uxTaskGetStackHighWaterMark     ( 1 )
#define INCLUDE_xTaskGetIdleTaskHandle          ( 0 )
#define INCLUDE_eTaskGetState                   ( 0 )
#define INCLUDE_xEventGroupSetBitFromISR        ( 0 )
#define INCLUDE_xTimerPendFunctionCall          ( 0 )
#define INCLUDE_xTaskAbortDelay                 ( 0 )
#define INCLUDE_xTaskGetHandle                  ( 0 )
#define INCLUDE_xTaskResumeFromISR              ( 0 )

/*-------NAME--------------------size [4 byte Word] */
#define ADC_TSK_SZ_STACK        512
#define SYSTEM_TSK_SZ_STACK     512
#define MODBUS_TSK_SZ_STACK     512
#define DS18B_TSK_SZ_STACK      512
/*-------NAME--------------------size [4 byte Word] */
#define ADC_TSK_PRIO            4
#define SYSTEM_TSK_PRIO         3
#define MODBUS_TSK_PRIO         5
#define DS18B_TSK_PRIO          1

/* A header file that defines trace macro can be included here. */

#if configGENERATE_RUN_TIME_STATS == 1

void vConfigureTimerForRunTimeStats(void);
unsigned long vGetTimerForRunTimeStats(void);

#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()    vConfigureTimerForRunTimeStats()
#define portGET_RUN_TIME_COUNTER_VALUE()            vGetTimerForRunTimeStats()

#endif

#endif /* FREERTOS_CONFIG_H */
/*************** LGPL ************** END OF FILE *********** D_EL ************/
