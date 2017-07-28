/*!****************************************************************************
* @file    		newLib.c
* @author  		d_el
* @version 		V1.0
* @date    		30.12.2016, Storozhenko Roman
* @copyright 	GNU Public License
*/

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include <sys/types.h>
#include "stm32f3xx.h"

/* These magic symbols are provided by the linker.  */
extern void (*__preinit_array_start []) (void) __attribute__((weak));
extern void (*__preinit_array_end []) (void) __attribute__((weak));
extern void (*__init_array_start []) (void) __attribute__((weak));
extern void (*__init_array_end []) (void) __attribute__((weak));
extern void (*__fini_array_start []) (void) __attribute__((weak));
extern void (*__fini_array_end []) (void) __attribute__((weak));

extern void _init (void);
extern void _fini (void);
extern uint32_t __get_MSP(void);

/*!****************************************************************************
 * sbrk
 * Increase program data space.
 * Malloc and related functions depend on this
 */
caddr_t _sbrk(int incr) {
    extern char _ebss; // Defined by the linker
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_ebss;
    }
    prev_heap_end = heap_end;

    char * stack = (char*) __get_MSP();
     if (heap_end + incr >  stack)
     {
         //_write (STDERR_FILENO, "Heap and stack collision\n", 25);
         errno = ENOMEM;
         return  (caddr_t) -1;
         //abort ();
     }

    heap_end += incr;
    return (caddr_t) prev_heap_end;

}

/*!****************************************************************************
 */
void
__attribute__((weak))
_exit(int code __attribute__((unused)))
{
	while(1);
}

/*!****************************************************************************
 */
void
__attribute__((weak))
_init (){

}

/*!****************************************************************************
 */
void
__attribute__((weak))
_fini (){

}


/*!****************************************************************************
 * Iterate over all the init routines
 */
void
__libc_init_array (void){
  size_t count;
  size_t i;

  count = __preinit_array_end - __preinit_array_start;
  for (i = 0; i < count; i++)
    __preinit_array_start[i] ();

  _init ();

  count = __init_array_end - __init_array_start;
  for (i = 0; i < count; i++)
    __init_array_start[i] ();
}

/*!****************************************************************************
 * Run all the cleanup routines
 */
void
__libc_fini_array (void){
  size_t count;
  size_t i;

  count = __fini_array_end - __fini_array_start;
  for (i = count; i > 0; i--)
    __fini_array_start[i-1] ();

  _fini ();
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/
