add_library(systems STATIC
	src/debugCore.c
	src/memmgr.c
	src/SEGGER_RTT.c
	src/semihosting.c
	src/startup_stm32f373xx.c
	src/syscall.c
	src/sysTimeMeas.c
	src/write.c
	)

target_include_directories(systems PUBLIC
	include
	)

target_compile_options(systems PUBLIC
	-DSEGGER_RTT_SECTION=".rtti"
	)

target_link_libraries(systems cmsis drivers freertos utils)

target_link_options(systems PUBLIC -Wl,--require-defined=writeInit)
