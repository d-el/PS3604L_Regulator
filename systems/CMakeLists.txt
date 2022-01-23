add_library(systems STATIC
	src/debugCore.c
	src/memmgr.c
	src/printp.c
	src/semihosting.c
	src/startup_stm32f373xx.c
	src/syscall.c
	src/sysTimeMeas.c
	)

target_include_directories(systems PUBLIC
	include
	)

target_link_libraries(systems cmsis drivers freertos)
