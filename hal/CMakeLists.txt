add_library(hal STATIC
	src/clock.c
	src/cm4.c
	src/dac.c
	src/drivers.c
	src/flash.c
	src/gpio.c
	src/sdAdc.c
	src/spi.c
	src/sysTimeMeas.c
	src/uart.c
	)

target_include_directories(hal
	PUBLIC
		include
	PRIVATE
		include/hal
	)

target_link_libraries(hal cmsis)
