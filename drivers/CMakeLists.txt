add_library(drivers STATIC
	src/clock.c
	src/cm4.c
	src/dac.c
	src/drivers.c
	src/flash.c
	src/gpio.c
	src/sdAdc.c
	src/spi.c
	src/uart.c
	)

target_include_directories(drivers PUBLIC
	include
	)

target_link_libraries(drivers cmsis)
