add_library(drivers STATIC
	src/clock.c
	src/dac.c
	src/drivers.c
	src/flash.c
	src/gpio.c
	src/i2c.c
	src/pwm.c
	src/sdAdc.c
	src/uart.c
	)

target_include_directories(drivers PUBLIC
	include
	)

target_link_libraries(drivers cmsis)
