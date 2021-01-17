add_library(cmsis STATIC
	Device/ST/STM32F3xx/Source/Templates/system_stm32f3xx.c
	)

target_include_directories(cmsis PUBLIC
	Include
	Device/ST/STM32F3xx/Include
	)
