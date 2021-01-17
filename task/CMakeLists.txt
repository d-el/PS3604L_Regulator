add_library(task STATIC
	src/adcTSK.c
	src/ds18TSK.c
	src/modbusTSK.c
	src/monitorTSK.c
	src/systemTSK.c
	)

target_include_directories(task PUBLIC
	include
	)
target_compile_options(task PUBLIC "-O0")
target_link_libraries(task freertos utils drivers dev systems prmsystem iqmathlib-cm4 freemodbus math)
