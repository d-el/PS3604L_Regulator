add_library(utils STATIC
	src/crc.c
	src/hexdump.c
	src/plog.c
	src/version.c
	)

target_include_directories(utils PUBLIC
	include
	)
