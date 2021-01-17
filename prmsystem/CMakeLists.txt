add_library(prmsystem STATIC
	src/prmSystem.c
	src/prmSystemCallback.c
	)

target_include_directories(prmsystem PUBLIC
	include
	)

add_custom_command(
	OUTPUT prmsytem2json
	OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/../tools/prmsytem2json/build
	COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../tools/prmsytem2json/build.sh
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	)

add_custom_target(prmsystemobj
	BYPRODUCTS ${CMAKE_CURRENT_SOURCE_DIR}/include/parametrestype.h
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../tools/prmsytem2json/build/prmsytem2json ${CMAKE_CURRENT_SOURCE_DIR}/include/parametres.txt ${CMAKE_CURRENT_SOURCE_DIR}/include/parametrestype.h type
	
	BYPRODUCTS ${CMAKE_CURRENT_SOURCE_DIR}/include/parametresobj.h
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../tools/prmsytem2json/build/prmsytem2json ${CMAKE_CURRENT_SOURCE_DIR}/include/parametres.txt ${CMAKE_CURRENT_SOURCE_DIR}/include/parametresobj.h object
	
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/parametres.txt
	DEPENDS prmsytem2json
	COMMENT "Generate parametrestype.h from parametres.h"
)

add_custom_target(parameters.json
	BYPRODUCTS ${PROJECT_SOURCE_DIR}/parameters.json
    COMMAND ${PROJECT_SOURCE_DIR}/tools/prmsytem2json/build/prmsytem2json ${CMAKE_CURRENT_SOURCE_DIR}/include/parametres.txt ${PROJECT_SOURCE_DIR}/parameters.json json
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	DEPENDS prmsytem2json
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/parametres.txt
	COMMENT "Generate parameters.json from parametres.h"
)

add_dependencies(prmsystem prmsystemobj)
	
target_link_libraries(prmsystem utils)
