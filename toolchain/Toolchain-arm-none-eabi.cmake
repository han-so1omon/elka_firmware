include(CMakeForceCompiler)

# this one is important
set(CMAKE_SYSTEM_NAME Generic)

# Specify processor to build for
set(CMAKE_SYSTEM_PROCESSOR arm)

#this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
# requires a symbolic link typically from /usr/bin/arm-none-eabi-gcc
find_program(C_COMPILER arm-none-eabi-gcc
	PATHS /usr/bin/ 
	NO_DEFAULT_PATH
)

if(NOT C_COMPILER)
	message(FATAL_ERROR "could not find C compiler")
endif()
CMAKE_FORCE_C_COMPILER(${C_COMPILER} GNU)

find_program(CXX_COMPILER arm-none-eabi-g++
	PATHS /usr/bin/
	NO_DEFAULT_PATH
)

if(NOT CXX_COMPILER)
	message(FATAL_ERROR "could not find C++ compiler")
endif()
CMAKE_FORCE_CXX_COMPILER(${CXX_COMPILER} GNU)

# compiler tools
foreach(tool objcopy nm ld)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} arm-none-eabi-${tool}
		PATHS /usr/bin
		NO_DEFAULT_PATH
		)
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find arm-none-eabi-${tool}")
	endif()
endforeach()

# os tools
foreach(tool echo grep rm mkdir nm cp touch make unzip)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${TOOL}")
	endif()
endforeach()

#add_definitions(
#  -D __DF_RPI
#  )

# where is the target environment
set(CMAKE_FIND_ROOT_PATH  get_file_component(${C_COMPILER} PATH))

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
