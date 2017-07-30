if("${CMAKE_VERSION}" VERSION_LESS 3.1.0)
	message("Not a valid CMake version")
	message("On Ubuntu >= 16.04, install or upgrade via:")
	message("	sudo apt-get install cmake")
	message("")
	message("Official website: https://cmake.org/download/")
	message(FATAL_ERROR "Update CMake and try again" )
endif()

# Warning: Changing this modifies CMake's internal workings
# and leads to wrong toolchain detection
cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(Elka)

set(ELKA_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(ELKA_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")

#=============================================================================
# configuration
#
# must come before project to set toolchain
set(OS "FreeRTOS")

# Define GNU standard installation directories
include(GNUInstallDirs)

# Add support for external project building
include(ExternalProject)

# Add support for using python interpreter
include(FindPythonInterp)

list(APPEND CMAKE_MODULE_PATH "${ELKA_SOURCE_DIR}/cmake")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

include(common/elka_base)

# Setup install paths
if(NOT CMAKE_INSTALL_PREFIX)
	if (${OS} STREQUAL "posix")
		set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Install path prefix" FORCE)
	endif()
endif()
if (CMAKE_INSTALL_PREFIX)
	message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
endif()

# cmake modules
include(ExternalProject)

# force static lib build
set(BUILD_SHARED_LIBS OFF)

# Add libraries, subdirectories, etc to include all source and header files
include_directories(
  ${ELKA_BINARY_DIR}/inc/
  ${ELKA_BINARY_DIR}/src
  ${ELKA_SOURCE_DIR}/inc
  ${ELKA_SOURCE_DIR}/inc/CMSIS
  ${ELKA_SOURCE_DIR}/inc/Device/STM32F4xx
  ${ELKA_SOURCE_DIR}/inc/drivers
  ${ELKA_SOURCE_DIR}/inc/elka_hal
  ${ELKA_SOURCE_DIR}/inc/FreeRTOS
  ${ELKA_SOURCE_DIR}/inc/FreeRTOS/GCC/ARM_CM4F
  ${ELKA_SOURCE_DIR}/inc/modules
  ${ELKA_SOURCE_DIR}/inc/nvicconf
  ${ELKA_SOURCE_DIR}/inc/STM32F4xx_StdPeriph_Driver
  ${ELKA_SOURCE_DIR}/inc/systemlib
  ${ELKA_SOURCE_DIR}/inc/utils
  ${ELKA_SOURCE_DIR}/inc/uORB
  ${ELKA_SOURCE_DIR}/src/
  ${ELKA_SOURCE_DIR}/src/elka_comm
)

link_directories(
  ${ELKA_BINARY_DIR}
  ${ELKA_BINARY_DIR}/src
  ${ELKA_BINARY_DIR}/src/elka_comm/common
  ${ELKA_BINARY_DIR}/src/elka_comm/free_rtos
  ${ELKA_BINARY_DIR}/src/drivers
  ${ELKA_BINARY_DIR}/src/elka_hal
  ${ELKA_BINARY_DIR}/src/FreeRTOS
  ${ELKA_BINARY_DIR}/src/modules
  ${ELKA_BINARY_DIR}/src/STM32F4xx_StdPeriph_Driver
  ${ELKA_BINARY_DIR}/src/uORB
  ${ELKA_BINARY_DIR}/src/utils
)

set(ELKA_LINK_LIBS "")

add_definitions(
  -D__ELKA_FREERTOS
)

#=============================================================================
# git
#

elka_add_git_submodule(TARGET git_gencpp PATH "tools/gencpp")
elka_add_git_submodule(TARGET git_genmsg PATH "tools/genmsg")
elka_add_git_submodule(TARGET git_elka_comm PATH "src/elka_comm")

add_custom_target(submodule_clean
  WORKING_DIRECTORY ${ELKA_SOURCE_DIR}
	COMMAND git submodule deinit -f .
	COMMAND rm -rf .git/modules/*
	)

#=============================================================================
# source code generation
#

add_subdirectory(msg)
elka_generate_messages(TARGET msg_gen
	MSG_FILES ${msg_files}
	OS ${OS}
	INCLUDES ${msg_include_paths}
	DEPENDS git_genmsg git_gencpp
)

set(ELKA_LINK_LIBS
  ${ELKA_LINK_LIBS} msg_gen
)

#=============================================================================


# Set C/C++ compile and linking flags
set(GCC_COVERAGE_COMPILE_FLAGS "-march=armv7e-m -mthumb \
-mfloat-abi=softfp -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 \
-fsigned-char -ffunction-sections -fdata-sections -ffreestanding \
-fno-move-loop-invariants -Wall -Wextra -g3")

set(GXX_COVERAGE_COMPILE_FLAGS "-std=c++11")

set(GCC_COVERAGE_LINK_FLAGS
"-T \"${ELKA_SOURCE_DIR}/ldscripts/mem.ld\" \
-T \"${ELKA_SOURCE_DIR}/ldscripts/libs.ld\" \
-T \"${ELKA_SOURCE_DIR}/ldscripts/sections.ld\" \
-nostartfiles -Xlinker --gc-sections -Wl,-Map,\"elka.map\" \
--specs=nano.specs -o \"elka.elf\"")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${GXX_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER__FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

add_definitions(-DUSE_STDPERIPH_DRIVER)

add_subdirectory(src)

add_executable(elka
  src/main.cpp
  src/_write.cpp
)

add_dependencies(elka msg_gen)

target_link_libraries(elka
  ${ELKA_LINK_LIBS}
)

# Make executable depend on linkerscript through dummy file. This results
# in dummy.c being recompiled and, thus, the executable relinked when the
# linker script is touched. Since dummy.c is empty, the executable should
# be hardly affected.
#set_source_files_properties(
#  dummy.c PROPERTIES OBJECT_DEPENDS
#  ${CMAKE_SOURCE_DIR}/ldscripts/mems.ld
#  ${CMAKE_SOURCE_DIR}/ldscripts/libs.ld
#  ${CMAKE_SOURCE_DIR}/ldscripts/sections.ld
#)

# ------------------------------------------
# Add objcopy command to copy from .elf to .hex file at end
#add_custom_command(
#  OUTPUT elka.hex
#  COMMAND arm-none-eabi-objcopy -O ihex "elka.elf" "elka.hex"
#)

# Add size command to generate .siz file at end
#add_custom_command(
#  OUTPUT elka.siz
#  COMMAND arm-none-eabi-size --format=berkeley "elka.elf"
#)
