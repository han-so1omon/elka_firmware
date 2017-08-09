cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(Elka)

set(CMAKE_BUILD_TYPE DEBUG)

# Set C/C++ compile and linking flags
set(GCC_COVERAGE_COMPILE_FLAGS "-march=armv7e-m -mthumb \
-mfloat-abi=softfp -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 \
-fsigned-char -ffunction-sections -fdata-sections -ffreestanding \
-fno-move-loop-invariants -Wall -Wextra -g3")

set(GCC_COVERAGE_LINK_FLAGS
  #"-T \"${PROJECT_SOURCE_DIR}/ldscripts/mem.ld\" \
  #-T \"${PROJECT_SOURCE_DIR}/ldscripts/libs.ld\" \
  #-T \"${PROJECT_SOURCE_DIR}/ldscripts/sections.ld\" \
  "-T \"${PROJECT_SOURCE_DIR}/ldscripts/stm32_flash.ld\" \
-Xlinker --gc-sections -Wl,-Map,\"elka.map\" \
--specs=nano.specs")
#--specs=nosys.specs")
#--specs=rdimon.specs")
#-nostartfiles -Xlinker --gc-sections -Wl,-Map,\"elka.map\" \

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER__FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

#set(ELKA_LINK_LIBS rdimon)

add_definitions(-DUSE_STDPERIPH_DRIVER)

# Add libraries, subdirectories, etc to include all source and header files
include_directories(${PROJECT_SOURCE_DIR}/inc
${PROJECT_SOURCE_DIR}/inc/CMSIS
${PROJECT_SOURCE_DIR}/inc/Device/STM32F4xx
${PROJECT_SOURCE_DIR}/inc/drivers
${PROJECT_SOURCE_DIR}/inc/elka_hal
${PROJECT_SOURCE_DIR}/inc/FreeRTOS
${PROJECT_SOURCE_DIR}/inc/FreeRTOS/GCC/ARM_CM4F
${PROJECT_SOURCE_DIR}/inc/modules
${PROJECT_SOURCE_DIR}/inc/nvicconf
${PROJECT_SOURCE_DIR}/inc/STM32F4xx_StdPeriph_Driver
${PROJECT_SOURCE_DIR}/inc/utils
)

add_subdirectory(src)

# Set startup assembly file to be C file so that it may be linked
# against
set_property(
  SOURCE src/startup/startup_stm32f4xx.s
  PROPERTY LANGUAGE C
)

#FIXME make sure that this will create elka executable file
#TODO link against libraries
add_executable(elka
  src/main.c
  #src/test.c
  src/debug.c
  src/hw_config.c
  src/_write.c
  src/startup/startup_stm32f4xx.s
)

add_dependencies(elka
  ${ELKA_LINK_LIBS}
)

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
add_custom_target(elka_hex)
add_custom_command(
  TARGET elka_hex
  COMMAND arm-none-eabi-objcopy -O ihex "elka" "elka.hex" && chmod a+x elka.hex
  DEPENDS elka
  POST_BUILD
)

add_custom_target(elka_bin)
add_custom_command(
  TARGET elka_bin
  COMMAND arm-none-eabi-objcopy -O binary "elka" "elka.bin" && chmod a+x elka.bin
  DEPENDS elka
  POST_BUILD
)

# Add size command to generate .siz file at end
#add_custom_command(
#  OUTPUT elka.siz
#  COMMAND arm-none-eabi-size --format=berkeley "elka.elf"
#)
