include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Add libraries, subdirectories, etc to include all source and header files
add_subdirectory(drivers)
add_subdirectory(elka_hal)
add_subdirectory(elka_comm/common)
add_subdirectory(FreeRTOS)
add_subdirectory(modules)
add_subdirectory(STM32F4xx_StdPeriph_Driver)
add_subdirectory(utils)
