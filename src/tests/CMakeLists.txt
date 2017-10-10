# src/tests
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests )
set( CMAKE_BUILD_DIRECTORY ${CMAKE_BINARY_DIR}/tests )

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}/inc/drivers
  ${PROJECT_SOURCE_DIR}/inc/STM32F4xx_StdPeriph_Driver
	${PROJECT_SOURCE_DIR}/inc/Device/STM32F4xx
)

add_executable(test_i2c
  test_i2c.c
)

add_dependencies(test_i2c
  elka_drivers
  STM32F4xx_StdPeriph_Driver
)  

target_link_libraries(test_i2c
  elka_drivers
  STM32F4xx_StdPeriph_Driver
)

add_custom_target(test_i2c_bin)
add_custom_command(
  TARGET test_i2c_bin 
  COMMAND cd ${CMAKE_BINARY_DIR}/tests && arm-none-eabi-objcopy -O binary "test_i2c" "test_i2c.bin" && chmod a+x test_i2c.bin 
  DEPENDS test_i2c
  POST_BUILD
)

