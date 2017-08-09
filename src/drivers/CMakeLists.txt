# src/drivers
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}/inc/drivers
  ${PROJECT_SOURCE_DIR}/inc/STM32F4xx_StdPeriph_Driver
	${PROJECT_SOURCE_DIR}/inc/Device/STM32F4xx
)

add_library(elka_drivers STATIC
	adc.c
	exti.c
	i2cdev.c
	i2croutines.c
	motors.c
	mpu6050.c
	nrf24l01.c
	nvic.c
  px4flow.c
)

add_dependencies(elka_drivers
  STM32F4xx_StdPeriph_Driver
)

target_link_libraries(elka_drivers
  STM32F4xx_StdPeriph_Driver
)

set(ELKA_LINK_LIBS
  ${ELKA_LINK_LIBS} elka_drivers
	PARENT_SCOPE
)
