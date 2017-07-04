# src/drivers
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}/inc/drivers
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
)

set(ELKA_LINK_LIBS
  "${ELKA_LINK_LIBS} elka_drivers"
)
