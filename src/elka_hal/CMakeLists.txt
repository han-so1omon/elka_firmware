# src/elka_hal
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}/inc
  ${PROJECT_SOURCE_DIR}/inc/drivers
  ${PROJECT_SOURCE_DIR}/inc/elka_hal
  ${PROJECT_SOURCE_DIR}/inc/utils
  ${PROJECT_SOURCE_DIR}/inc/FreeRTOS
  ${PROJECT_SOURCE_DIR}/inc/FreeRTOS/GCC/ARM_CM4F
)

add_library(elka_hal STATIC
	imu.c
)

add_dependencies(elka_hal
  FreeRTOS
  elka_utils
  elka_drivers
)

target_link_libraries(elka_hal
  FreeRTOS
  elka_utils
  elka_drivers
)

set(ELKA_LINK_LIBS 
  ${ELKA_LINK_LIBS} elka_hal
  PARENT_SCOPE
)
