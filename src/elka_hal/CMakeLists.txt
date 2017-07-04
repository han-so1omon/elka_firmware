# src/elka_hal
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}/inc/elka_hal
)

add_library(elka_hal STATIC
	imu.c
)

set(ELKA_LINK_LIBS "${ELKA_LINK_LIBS} elka_hal")
