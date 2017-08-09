# src/utils
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}/inc/utils
	${PROJECT_SOURCE_DIR}/inc/Device/STM32F4xx
	${PROJECT_SOURCE_DIR}/inc/STM32F4xx_StdPeriph_Driver
)

add_library(elka_utils STATIC
	cfassert.c
	filter.c
	stm32f4_discovery.c
	stm32f4_discovery_lis302dl.c	
)

set(ELKA_LINK_LIBS
  ${ELKA_LINK_LIBS} elka_utils
  PARENT_SCOPE
)
