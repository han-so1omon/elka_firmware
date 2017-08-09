# src/modules
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}/inc/modules
  ${PROJECT_SOURCE_DIR}/inc/drivers
)

add_library(elka_modules STATIC
	sensfusion6.c
	stabilizer.c
)

add_dependencies(elka_modules
  elka_drivers
  elka_hal
)

target_link_libraries(elka_modules
  m
  elka_drivers
  elka_hal
)

set(ELKA_LINK_LIBS
  ${ELKA_LINK_LIBS} elka_modules
  PARENT_SCOPE
)
