# src/modules
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}/inc/modules
)

add_library(elka_modules STATIC
	sensfusion6.c
	stabilizer.c
)

set(ELKA_LINK_LIBS
  "${ELKA_LINK_LIBS} elka_modules"
)
