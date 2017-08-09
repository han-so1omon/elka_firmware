# src/FreeRTOS
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}/inc/FreeRTOS
  ${PROJECT_SOURCE_DIR}/inc/FreeRTOS/GCC/ARM_CM4F
)

add_library(FreeRTOS STATIC
	croutine.c
	list.c
	queue.c
	tasks.c
	timers.c
	GCC/ARM_CM4F/port.c
	MemMang/heap_1.c
)

set(ELKA_LINK_LIBS
  ${ELKA_LINK_LIBS} FreeRTOS
  PARENT_SCOPE
)
