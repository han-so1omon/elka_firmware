# Install script for directory: /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "DEBUG")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/src/drivers/cmake_install.cmake")
  include("/home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/src/elka_hal/cmake_install.cmake")
  include("/home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/src/FreeRTOS/cmake_install.cmake")
  include("/home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/src/modules/cmake_install.cmake")
  include("/home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/src/STM32F4xx_StdPeriph_Driver/cmake_install.cmake")
  include("/home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/src/utils/cmake_install.cmake")

endif()

