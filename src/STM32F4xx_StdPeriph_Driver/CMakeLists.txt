# src/STM32F4xx_StdPeriph_Driver
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}/inc/STM32F4xx_StdPeriph_Driver
	${PROJECT_SOURCE_DIR}/inc/Device/STM32F4xx
)

add_library(STM32F4xx_StdPeriph_Driver STATIC
	misc.c
	stm32f4xx_adc.c
	stm32f4xx_can.c
	stm32f4xx_crc.c
	stm32f4xx_cryp.c
	stm32f4xx_cryp_aes.c
	stm32f4xx_cryp_des.c
	stm32f4xx_cryp_tdes.c
	stm32f4xx_dac.c
	stm32f4xx_dbgmcu.c
	stm32f4xx_dcmi.c
	stm32f4xx_dma.c
	stm32f4xx_exti.c
	stm32f4xx_flash.c
	stm32f4xx_fsmc.c
	stm32f4xx_gpio.c
	stm32f4xx_hash.c
	stm32f4xx_hash_md5.c
	stm32f4xx_hash_sha1.c
	stm32f4xx_i2c.c
	stm32f4xx_it.c
	stm32f4xx_iwdg.c
	stm32f4xx_pwr.c
	stm32f4xx_rcc.c
	stm32f4xx_rng.c
	stm32f4xx_rtc.c
	stm32f4xx_sdio.c
	stm32f4xx_spi.c
	stm32f4xx_syscfg.c
	stm32f4xx_tim.c
	stm32f4xx_usart.c
	stm32f4xx_wwdg.c
	system_stm32f4xx.c
)

set(ELKA_LINK_LIBS
  ${ELKA_LINK_LIBS} STM32F4xx_StdPeriph_Driver
  PARENT_SCOPE
)
