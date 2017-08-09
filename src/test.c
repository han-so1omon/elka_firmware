/* Includes ------------------------------------------------------------------*/
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>

#include <stm32f4_discovery.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <FreeRTOSConfig.h>

int main(void) {
  GPIO_InitTypeDef GPIO_InitDef;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitDef.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
  GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIOC, &GPIO_InitDef);

  int i = 0;
  while(1) {
    if (!(i % 900000)) {
      //GPIOC->BSRRL = GPIO_Pin_2 | GPIO_Pin_3;
      //GPIOC->ODR ^= GPIO_Pin_2 | GPIO_Pin_3;
      GPIOC->ODR ^= GPIO_Pin_2 | GPIO_Pin_3;
    }
    i++;
  }

  return 0;
}
