/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * motors.c - Motor driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 */

#include <stdbool.h>

#include "motors.h"

// ST lib includes
#include "stm32f4xx_conf.h"

//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

// HW defines
/*
#define MOTORS_GPIO_TIM_PERIF     RCC_APB1Periph_TIM3
#define MOTORS_GPIO_TIM_M1_2      TIM3
#define MOTORS_GPIO_TIM_M1_2_DBG  DBGMCU_TIM3_STOP //timer counter stopped when core is halted
#define MOTORS_REMAP              GPIO_PartialRemap_TIM3 //remap not avalable on 48 pin package?


#define MOTORS_GPIO_TIM_M3_4_PERIF  RCC_APB1Periph_TIM4
#define MOTORS_GPIO_TIM_M3_4        TIM4
#define MOTORS_GPIO_TIM_M3_4_DBG    DBGMCU_TIM4_STOP


#define MOTORS_GPIO_TIM_M5_6_PERIF  RCC_APB1Periph_TIM2
#define MOTORS_GPIO_TIM_M5_6        TIM2
#define MOTORS_GPIO_TIM_M5_6_DBG    DBGMCU_TIM2_STOP

#define MOTORS_GPIO_PERIF         RCC_APB2Periph_GPIOB
#define MOTORS_GPIO_PORT          GPIOB
#define MOTORS_GPIO_M1            GPIO_Pin_1 // T3_CH4
#define MOTORS_GPIO_M2            GPIO_Pin_0 // T3_CH3
#define MOTORS_GPIO_M3            GPIO_Pin_9 // T4_CH4
#define MOTORS_GPIO_M4            GPIO_Pin_8 // T4_CH3
*/

/* Utils Conversion macro */
#define C_BITS_TO_16(X) ((X)<<(16-MOTORS_PWM_BITS))
#define C_16_TO_BITS(X) ((X)>>(16-MOTORS_PWM_BITS)&((1<<MOTORS_PWM_BITS)-1))

static bool isInit=false;

/* Public functions */

//Initialization. Will set all motors ratio to 0%
void motorsInit()
{
  //Motor pins  chosen
  // PA3 - Tim2,Ch4 - Motor frontleft
  // PA5 - Tim8, Ch1 - Motor rearleft
  // PA6 - Tim3, Ch1 - Motor rearright
  // PA7 - Tim3, Ch2 - Motor frontright

  //if (isInit)
  //  return;

  uint16_t PrescalerValue = 0;
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

  //Enable gpio and the timer
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* GPIOB Configuration: TIM3 CH1 (PB4) and TIM2 CH2 (PB4) */
/*  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);*/

  /* GPIOA Configuration:   */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect TIM3 pins to AF2 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

  PrescalerValue = (uint16_t) ((SystemCoreClock) / 2 / 1000000) - 1;

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = 2000;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue*2;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period = 2000;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period = 2000;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Tim8, Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);


    /* PWM1 Mode configuration: Tim2, Channel 4 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);


    /* PWM1 Mode configuration: Tim3, Channel 1,2 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);




    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_CtrlPWMOutputs (TIM8, ENABLE);

    /* TIM3 enable counter */
    TIM_Cmd(TIM8, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

  
  isInit = true;
}

void motorsSetRatio(int id, uint16_t ratio)
{

	  //Motor pins  chosen
	  // PA3 - Tim2,Ch4 - Motor frontleft
	  // PA5 - Tim8, Ch1 - Motor rearleft
	  // PA6 - Tim3, Ch1 - Motor rearright
	  // PA7 - Tim3, Ch2 - Motor frontright

	switch(id) {

	    case MOTOR_LEFTFRONT:
	      //TIM_SetCompare3(MOTORS_GPIO_TIM_M3_4, C_16_TO_BITS(ratio));
	      TIM_SetCompare4(TIM2, C_16_TO_BITS(ratio));
	      break;
	    case MOTOR_LEFTREAR:
	      TIM_SetCompare1(TIM8, C_16_TO_BITS(ratio));
	      break;
	    case MOTOR_RIGHTFRONT:
	      TIM_SetCompare2(TIM3, C_16_TO_BITS(ratio));
	      break;
	    case MOTOR_RIGHTREAR:
	      TIM_SetCompare1(TIM3, C_16_TO_BITS(ratio));
	      break;
 }

  return;
}





#ifdef MOTOR_RAMPUP_TEST
// FreeRTOS Task to test the Motors driver with a rampup of each motor alone.
void motorsTestTask(void* params)
{
  int step=0;
  float rampup = 0.01;

  motorsSetupMinMaxPos();
  motorsSetRatio(MOTOR_LEFT, 1*(1<<16) * 0.0);
  motorsSetRatio(MOTOR_REAR, 1*(1<<16) * 0.0);
  motorsSetRatio(MOTOR_RIGHT, 1*(1<<16) * 0.0);
  motorsSetRatio(MOTOR_FRONT, 1*(1<<16) * 0.0);
  vTaskDelay(M2T(1000));

  while(1)
  {
    vTaskDelay(M2T(100));

    motorsSetRatio(MOTOR_LEFT, 1*(1<<16) * rampup);
    motorsSetRatio(MOTOR_REAR, 1*(1<<16) * rampup);
    motorsSetRatio(MOTOR_RIGHT, 1*(1<<16) * rampup);
    motorsSetRatio(MOTOR_FRONT, 1*(1<<16) * rampup);

    rampup += 0.001;
    if (rampup >= 0.1)
    {
      if(++step>3) step=0;
      rampup = 0.01;
    }
  }
}
#else
// FreeRTOS Task to test the Motors driver
void motorsTestTask(void* params)
{
  static const int sequence[] = {0.1*(1<<16), 0.15*(1<<16), 0.2*(1<<16), 0.25*(1<<16)};
  int step=0;

  //Wait 3 seconds before starting the motors
  vTaskDelay(3000);

  while(1) {
    motorsSetRatio(MOTOR_LEFT, sequence[step%4]);
    motorsSetRatio(MOTOR_REAR, sequence[(step+1)%4]);
    motorsSetRatio(MOTOR_RIGHT, sequence[(step+2)%4]);
    motorsSetRatio(MOTOR_FRONT, sequence[(step+3)%4]);

    if(++step>3) step=0;

    vTaskDelay(1000);
  }
}
#endif

