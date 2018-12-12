/*
 * motor_functions.c
 *
 *  Created on: 16/nov/2018
 *      Author: Veronica
 */
#include "ee.h"

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "kernel/sem/inc/ee_sem.h"
#include "kernel/sem/inc/ee_api.h"
#include "ee_api.h"

#include "motor_functions.h"

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

uint16_t CCR1_Val = 333;
uint16_t CCR2_Val = 249;
uint16_t CCR3_Val = 166;
uint16_t CCR4_Val = 83;
uint16_t PrescalerValue = 0;

void breakleft(){
	 /*pin 14-15 a 11*/
	 GPIO_SetBits(GPIOD, GPIO_Pin_14|GPIO_Pin_15);
}

void folleleft(){
	 /*pin 14-15 a 00*/
	 GPIO_ResetBits(GPIOD, GPIO_Pin_14|GPIO_Pin_15);
}

void forwardleft(){
	 /*pin 14-15 a 10*/
	 GPIO_SetBits(GPIOD, GPIO_Pin_15);
	 GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}
void backwardleft(){
	 /*pin 14-15 a 01*/
	 GPIO_SetBits(GPIOD, GPIO_Pin_14);
	 GPIO_ResetBits(GPIOD, GPIO_Pin_15);
}

/*lead right motor*/
void breakright(){
	 /*pin 12-13 a 11*/
	 GPIO_SetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13);
}

void folleright(){
	 /*pin 12-13 a 00*/
	 GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13);
}

void forwardright(){
	 /*pin 12-13 a 10*/
	 GPIO_SetBits(GPIOD, GPIO_Pin_13);
	 GPIO_ResetBits(GPIOD, GPIO_Pin_12);
}
void backwardright(){
	 /*pin 12-13 a 01*/
	 GPIO_SetBits(GPIOD, GPIO_Pin_12);
	 GPIO_ResetBits(GPIOD, GPIO_Pin_13);
}

void EnableMotors(){
	GPIO_SetBits(GPIOD, GPIO_Pin_8);
}

void DisableMotors(){
	GPIO_ResetBits(GPIOD, GPIO_Pin_8);
}

/**
  * @brief  Configure and enable the PWM Ouput Channels.
  * @param  None
  * @retval None
*/
void InitMotors(){
	/*Pin to lead the motors: Pin12-13 for motor dx - Pin14-15 for sx motor*/
	/*Pin to enable motor driver: PinD8*/
	/*
	 * pin 13-15 bit1 msb
	 * pin 12-14 bit0 lsb
	 * comandi motore:
	 * 00:folle
	 * 01:indietro
	 * 10:avanti
	 * 11:bloccato
	 * */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	DisableMotors();

	//forwardleft();
	//forwardright();
}

void PWM_Config_and_En()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* GPIOC and GPIOB clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Connect TIM3 pins to AF2 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);

	  /* -----------------------------------------------------------------------
	    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles.

	    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
	    since APB1 prescaler is different from 1.
	      TIM3CLK = 2 * PCLK1
	      PCLK1 = HCLK / 4
	      => TIM3CLK = HCLK / 2 = SystemCoreClock /2

	    To get TIM3 counter clock at 28 MHz, the prescaler is computed as follows:
	       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	       Prescaler = ((SystemCoreClock /2) /28 MHz) - 1

	    To get TIM3 output clock at 30 KHz, the period (ARR)) is computed as follows:
	       ARR = (TIM3 counter clock / TIM3 output clock) - 1
	           = 665

	    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
	    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
	    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
	    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%

	    Note:
	     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
	     function to update SystemCoreClock variable value. Otherwise, any configuration
	     based on this variable will be incorrect.
	  ----------------------------------------------------------------------- */

	  /* Compute the prescaler value */
	  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 2000000) - 1;

	  /* Time base configuration */
	  TIM_TimeBaseStructure.TIM_Period = 665;
	  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	  /* PWM1 Mode configuration: Channel1 */
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	  /* PWM1 Mode configuration: Channel2 */
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;

	  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	  TIM_ARRPreloadConfig(TIM3, ENABLE);

	  /* TIM3 enable counter */
	  TIM_Cmd(TIM3, ENABLE);

}
