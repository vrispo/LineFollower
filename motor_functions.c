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
