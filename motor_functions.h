/*
 * motor_functions.h
 *
 *  Created on: 16/nov/2018
 *      Author: Veronica
 */

#ifndef MOTOR_FUNCTIONS_H_
#define MOTOR_FUNCTIONS_H_

#include "ee.h"

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "kernel/sem/inc/ee_sem.h"
#include "kernel/sem/inc/ee_api.h"
#include "ee_api.h"

/*public functions-------------------*/

/*lead left motor*/
void breakleft(void);
void folleleft(void);
void forwardleft(void);
void backwardleft(void);

/*lead right motor*/
void breakright(void);
void folleright(void);
void forwardright(void);
void backwardright(void);

/*utils*/
void EnableMotors(void);
void DisableMotors(void);

void InitMotors(void);
void PWM_Config_and_En(void);

#endif /* MOTOR_FUNCTIONS_H_ */
