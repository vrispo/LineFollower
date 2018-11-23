/* ###*B*###
 * ERIKA Enterprise - a tiny RTOS for small microcontrollers
 *
 * Copyright (C) 2002-2013  Evidence Srl
 *
 * This file is part of ERIKA Enterprise.
 *
 * ERIKA Enterprise is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation,
 * (with a special exception described below).
 *
 * Linking this code statically or dynamically with other modules is
 * making a combined work based on this code.  Thus, the terms and
 * conditions of the GNU General Public License cover the whole
 * combination.
 *
 * As a special exception, the copyright holders of this library give you
 * permission to link this code with independent modules to produce an
 * executable, regardless of the license terms of these independent
 * modules, and to copy and distribute the resulting executable under
 * terms of your choice, provided that you also meet, for each linked
 * independent module, the terms and conditions of the license of that
 * module.  An independent module is a module which is not derived from
 * or based on this library.  If you modify this code, you may extend
 * this exception to your version of the code, but you are not
 * obligated to do so.  If you do not wish to do so, delete this
 * exception statement from your version.
 *
 * ERIKA Enterprise is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License version 2 for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 along with ERIKA Enterprise; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301 USA.
 * ###*E*### */

#include "ee.h"

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "kernel/sem/inc/ee_sem.h"
#include "kernel/sem/inc/ee_api.h"
#include "ee_api.h"

#include <stdio.h>	//Serial port debug only

#include "motor_functions.h"

#define SENSOR_START 0
#define SENSOR_INIT 1
#define SENSOR_WAIT 2
#define SENSOR_READ 3

#define DELTA_WAIT 1	//time to wait
#define LIGHT_THRESHOLD 2	//(ms) threshold to separe white and black sensor readings

 /*Global Variables-------------------------------------------------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
double system_time;	//System time in ms

//Global variables for sensor reading
GPIO_InitTypeDef GPIO_InitStructure_LightSensors[8];

double led_ms[8]={0,0,0,0,0,0,0,0};		//ms value of each sensor
uint8_t led_flags[8]={0,0,0,0,0,0,0,0};	//flag to signal the successful reading of each flag

uint8_t sensor_mode = 0;
double sensor_up_time;	//Time at which sensors pin have been setted to HIGH (used to hold up at least 10ms)
double reference_time;	//Time at which sensors read starts (used to extimate correctly the delta value)

double delta_sensor[8];
SemType delta_sensor_sem = STATICSEM(1);	//Semaphore to protect delta_sensor
/*-------------------------------------------------------------------------------------------------------------------*/

//DEBUGGING SERIAL FUNCTION
console_out(char* str)
{
	EE_UINT8 i = 0;

	while (str[i] != '\0') {
		USART_SendData(EVAL_COM1, (uint8_t) str[i++]);
		/* Loop until the end of transmission */
		while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET);
	}
}

/*
 * OUR FUNCTIONS start
 */

 /*init line sensor*/
 void InitLineSensor(){
		GPIO_InitTypeDef GPIO_InitStructure;

		/*inizializzazione pin PA9 on/off led sensore luminoso*/
/*
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
*/

		//Inizializzazione pin sensori
		GPIO_InitStructure_LightSensors[0].GPIO_Pin=GPIO_Pin_1;
		GPIO_InitStructure_LightSensors[0].GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStructure_LightSensors[0].GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure_LightSensors[0].GPIO_Speed=GPIO_Speed_100MHz;
		GPIO_InitStructure_LightSensors[0].GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOD, &GPIO_InitStructure_LightSensors[0]);

		GPIO_InitStructure_LightSensors[1].GPIO_Pin=GPIO_Pin_3;
		GPIO_InitStructure_LightSensors[1].GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStructure_LightSensors[1].GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure_LightSensors[1].GPIO_Speed=GPIO_Speed_100MHz;
		GPIO_InitStructure_LightSensors[1].GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOD, &GPIO_InitStructure_LightSensors[1]);

		GPIO_InitStructure_LightSensors[2].GPIO_Pin=GPIO_Pin_4;
		GPIO_InitStructure_LightSensors[2].GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStructure_LightSensors[2].GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure_LightSensors[2].GPIO_Speed=GPIO_Speed_100MHz;
		GPIO_InitStructure_LightSensors[2].GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure_LightSensors[2]);

		GPIO_InitStructure_LightSensors[3].GPIO_Pin=GPIO_Pin_5;
		GPIO_InitStructure_LightSensors[3].GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStructure_LightSensors[3].GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure_LightSensors[3].GPIO_Speed=GPIO_Speed_100MHz;
		GPIO_InitStructure_LightSensors[3].GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOD, &GPIO_InitStructure_LightSensors[3]);

		GPIO_InitStructure_LightSensors[4].GPIO_Pin=GPIO_Pin_6;
		GPIO_InitStructure_LightSensors[4].GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStructure_LightSensors[4].GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure_LightSensors[4].GPIO_Speed=GPIO_Speed_100MHz;
		GPIO_InitStructure_LightSensors[4].GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure_LightSensors[4]);

		GPIO_InitStructure_LightSensors[5].GPIO_Pin=GPIO_Pin_7;
		GPIO_InitStructure_LightSensors[5].GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStructure_LightSensors[5].GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure_LightSensors[5].GPIO_Speed=GPIO_Speed_100MHz;
		GPIO_InitStructure_LightSensors[5].GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOD, &GPIO_InitStructure_LightSensors[5]);

		GPIO_InitStructure_LightSensors[6].GPIO_Pin=GPIO_Pin_10;
		GPIO_InitStructure_LightSensors[6].GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStructure_LightSensors[6].GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure_LightSensors[6].GPIO_Speed=GPIO_Speed_100MHz;
		GPIO_InitStructure_LightSensors[6].GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC, &GPIO_InitStructure_LightSensors[6]);

		GPIO_InitStructure_LightSensors[7].GPIO_Pin=GPIO_Pin_12;
		GPIO_InitStructure_LightSensors[7].GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStructure_LightSensors[7].GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure_LightSensors[7].GPIO_Speed=GPIO_Speed_100MHz;
		GPIO_InitStructure_LightSensors[7].GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC, &GPIO_InitStructure_LightSensors[7]);
 }

 /*uint32_t LineSensors_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t index){
	 uint32_t read_val = 0;

	 //set I/O output to 1
	 GPIO_SetBits(GPIOx, GPIO_Pin);

	 //mantieni per almeno 10 micros
	 //Delay(10);

	 //set I/O as input pin
	 GPIO_InitStructure_LightSensors[index].GPIO_Mode=GPIO_Mode_IN;
	 GPIO_Init(GPIOx, &GPIO_InitStructure_LightSensors[index]);

	 //read
	 while(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) != (uint8_t)Bit_RESET){
		 read_val++;
		 //Delay(1);
	 }

	 //set I/O as output pin
	 GPIO_InitStructure_LightSensors[index].GPIO_Mode=GPIO_Mode_OUT;
	 GPIO_Init(GPIOx, &GPIO_InitStructure_LightSensors[index]);

	 return read_val;
 }*/

 double my_get_systime(){
 	return system_time;
 }

 void InitDataStruct(){
 	int i;
 	for(i = 0; i < 8; i++){
 		delta_sensor[i] = 0;
 	}
 }
 void read_task_init(){
	 int i;	//counter

	 //Clock for SYSCFG
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	 //For each used EXTI line tell wich pin is selected
	 SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);
	 SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource3);
	 SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource4);
	 SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource5);
	 SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource6);
	 SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource7);
	 SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource10);
	 SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource12);

	 //External interrupt settings
	 EXTI_InitTypeDef EXTI_InitStruct;
	 EXTI_InitStruct.EXTI_Line = EXTI_Line1;	//Pin D1 have interrupt line 1
	 EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	 EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;	//Generate interrupt on falling edge //TODO: check
	 EXTI_Init(&EXTI_InitStruct);

	 EXTI_InitStruct.EXTI_Line = EXTI_Line3;	//Pin D3 have interrupt line 3
	 EXTI_Init(&EXTI_InitStruct);

	 EXTI_InitStruct.EXTI_Line = EXTI_Line4;	//Pin B4 have interrupt line 4
	 EXTI_Init(&EXTI_InitStruct);

	 EXTI_InitStruct.EXTI_Line = EXTI_Line5;	//Pin D5 have interrupt line 5
	 EXTI_Init(&EXTI_InitStruct);

	 EXTI_InitStruct.EXTI_Line = EXTI_Line6;	//Pin B6 have interrupt line 6
	 EXTI_Init(&EXTI_InitStruct);

	 EXTI_InitStruct.EXTI_Line = EXTI_Line7;	//Pin D7 have interrupt line 7
	 EXTI_Init(&EXTI_InitStruct);

	 EXTI_InitStruct.EXTI_Line = EXTI_Line10;	//Pin C10 have interrupt line 10
	 EXTI_Init(&EXTI_InitStruct);

	 EXTI_InitStruct.EXTI_Line = EXTI_Line12;	//Pin C12 have interrupt line 12
	 EXTI_Init(&EXTI_InitStruct);

	 NVIC_InitTypeDef NVIC_InitStruct;
	 NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;	//Line 1 have this IRQ channel
	 NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;	//Most important interrupt
	 NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	 NVIC_Init(&NVIC_InitStruct);

	 NVIC_InitStruct.NVIC_IRQChannel = EXTI3_IRQn;	//Line 3 have this IRQ channel
	 NVIC_Init(&NVIC_InitStruct);

	 NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;	//Line 4 have this IRQ channel
	 NVIC_Init(&NVIC_InitStruct);

	 NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;	//Line 5, 6 and 7 have this IRQ channel
	 NVIC_Init(&NVIC_InitStruct);

	 NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;	//Line 10 and 12 have this IRQ channel
	 NVIC_Init(&NVIC_InitStruct);
 }

 //HANDLERS FOR PIN INTERRUPTS
 void EXTI1_IRQHandler(void){
	 console_out("+H1+");
	 //Checks the line is correct
	 if(EXTI_GetITStatus(EXTI_Line1)){
		 //PIN 1
		 //set time and flag for pin 1 low
		 if(led_flags[5]==0){
			 //led_ms[5]=EE_systick_get_value();
			 led_ms[5] = my_get_systime();
			 led_flags[5]=1;
		 }
	 }
 }

 void EXTI3_IRQHandler(void){
	 //Checks the line is correct
	 if(EXTI_GetITStatus(EXTI_Line3)){
		 console_out("+H3+");
		 //PIN 3
		 //set time and flag for pin 3 low
		 if(led_flags[4]==0){
			 led_ms[4] = my_get_systime();
			 led_flags[4]=1;
		 }
	 }
 }

 void EXTI4_IRQHandler(void){
	 //Checks the line is correct
	 if(EXTI_GetITStatus(EXTI_Line4)){
		 console_out("+H4+");
		 //PIN 1
		 //set time and flag for pin 4 low
		 if(led_flags[1]==0){
			 led_ms[1] = my_get_systime();
			 led_flags[1]=1;
		 }
	 }
 }

 //Handler for the interrupts from pins 5 to 9
 void EXTI9_5_IRQHandler(void){
	 //Checks the line where comes the interrupt
	 if(EXTI_GetITStatus(EXTI_Line5)){
		 console_out("+H5+");
		 //Pin 5
		 //set time and flag for pin 5 low
		 if(led_flags[3]==0){
			 led_ms[0] = my_get_systime();
			 led_flags[3]=1;
		 }
	 }
	 if(EXTI_GetITStatus(EXTI_Line6)){
		 console_out("+H6+");
		 //Pin6
		 //do something when pin6 is low (set time and set flag)
		 if(led_flags[0]==0){
			 led_ms[0] = my_get_systime();
			 led_flags[0]=1;
		 }
	 }
	 if(EXTI_GetITStatus(EXTI_Line7)){
		 console_out("+H7+");
		 //Pin 7
		 //set time and flag for pin 7 low
		 if(led_flags[2]==0){
			 led_ms[2] = my_get_systime();
			 led_flags[2]=1;
		 }
	 }
 }

 void EXTI15_10_IRQHandler(void){
	 //Checks the line is correct
	 if(EXTI_GetITStatus(EXTI_Line10)){
		 console_out("+H10+");
		 //PIN 10
		 //set time and flag for pin 10 low
		 if(led_flags[7]==0){
			 led_ms[7] = my_get_systime();
			 led_flags[7]=1;
		 }
	 }
	 if(EXTI_GetITStatus(EXTI_Line12)){
		 console_out("+H12+");
		 //PIN 12
		 //set time and flag for pin 12 low
		 if(led_flags[6]==0){
			 //led_ms[6]=EE_systick_get_value();
			 led_ms[6] = my_get_systime();
			 led_flags[6]=1;
		 }
	 }
 }

/*
 * OUR FUNCTIONS end
 */

/*
 * SysTick ISR2
 */
ISR2(systick_handler)
{
	CounterTick(myCounter);	//Count the system ticks to wake up expired alarms
}

//alarm callback to manage the time in ms
void my_system_time(){
	system_time++;
}

/**
 * This task manage the light sensors
 */
TASK(CheckRead){
	int i;
	double delta;
	int end;	//end flag
	
	char str[64];
	console_out("*READ_TASK*");

	if(sensor_mode == SENSOR_START){
		//INITIAL SENSOR SETUP
		console_out("SEN_S--");
		InitLineSensor();	//Setup pins
		read_task_init();	//Setup interrupt handlers
		sensor_mode = SENSOR_INIT;
	}else if(sensor_mode == SENSOR_INIT){
		console_out("SEN_I--");

		//Put high all sensor pins
		GPIO_SetBits(GPIOD, GPIO_Pin_1);
		GPIO_SetBits(GPIOD, GPIO_Pin_3);
		GPIO_SetBits(GPIOB, GPIO_Pin_4);
		GPIO_SetBits(GPIOD, GPIO_Pin_5);
		GPIO_SetBits(GPIOB, GPIO_Pin_6);
		GPIO_SetBits(GPIOD, GPIO_Pin_7);
		GPIO_SetBits(GPIOC, GPIO_Pin_10);
		GPIO_SetBits(GPIOC, GPIO_Pin_12);

		console_out("SEN_I gpio_set");

		//Save system time
		sensor_up_time = my_get_systime();

		//Init pin readings and flags
		 for(i = 0; i < 8; i++){
			 led_ms[i] = 0;
			 led_flags[i] = 0;
		 }

		 sensor_mode = SENSOR_WAIT;	//Put task in wait mode
		 sprintf(str, "--%f SEN_I 2", sensor_up_time);
		 console_out(str);
	}else if(sensor_mode == SENSOR_WAIT){
		sprintf(str, "---%f SEN_W", my_get_systime());
		console_out(str);

		double actual_systick = my_get_systime();
		delta = actual_systick - sensor_up_time;	//Compute elapsed time from sensor pins up

		if(delta >= DELTA_WAIT){
			sprintf(str, "--%f SET_SENSOR_INPUT", my_get_systime());
			console_out(str);

			reference_time = my_get_systime();

			//Set all pins as input
			GPIO_InitStructure_LightSensors[0].GPIO_Mode=GPIO_Mode_IN;
			GPIO_Init(GPIOD, &GPIO_InitStructure_LightSensors[0]);

			GPIO_InitStructure_LightSensors[1].GPIO_Mode=GPIO_Mode_IN;
			GPIO_Init(GPIOD, &GPIO_InitStructure_LightSensors[1]);

			GPIO_InitStructure_LightSensors[2].GPIO_Mode=GPIO_Mode_IN;
			GPIO_Init(GPIOB, &GPIO_InitStructure_LightSensors[2]);

			GPIO_InitStructure_LightSensors[3].GPIO_Mode=GPIO_Mode_IN;
			GPIO_Init(GPIOD, &GPIO_InitStructure_LightSensors[3]);

			GPIO_InitStructure_LightSensors[4].GPIO_Mode=GPIO_Mode_IN;
			GPIO_Init(GPIOB, &GPIO_InitStructure_LightSensors[4]);

			GPIO_InitStructure_LightSensors[5].GPIO_Mode=GPIO_Mode_IN;
			GPIO_Init(GPIOD, &GPIO_InitStructure_LightSensors[5]);

			GPIO_InitStructure_LightSensors[6].GPIO_Mode=GPIO_Mode_IN;
			GPIO_Init(GPIOC, &GPIO_InitStructure_LightSensors[6]);

			GPIO_InitStructure_LightSensors[7].GPIO_Mode=GPIO_Mode_IN;
			GPIO_Init(GPIOC, &GPIO_InitStructure_LightSensors[7]);

			sensor_mode = SENSOR_READ;
		}else{
			sprintf(str, "-%f SENSOR_HAVE_TO_HOLD", my_get_systime());
			console_out(str);
		}
	}else if(sensor_mode == SENSOR_READ){
		console_out("--SEN_R");
		end = 1;
		for(i = 0; i < 8; i++){
			if(led_flags[i] == 0){
				end = 0;
				break;
			}
		}
		/*TODO: if reference-actual>timeout restart the procedure and reset the time counter variable*/
		if(end == 1){
			//All sensors have returned value
			WaitSem(&delta_sensor_sem);
			for(i = 0; i < 8; i++){
				delta_sensor[i] = led_ms[i] - reference_time;
			}
			PostSem(&delta_sensor_sem);

			sensor_mode = SENSOR_INIT;
		}
	}
}

/**
 * This task controls the motor
 */
TASK(TaskMotorControl){
	double sensor_time[8];	//Local copy of decay time of light sensors
	int i;	//counter
	int left, right;	//counters of left and right sensor that detects black

	left = right = 0;	//Initialize left and right black sensors counters

	console_out("*MOTOR_TASK*");

	//protect copy of delta_sensor to local sensor_time
	WaitSem(&delta_sensor_sem);
	for(int i=0; i < 8; i++)
		sensor_time[i] = delta_sensor[i];
	PostSem(&delta_sensor_sem);

	for(i = 0; i < 8; i++){
		if(sensor_time[i] > LIGHT_THRESHOLD){
			//Sensor detects black
			if(i < 4){
				//Left sensor
				left++;
			}else{
				right++;
			}
		}
	}

	if(left + right == 8){
		//All sensors detects black --> STOP (end of lap)
		breakleft();
		breakright();
	}else if(left > right){
		//Turn left
		forwardright();
		folleleft();
	}else if(right > left){
		//Turn right
		folleright();
		forwardleft();
	}else if(left == 0 && right == 0){
		//No info on the line --> STOP
		breakleft();
		breakright();
	}else{
		//Go straight forward
		forwardleft();
		forwardright();
	}
}

int main(void)
{
	/*
	 * Setup the microcontroller system.
	 * Initialize the Embedded Flash Interface, the PLL and update the
	 * SystemFrequency variable.
	 * For default settings look at:
	 * pkg/mcu/st_stm32_stm32f4xx/src/system_stm32f4xx.c
	 */
	SystemInit();

	/*Initialize Erika related stuffs*/
	EE_system_init();

	system_time = 0.0;

	/*Initialize systick */
	EE_systick_set_period(MILLISECONDS_TO_TICKS(1, SystemCoreClock));	//1 tick for each millisecond
	EE_systick_enable_int();
	EE_systick_start();

	//----------------------------------------
	/* USARTx configured as follow:
			- BaudRate = 115200 baud
			- Word Length = 8 Bits
			- One Stop Bit
			- No parity
			- Hardware flow control disabled (RTS and CTS signals)
			- Receive and transmit enabled
		*/
		USART_InitStructure.USART_BaudRate = 115200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		STM_EVAL_COMInit(COM1, &USART_InitStructure);

	//------------------------------
	console_out("*System init*");
	/*Setup motors */
	InitMotors();
	console_out("*0");
	PWM_Config_and_En();
	console_out("*1");
	//Enable motors
	EnableMotors();

	console_out("*2");
	//At the beginning motors are stopped
	breakright();
	console_out("*3");
	breakleft();
	console_out("*4");
	/*Start line sensor*/
	InitLineSensor();
	console_out("*5");
	InitDataStruct();

	console_out("*STRUCT_END");

	//Program cyclic alarm to periodically activate tasks*/
	SetRelAlarm(CheckReadAlarm, 10, 10);
	SetRelAlarm(IncrementTimeAlarm, 1, 1);//TODO: check the cycle value (1)
	SetRelAlarm(MotorControlAlarm, 10, 10);	//TODO: check the cycle value (1)

	console_out("**INIT END**");

	/* Forever loop: background activities (if any) should go here */
	for (;;);
}
