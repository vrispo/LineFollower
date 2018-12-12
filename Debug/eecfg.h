#ifndef EECFG_H
#define EECFG_H


#define RTDRUID_CONFIGURATOR_NUMBER 1278



/***************************************************************************
 *
 * Common defines ( CPU 0 )
 *
 **************************************************************************/

    /* TASK definition */
    #define EE_MAX_TASK 2
    #define CheckRead 0
    #define TaskMotorControl 1

    /* MUTEX definition */
    #define EE_MAX_RESOURCE 0U

    /* ALARM definition */
    #define EE_MAX_ALARM 3U
    #define IncrementTimeAlarm 0U
    #define CheckReadAlarm 1U
    #define MotorControlAlarm 2U

    /* SCHEDULING TABLE definition */
    #define EE_MAX_SCHEDULETABLE 0U

    /* COUNTER OBJECTS definition */
    #define EE_MAX_COUNTER_OBJECTS (EE_MAX_ALARM + EE_MAX_SCHEDULETABLE)

    /* COUNTER definition */
    #define EE_MAX_COUNTER 1U
    #define myCounter 0U

    /* APPMODE definition */
    #define EE_MAX_APPMODE 0U

    /* CPUs */
    #define EE_MAX_CPU 1
    #define EE_CURRENTCPU 0

    /* Number of isr 2 */
    #define EE_MAX_ISR2   6
    #define EE_MAX_ISR_ID 6

#ifndef __DISABLE_EEOPT_DEFINES__


/***************************************************************************
 *
 * User options
 *
 **************************************************************************/
#define __SEM__
#define __USE_SYSTICK__
#define __ADD_LIBS__


/***************************************************************************
 *
 * Automatic options
 *
 **************************************************************************/
#define __RTD_CYGWIN__
#define __STM32__
#define __STM32F4xx__
#define __CORTEX_MX__
#define __CORTEX_M4__
#define __GNU__
#define __FP__
#define __MULTI__
#define __ALARMS__
#define __FP_NO_RESOURCE__

#endif



/***************************************************************************
 *
 * ISR definition
 *
 **************************************************************************/
#define EE_CORTEX_MX_SYSTICK_ISR systick_handler
#define EE_CORTEX_MX_SYSTICK_ISR_PRI EE_ISR_PRI_1
#define EE_CORTEX_MX_EXTI1_ISR EXTI1_IRQHandler
#define EE_CORTEX_MX_EXTI1_ISR_PRI EE_ISR_PRI_2
#define EE_CORTEX_MX_EXTI3_ISR EXTI3_IRQHandler
#define EE_CORTEX_MX_EXTI3_ISR_PRI EE_ISR_PRI_2
#define EE_CORTEX_MX_EXTI4_ISR EXTI4_IRQHandler
#define EE_CORTEX_MX_EXTI4_ISR_PRI EE_ISR_PRI_2
#define EE_CORTEX_MX_EXTI9_5_ISR EXTI9_5_IRQHandler
#define EE_CORTEX_MX_EXTI9_5_ISR_PRI EE_ISR_PRI_2
#define EE_CORTEX_MX_EXTI15_10_ISR EXTI15_10_IRQHandler
#define EE_CORTEX_MX_EXTI15_10_ISR_PRI EE_ISR_PRI_2


/***************************************************************************
 *
 * Vector size defines
 *
 **************************************************************************/
    #define EE_ALARM_ROM_SIZE 3
    #define EE_CORTEX_MX_SYSTEM_TOS_SIZE 1


#endif

