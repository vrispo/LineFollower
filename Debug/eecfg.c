#include "ee.h"



/***************************************************************************
 *
 * Stack definition for CORTEX M0
 *
 **************************************************************************/


    const EE_UREG EE_std_thread_tos[EE_MAX_TASK+1] = {
        0U,	 /* dummy*/
        0U,	 /* CheckRead*/
        0U 	 /* TaskMotorControl*/
    };

    struct EE_TOS EE_cortex_mx_system_tos[EE_CORTEX_MX_SYSTEM_TOS_SIZE] = {
        {0} 	/* Task   (dummy), Task 0 (CheckRead), Task 1 (TaskMotorControl) */
    };

    EE_UREG EE_cortex_mx_active_tos = 0U; /* dummy */



/***************************************************************************
 *
 * Kernel ( CPU 0 )
 *
 **************************************************************************/
    /* Definition of task's body */
    DeclareTask(CheckRead);
    DeclareTask(TaskMotorControl);

    const EE_THREAD_PTR EE_hal_thread_body[EE_MAX_TASK] = {
        &FuncCheckRead,		/* thread CheckRead */
        &FuncTaskMotorControl 		/* thread TaskMotorControl */

    };

    /* ready priority */
    const EE_TYPEPRIO EE_th_ready_prio[EE_MAX_TASK] = {
        0x1U,		/* thread CheckRead */
        0x1U 		/* thread TaskMotorControl */
    };

    /* dispatch priority */
    const EE_TYPEPRIO EE_th_dispatch_prio[EE_MAX_TASK] = {
        0x1U,		/* thread CheckRead */
        0x1U 		/* thread TaskMotorControl */
    };

    /* thread status */
    #if defined(__MULTI__) || defined(__WITH_STATUS__)
        EE_TYPESTATUS EE_th_status[EE_MAX_TASK] = {
            EE_READY,
            EE_READY
        };
    #endif

    /* next thread */
    EE_TID EE_th_next[EE_MAX_TASK] = {
        EE_NIL,
        EE_NIL
    };

    EE_TYPEPRIO EE_th_nact[EE_MAX_TASK];
    /* The first stacked task */
    EE_TID EE_stkfirst = EE_NIL;

    /* The first task into the ready queue */
    EE_TID EE_rqfirst  = EE_NIL;

    /* system ceiling */
    EE_TYPEPRIO EE_sys_ceiling= 0x0000U;



/***************************************************************************
 *
 * Counters
 *
 **************************************************************************/
    EE_counter_RAM_type       EE_counter_RAM[EE_MAX_COUNTER] = {
        {0, -1}         /* myCounter */
    };



/***************************************************************************
 *
 * Alarms
 *
 **************************************************************************/
    /* Functions */
    void my_system_time(void);

    const EE_alarm_ROM_type   EE_alarm_ROM[EE_ALARM_ROM_SIZE] = {
        {0, EE_ALARM_ACTION_CALLBACK, 0, my_system_time},
        {0, EE_ALARM_ACTION_TASK    , CheckRead, NULL},
        {0, EE_ALARM_ACTION_TASK    , TaskMotorControl, NULL}
    };

    EE_alarm_RAM_type         EE_alarm_RAM[EE_MAX_ALARM];

