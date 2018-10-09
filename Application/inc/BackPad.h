#ifndef __BACKPAD_H__
#define __BACKPAD_H__

#define BACK_MOTOR_ENBL_PORT   gpioPortE
#define BACK_MOTOR_ENBL_BIT    15
#define BACK_MOTOR_ENBL_MODE   gpioModePushPull

#define BACK_MOTOR_PHASE_PORT   gpioPortE
#define BACK_MOTOR_PHASE_BIT   10
#define BACK_MOTOR_PHASE_MODE   gpioModePushPull

#define BACK_MOTOR_DECAY_PORT   gpioPortE
#define BACK_MOTOR_DECAY_BIT    11
#define BACK_MOTOR_DECAY_MODE   gpioModePushPull

#define BACK_MOTOR_FAULT_PORT   gpioPortE
#define BACK_MOTOR_FAULT_BIT    12
#define BACK_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define BACK_MOTOR_RESET_PORT   gpioPortE
#define BACK_MOTOR_RESET_BIT    13//9
#define BACK_MOTOR_RESET_MODE   gpioModePushPull

#define BACK_MOTOR_AT_MID     0
#define BACK_MOTOR_AT_BOTTOM  1
#define BACK_MOTOR_AT_TOP     2
//靠背电动缸最大运行时间ms
#define BACK_MOTOR_MAX_POSITION 1600



#define STATE_RUN_BACK_DOWN   0
#define STATE_RUN_BACK_UP     1
#define STATE_BACK_IDLE   2

#define BACK_MOTOR_TIMER           TIMER3
#define BACK_MOTOR_TIMER_CHANNEL   1
#define BACK_MOTOR_ROUTE_EN        TIMER_ROUTE_CC1PEN
#define BACK_MOTOR_ROUTE_LOCATION  TIMER_ROUTE_LOCATION_LOC0


#define BACK_MOTOR_PRESCALE        timerPrescale4
#define BACK_MOTOR_DEFAULT_TOP     131

#define BACK_MOTOR_Timer_CCInit     \
{                                   \
    timerEventEveryEdge,            \
    timerEdgeBoth,                  \
    timerPRSSELCh0,                 \
    timerOutputActionNone,          \
    timerOutputActionNone,          \
    timerOutputActionToggle,        \
    timerCCModePWM,                 \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
} 

#define BACK_MOTOR_Timer_Init      \
{                                   \
    true,                           \
    true,                           \
    BACK_MOTOR_PRESCALE,            \
    timerClkSelHFPerClk,            \
    false,                          \
    false,                          \
    timerInputActionNone,           \
    timerInputActionNone,           \
    timerModeUp,                    \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
}

enum
{
  BACK_MOTOR_POWER_ON, 
  BACK_MOTOR_POWER_OFF 
};
enum
{
  BACK_MOTOR_GO_UP, 
  BACK_MOTOR_GO_DOWN 
};

enum
{
 BACK_MOTOR_CURRENT_HIGH,
 BACK_MOTOR_CURRENT_LOW
};

enum
{
 BACK_MOTOR_NORMAL,
 BACK_MOTOR_FAIL
};
//int BackPower_On(void);
void BackPower_Off(void);

#define BACK_SET_VOLTAGE    260000 //25V

void BackMotor_Initial_IO(void);
void BackMotor_Up(void);
void BackMotor_Down(void);
void BackMotor_Break(void);
void BackMotor_Reset(void);
void BackMotor_Reset_Cancel(void);
int BackMotor_Get_Fault(void);
void BackMotor_10ms_int(void);
void BackMotor_Proce(void);
unsigned int BackMotor_Get_Position(void);
unsigned char BackMotor_Control(unsigned char nFinalBackPadMotorState);
int BackMotor_GetDirection(void);
int BackMotor_GetPower(void);
void currentBackPadMotorState_reset(void);









#endif
