#ifndef __HOT_FLEX_H__
#define __HOT_FLEX_H__

//#define FLEX_AUTO  1
//#define FLEX_MANUAL 0

#define FLEX_MOTOR_STOP   0  
#define FLEX_TO_OUT       1
#define FLEX_TO_IN        2
#define FLEX_TO_AUTO      3

#define FLEX_ON  0
#define FLEX_OFF 1

#define FLEX_AT_IN  1
#define FLEX_AT_OUT 0
#define FLEX_AT_MID 2
#define FLEX_AT_SMALL_ANGLE 3

#define FOOT_SWITCH_ON		1 //≈ˆµΩΩ≈¡À
#define FOOT_SWITCH_OFF		0

#define FLEX_MOTOR_CURRENT_1A     0
#define FLEX_MOTOR_CURRENT_15A    1
#define FLEX_MOTOR_CURRENT_2A     2
#define FLEX_MOTOR_CURRENT_25A   3
#define FLEX_MOTOR_CURRENT_3A     4
#define FLEX_MOTOR_CURRENT_35A   5
#define FLEX_MOTOR_CURRENT_4A     6
#define FLEX_MOTOR_CURRENT_5A     7

//void Flex_SetDirection(unsigned int flexDirection);
unsigned char Flex_GetDirection(void);
void Flex_SetPower(unsigned char power);
unsigned char Flex_GetPower(void);
void Flex_SetMode(unsigned int mode);
unsigned char Flex_GetMode(void);
unsigned char Flex_GetCurrent(void);
//void Flex_SetCurrent(unsigned char current);
unsigned char Flex_GetCurrent(void);
void FlexMotorFollowingFood(void);
bool FlexMotorGetEnable(void);
void FlexMotorSetEnable(void);
void FlexMotorSetDisable(void);
void Flex_SetStatus(unsigned char status);

void Flex_SetDisableAngle(bool disable);
unsigned char Flex_GetDisableAngle();

unsigned char Flex_ControlIn(unsigned int current);
unsigned char Flex_ControlOut(unsigned int current);
void Flex_ControlStop(void);

void Flex_100msInt(void);
bool GetFootStatus(void);

bool  GetFood(void);
unsigned char Flex_GetStatus(void);

#endif

