#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "LegMotor.h"
#include "Flex_Uart.h"
#include "EFM32_def.h"
#include "EFM32_types.h"

#define FLEX_START_ANGLE  200
static unsigned char nFlexMode;
static unsigned char nFlexDirection;
static unsigned char nFlexCurrent;
static bool FlexMotorEnable = false;
//角度禁止标志 当角度大于45度时，不允许伸出，以防顶翻机器
static bool angleFlexDisable = true;
static unsigned int w_FlexAdjStep;
static unsigned char nFlexStatus;
static volatile unsigned int w_Timer;
extern unsigned short nLegAngle;
extern bool  bAUTO1AirEnable;
static void Flex_SetDirection(unsigned int flexDirection);


//static unsigned char Flex_GetCurrent(void);
void Flex_SetDirection(unsigned int flexDirection)
{
   nFlexDirection = flexDirection;

}
unsigned char Flex_GetDirection(void)
{
   return(nFlexDirection);
}

void Flex_SetDisableAngle(bool disable)
{
  if(disable)
  {
    angleFlexDisable = true;
  }
  else
  {
    angleFlexDisable = false;
  }
}

unsigned char Flex_GetDisableAngle()
{
  return (unsigned char)(angleFlexDisable);
}

unsigned char Flex_GetCurrent(void)
{
   return(nFlexCurrent);
}

void Flex_SetCurrent(unsigned char current)
{
   nFlexCurrent = current;
}

void Flex_SetMode(unsigned int mode)
{
    nFlexMode = mode;
}
unsigned char Flex_GetMode(void)
{
    return(nFlexMode);
}

bool GetFootStatus(void)
{
  
   if(nFlexStatus&0x04)return 1;
   else return 0;
   
  
}
bool  GetFood(void)
{
  
  if(nFlexStatus&0x04)return 1;
  else return 0;
  
  
}

void FlexMotorFollowingFood(void)
{
  if(!FlexMotorEnable)
  {
    return;
  }
  if(LegPower_Get())
  {
    return;  //小腿上下电动缸还在运行中
  }
  //printf("s0\n");
  switch(w_FlexAdjStep)
  {
  case 0:
    {
   //   w_FlexAdjStep++;
       w_FlexAdjStep=1; //碰到脚向外走，
      if((nFlexStatus&0x04) == 0)  
      {  
      //   w_FlexAdjStep++; 
	   w_FlexAdjStep=2;//没有碰到脚,没有碰到脚向里走
      }
    }
    break;
  case 1://向外走
    if((nFlexStatus&0x03) ==  FLEX_AT_OUT) //处理个子太高，脚一直伸到行程开关位置处，此时停止找脚程序
    {           //向外走到底了
       //FlexMotorEnable = false;
       Flex_ControlStop();

    //   w_FlexAdjStep++;
         w_FlexAdjStep=2;//,没有碰到脚向里走
       w_Timer = 0;
       break;//add 20150827
    }
    if(nLegAngle < FLEX_START_ANGLE) //#define FLEX_START_ANGLE  220
    {               //向外走的时候没角度了
       FlexMotorEnable = false;
       Flex_ControlStop();
       break;
    }
    
    
    if(!(nFlexStatus &0x04))
    {  //向外走碰不到脚了 再向回来点
  //     w_FlexAdjStep++;
      w_FlexAdjStep=2;//,没有碰到脚向里走
       w_Timer = 0;
  
       Flex_ControlIn(FLEX_MOTOR_CURRENT_4A);
    }
    else
    {
      //向外走
      Flex_ControlOut(FLEX_MOTOR_CURRENT_5A);
    }
    break;
  case 2://向里走
   if(nFlexStatus&0x04)//向里走碰到脚
    {  
      // w_FlexAdjStep++;
           w_FlexAdjStep=3;
       w_Timer = 0;
       break;
    }
    if((nFlexStatus&0x03) ==  FLEX_AT_IN) 
    { //向里走到底了
       FlexMotorEnable = false;
       Flex_SetDirection(FLEX_MOTOR_STOP);
       break;
    }
    //向里走
    Flex_ControlIn(FLEX_MOTOR_CURRENT_4A);
    w_Timer = 0;
    break;
    
   case 3:  //碰到脚后 再向里走5秒
    Flex_ControlIn(FLEX_MOTOR_CURRENT_4A);
   /* if(bAUTO1AirEnable == TRUE)
    {
      if(w_Timer > 20) //&& (nFlexStatus&0x04 != 0))
      {
        FlexMotorEnable = false;
        Flex_ControlStop();
        // printf("s2\n");
      }
    }
    else*/
    {
      if(w_Timer > 20) //&& (nFlexStatus&0x04 != 0))
      {
        FlexMotorEnable = false;
        Flex_ControlStop();
        // printf("s2\n");
      }
    }
    
    
    if((nFlexStatus&0x03) ==  FLEX_AT_IN) 
    { //向里走到底了
       FlexMotorEnable = false;
       Flex_ControlStop();
       break;
    }
     break;
   default:
    FlexMotorEnable = false;
    Flex_ControlStop();
    break;
  }
}
/*
//自动找脚程序 
void FlexMotorFollowingFood(void)
{
  if(!FlexMotorEnable)
  {
    return;
  }
  if(LegPower_Get())
  {
    return;  //小腿上下电动缸还在运行中
  }
  //printf("s0\n");
  switch(w_FlexAdjStep)
  {
  case 0:
    {
      w_FlexAdjStep++;
      if((nFlexStatus&0x04) == 0)  
      {  
         w_FlexAdjStep++;  //没有碰到脚
      }
    }
    break;
  case 1://向外走
    if((nFlexStatus&0x03) ==  FLEX_AT_OUT) 
    {           //向外走到底了
       //FlexMotorEnable = false;
       Flex_ControlStop();
       //break;
       w_FlexAdjStep++;
       w_Timer = 0;
       break;//add 20150827
    }
    if(nLegAngle < FLEX_START_ANGLE) 
    {               //向外走的时候没角度了
       FlexMotorEnable = false;
       Flex_ControlStop();
       break;
    }
    
    
     if(!(nFlexStatus &0x04))
    {  //向外走碰不到脚了 再向回来点
       w_FlexAdjStep++;
       w_Timer = 0;
       //Flex_ControlStop();
       Flex_ControlIn(FLEX_MOTOR_CURRENT_4A);
    }
    else
    {
      //向外走
      Flex_ControlOut(FLEX_MOTOR_CURRENT_5A);
    }
    break;
  case 2://向里走
   if(nFlexStatus&0x04)
    {  
       w_FlexAdjStep++;
       w_Timer = 0;
       break;
    }
    if((nFlexStatus&0x03) ==  FLEX_AT_IN) 
    { //向里走到底了
       FlexMotorEnable = false;
       Flex_SetDirection(FLEX_MOTOR_STOP);
       break;
    }
    //向里走
    Flex_ControlIn(FLEX_MOTOR_CURRENT_4A);
    w_Timer = 0;
    break;
   case 3:  //碰到脚后 再向里走5秒
    Flex_ControlIn(FLEX_MOTOR_CURRENT_4A);
    if(bAUTO1AirEnable == TRUE)
    {
      if(w_Timer > 40) //&& (nFlexStatus&0x04 != 0))
      {
        FlexMotorEnable = false;
        Flex_ControlStop();
        // printf("s2\n");
      }
    }
    else
    {
      if(w_Timer > 20) //&& (nFlexStatus&0x04 != 0))
      {
        FlexMotorEnable = false;
        Flex_ControlStop();
        // printf("s2\n");
      }
    }
    
    
    if((nFlexStatus&0x03) ==  FLEX_AT_IN) 
    { //向里走到底了
       FlexMotorEnable = false;
       Flex_ControlStop();
       break;
    }
     break;
   default:
    FlexMotorEnable = false;
    Flex_ControlStop();
    break;
  }
}
*/





//1 执行自动跟脚程序
bool FlexMotorGetEnable(void)
{
  return(FlexMotorEnable);
}
void FlexMotorSetEnable(void)
{
  FlexMotorEnable = true;
  w_FlexAdjStep = 0; 
}
void FlexMotorSetDisable(void)
{
  FlexMotorEnable = false;
  w_FlexAdjStep = 0; 
}

void Flex_SetStatus(unsigned char status)
{
   nFlexStatus = status;
}
void Flex_ControlStop(void)
{
  Flex_SetDirection(FLEX_MOTOR_STOP);
}
unsigned char Flex_ControlIn(unsigned int current)
{//向里碰到角度开度开关 停止运行
  if((nFlexStatus&0x03) ==  FLEX_AT_IN || (nFlexStatus&BIT5))  //#define BIT5  0x20
  {
    Flex_SetDirection(FLEX_MOTOR_STOP);
    return 1;
  }
//    if((nFlexStatus&0x04) ==  0x04)
//  {
//    //Flex_SetDirection(FLEX_MOTOR_STOP);
//    return 4;
//  }  
  
  Flex_SetCurrent(current);
  Flex_SetDirection(FLEX_TO_IN);
  return 0;
}
unsigned char Flex_ControlOut(unsigned int current)
{
  if((nFlexStatus&0x03) ==  FLEX_AT_OUT)  
  {
    Flex_SetDirection(FLEX_MOTOR_STOP);
    return 1;
  }
  if(nLegAngle < FLEX_START_ANGLE)//小于角度开关没法向外伸
  {
    Flex_SetDirection(FLEX_MOTOR_STOP);
    return 1;
  } 
  Flex_SetCurrent(current);
  Flex_SetDirection(FLEX_TO_OUT);
  return 0;
}

void Flex_100msInt(void)
{
  if(nFlexStatus&0x04)
  w_Timer++;
}

unsigned char Flex_GetStatus(void)
{
  return(nFlexStatus);
}