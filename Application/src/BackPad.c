#include "EFM32_def.h"
#include "EFM32_types.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "input.h"
#include "memory.h"
#include "MassageStatus.h"
#include "valve.h"
#include  "ADC_Scan.h"
#include "BackPad.h"
#include "Flex_Uart.h"
//�����綯��λ�� ��ߣ�0 ��ͣ����λ��
#include "signalUart_new.h"


#define BACK_PROTECT_PORT     gpioPortD
#define BACK_PROTECT_BIT      10





//#ifndef BACK_PLUSE_COUNTER




UINT32 w_Position;
static bool bBackMotorFlag;

static unsigned int motorStatus = MOTOR_STOP_BREAK;
static volatile unsigned int motorBreakTime;

void BackMotor_Initial_IO(void)
{
  GPIO_PinModeSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT, BACK_MOTOR_RESET_MODE, 1);
  GPIO_PinModeSet(BACK_MOTOR_ENBL_PORT, BACK_MOTOR_ENBL_BIT, BACK_MOTOR_ENBL_MODE, 0);
  GPIO_PinModeSet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT, BACK_MOTOR_PHASE_MODE, 0);
  GPIO_PinModeSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT, BACK_MOTOR_DECAY_MODE, 0);
  GPIO_PinModeSet(BACK_MOTOR_FAULT_PORT, BACK_MOTOR_FAULT_BIT, BACK_MOTOR_FAULT_MODE, 1);
  /*
  GPIO_PinModeSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT, BACK_MOTOR_RESET_MODE, 1);
  GPIO_PinModeSet(BACK_MOTOR_ENBL_PORT, BACK_MOTOR_ENBL_BIT, BACK_MOTOR_ENBL_MODE, 1);
  GPIO_PinModeSet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT, BACK_MOTOR_PHASE_MODE, 1);
  GPIO_PinModeSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT, BACK_MOTOR_DECAY_MODE, 0);
  GPIO_PinModeSet(BACK_MOTOR_FAULT_PORT, BACK_MOTOR_FAULT_BIT, BACK_MOTOR_FAULT_MODE, 1);
  
  Power_On();
  while(1);
  */
  TIMER_InitCC_TypeDef timerCCInit = BACK_MOTOR_Timer_CCInit;
  TIMER_InitCC(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, &timerCCInit);
  BACK_MOTOR_TIMER->ROUTE |= (BACK_MOTOR_ROUTE_EN | BACK_MOTOR_ROUTE_LOCATION); 
  TIMER_TopSet(BACK_MOTOR_TIMER, BACK_MOTOR_DEFAULT_TOP);
  TIMER_CompareBufSet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL,0);//65);// 0);
  TIMER_Init_TypeDef timerInit = BACK_MOTOR_Timer_Init;
  TIMER_Init(BACK_MOTOR_TIMER, &timerInit);
  
}
/*
int BackMotor_GetPower(void)
{
  if(GPIO_PinOutGet(BACK_MOTOR_ENBL_PORT,BACK_MOTOR_ENBL_BIT))
  {
   return BACK_MOTOR_POWER_ON; 
  }
  return BACK_MOTOR_POWER_OFF; 
}
*/
int BackMotor_GetDirection(void)
{
  if(GPIO_PinOutGet(BACK_MOTOR_PHASE_PORT,BACK_MOTOR_PHASE_BIT))
  {
   return BACK_MOTOR_GO_UP; 
  }
  return BACK_MOTOR_GO_DOWN; 
}

void BackPower_Off(void)
{
  TIMER_CompareBufSet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, 0);
  //BackMotor_Set_Pwm_Data(0);
}

unsigned int BackMotor_VoltageAdj(void)
{
  unsigned short adc24;      //�˴��ĵ�ѹֵ�Ѿ�������100��
  unsigned int setDuty = BACK_MOTOR_DEFAULT_TOP;
  ADC_Get_Voltage(ADC_V24,&adc24);     
  if(adc24 <= BACK_SET_VOLTAGE/100) 
  {
    return setDuty;        //��ѹֵƫ�ͣ�����Ԥ��ֵ
  }
  unsigned int scale = BACK_SET_VOLTAGE / adc24; //�������趨��ѹ�ı���ֵ
  setDuty *= scale;
  unsigned int yushu = setDuty  % 100;
  setDuty /= 100;
  if(yushu > 50) setDuty++;
  return setDuty; 
}

void BackPower_On(void)
{
  unsigned long  ulDuty;
  ulDuty = BackMotor_VoltageAdj();
  if(BackMotor_Get_Fault() == BACK_MOTOR_FAIL) 
   {
      BackMotor_Reset();
     __no_operation();
     __no_operation();
     BackMotor_Reset_Cancel();
   }
  TIMER_CompareBufSet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, ulDuty);
}
/*
static void BackPower_On(void)
{
  BackMotor_Set_Pwm_Data(BACK_MOTOR_DEFAULT_TOP);
}
static void BackPower_Off(void)
{
  BackMotor_Set_Pwm_Data(0);
}
*/

void BackMotor_Up(void)
{
  GPIO_PinOutSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
//  GPIO_PinOutSet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT);
  GPIO_PinOutClear(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT);
  GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
}
void BackMotor_Down(void)
{
  GPIO_PinOutSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
 // GPIO_PinOutClear(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT);
   GPIO_PinOutSet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT);
  
  GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);//DECAY =1 ����ֹͣ//Ϊ�ߵ�ƽʱ��250V ��������ѹ���½���15V
}
void BackMotor_Break(void)
{
  BackPower_Off();    //�ر�����Դ
  if(motorStatus == MOTOR_RUN)
  {
  //tt GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT); //ʹ���˿ڴ��ڸ��裬��ʱ������ڹ��Ի���
   motorStatus = MOTOR_STOP_HZ;
   motorBreakTime = 0;
  }
  if(motorStatus == MOTOR_STOP_HZ)
  {
    if(motorBreakTime < MOTOR_STOP_HZ_TIME) return;
  }
 //tt GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
  //GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);  //��·�����������ɲ��״̬
  motorStatus = MOTOR_STOP_BREAK;
  
    //-------------------------------------------------------------------//������λ
   GPIO_PinOutClear(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
  //-------------------------------------------------------------------
  
  
  
}

void BackMotor_Reset(void)
{
  GPIO_PinOutClear(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
}
void BackMotor_Reset_Cancel(void)
{
  GPIO_PinOutSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
}
int BackMotor_Get_Fault(void)
{
  if(GPIO_PinInGet(BACK_MOTOR_FAULT_PORT, BACK_MOTOR_FAULT_BIT))
    return BACK_MOTOR_NORMAL;
  return BACK_MOTOR_FAIL;
}
//���ؿ����綯�׵Ĵ�λ�� ��ߣ���ͻ��м�
unsigned int BackMotor_Get_Location(void)
{
  if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT)   return BACK_MOTOR_AT_TOP;
  if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)   return BACK_MOTOR_AT_BOTTOM;
  return BACK_MOTOR_AT_MID;
}
//���ؿ����綯�׵ľ���λ�ã���ʱ������¼�������綯��λ����ߴ�ʱ��Ϊ0����λ10ms
unsigned int BackMotor_Get_Position(void)
{
  return w_Position;
}

int BackMotor_GetPower(void)
{
  if(TIMER_CompareBufGet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL))
  {
   return BACK_MOTOR_POWER_ON; 
  }
  return BACK_MOTOR_POWER_OFF; 
}

void BackMotor_10ms_int(void)
{
  bBackMotorFlag = TRUE;
}

void BackMotor_Proce(void)
{
  if(!bBackMotorFlag) return;
  
  if(REACH_BACK_LIMIT == Input_GetBackDownSwitch())
    {
      w_Position = BACK_MOTOR_MAX_POSITION; 
      return;
    }
  if(REACH_BACK_LIMIT == Input_GetBackUpSwitch())
    {
      w_Position = 0; 
      return;
    }
  
  bBackMotorFlag = FALSE;
  
  if(!TIMER_CompareBufGet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL)) return;
  
  if(!GPIO_PinOutGet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT))
  {
    if(w_Position > 0) w_Position--;  
  }
  else
  {
    if(w_Position < BACK_MOTOR_MAX_POSITION) w_Position++;  
  }
}
void currentBackPadMotorState_reset()
{
  //currentBackPadMotorState = STATE_BACK_IDLE;
}
//BackPad motor control function
unsigned char BackMotor_Control(unsigned char nFinalBackPadMotorState)
{
  static unsigned int position = 0;
  unsigned char nRetVal ;
  bool bPowerFlag;
  nRetVal = FALSE ;
  
  bool enable = ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS);
    if(enable) 
      if(Input_GetSlideBackwardSwitch() == REACH_SLIDE_LIMIT)
        if(Input_GetSlideForwardSwitch() == REACH_SLIDE_LIMIT)
    {
        BackMotor_Break();
        TIMER_CompareBufSet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, 0);
        return TRUE;
    }
  

    
    
  
  switch(nFinalBackPadMotorState)
  {
  case STATE_RUN_BACK_DOWN:  //back motor go down
      
    if((Flex_GetStatus()&BIT5))
    {
      //position = 1;
      bPowerFlag = FALSE;
      nRetVal = TRUE ;
      BackMotor_Break();
      break;
    }
      
    if(REACH_BACK_LIMIT == Input_GetBackDownSwitch() || position == 1)
    {
      position = 1;
      bPowerFlag = FALSE;
      nRetVal = TRUE ;
      BackMotor_Break();
      break;
    }
    position = 0;
    BackMotor_Down();
    bPowerFlag = TRUE;
    break ;
  case STATE_RUN_BACK_UP:  //back motor go up
      
    if(  (Flex_GetStatus()&BIT5) )/*��ѹ��*/
    {
      //position = 2;
      bPowerFlag = FALSE;
      nRetVal = TRUE ;
      BackMotor_Break();
      break;
    }
    if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT || position == 2)
    {
      position = 2;
      bPowerFlag = FALSE;
      nRetVal = TRUE ;
      BackMotor_Break();
      break;
    }
    position = 0;
    BackMotor_Up();
    bPowerFlag = TRUE;
    break ;
  case STATE_BACK_IDLE:
    nRetVal = TRUE ;
    BackMotor_Break();
    bPowerFlag = FALSE;
    break ;
  default://�쳣����
    break ;
  }
  //��Դ���ֵĴ���
  if(bPowerFlag == TRUE)
  {
    BackPower_On();
  }
  else
  {
    BackPower_Off();
  }
  return nRetVal ;
}

//========================================================================