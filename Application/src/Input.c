
/*
* Function: Input signal process
*/
/************************************************************************************/

#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "efm32_types.h"
#include "efm32_def.h"
#include "LegMotor.h"
#include "Valve.h"
#include "WalkMotor.h"
#include "AxisMotor.h"
#include "MassageStatus.h"
#include "memory.h"
#include "signalUart_new.h"
#include "backaction.h"
#include "BackPad.h"

#include "UartLeg.h"

#include "input.h"
#include "SingleLine.h"

BITS InputData1;
#define bLegDownSwitch 	        InputData1.bD0   
#define bLegUpSwitch		InputData1.bD1   
#define bSlidePadUpSwitch	InputData1.bD2  
#define bSlidePadDownSwitch	InputData1.bD3   
#define bWalkDownSwitch 	InputData1.bD4   
#define bWalkUpSwitch           InputData1.bD5   
#define bBackUpSwitch		InputData1.bD6
#define bBackDownSwitch		InputData1.bD7

#define Data1Offset         0
#define Data2Offset         1
#define Data3Offset         2



typedef  struct
{
        unsigned char timer_H;
        unsigned char timer_L;
        unsigned char flag;
}INPUT_ST;
//bool bVout;
bool bVshoulder=0;
bool vReadSholuder=0;
bool bWalkChange,bInputReady;
bool _3DfrontSwitch, _3DBackSwitch;
  
INPUT_ST st_FlexGround,st_StretchOut,st_StretchIn,st_Foot,st_Angle,st_Vout,st_AxisSW;
INPUT_ST st_SlideUp,st_SlideDown,st_BackUp,st_BackDown,st_LegUp,st_LegDown,st_WalkUp,st_WalkDown;
bool b5msFlag;
BYTE nPulseHigh;
UINT16 WalkCount = 0xffff;  // 0xffff indicate no initial, at top =0 at bottom = maximal, up:dec down:add
BYTE by_KneadPosition = KNEAD_WIDTH_UNKNOWN;
volatile unsigned short nCurWalkLocate;
//unsigned short nCounterCurWalkLocate;
unsigned short nCurAxisLocate;
unsigned short nCounterCurAxisLocate;
unsigned int tickAxisCount;
bool bKneadMin,bKneadMid,bKneadMax,bBalance;
bool bWalkPulseCount,bPreWalkPulseCount;
//unsigned int axisTickTimedec[100];
//unsigned int axisTickTimeadd[100];


volatile unsigned int nCurBackLocate;

bool bShould,bShouldDelay;

unsigned int tet,tet2;
unsigned int cont;

unsigned int phawe;

//get backmotor's pulse
#define BACK_PULSE_CHECK_TIME 3
unsigned char BackPulseCountHigh = 0;
unsigned char BackPulseCountLow = 0;
unsigned char BackPulseStep = 0;

unsigned short test_count=0;
void Input_Back_Pulse1MS(void)
{
  bool flag = GPIO_PinInGet(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT);
  if(flag == 0){
    //低电平为开关有效
    BackPulseCountLow++;
  }else{
    BackPulseCountHigh++;
  }
  //从高到底再到高为1个有效数
  switch(BackPulseStep){
  case 0:
    if((flag == 1) && (BackPulseCountHigh > BACK_PULSE_CHECK_TIME)){
      BackPulseCountLow = 0;
      BackPulseStep++;
    }
    break;
  case 1:
    //从高到底再到高为1个有效数
    if((flag == 0) && (BackPulseCountLow > BACK_PULSE_CHECK_TIME)){
      BackPulseCountHigh = 0;
      BackPulseStep = 0;
      //计数
      if(!GPIO_PinOutGet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT) != 0)
      {
        if(nCurBackLocate > 0 )nCurBackLocate-- ;
		//test_count--;
		
      }
	  else
	  {
        nCurBackLocate++ ;
		//test_count++;
		
      }
    }
    break;
  }
}







void GPIO_ODD_IRQHandler(void)
{
  
}

void GPIO_EVEN_IRQHandler(void)
{ 
 /* SingleLine_Interruput();
  if(GPIO_IntGet()&(1<<INPUT_WALK_PULSE_BIT))//读取3D机芯的行走电机脉冲信号
  {
    bWalkChange = TRUE;
    GPIO_IntClear(1<<INPUT_WALK_PULSE_BIT);
    if(WalkMotor_GetDirection() != WALK_MOTOR_GO_UP)
    {
      if(nCurWalkLocate > 0 )
      {
          nCurWalkLocate-- ;
      }
    }
    else
    {
      if(nCurWalkLocate < 1000)
      {
          nCurWalkLocate++ ;
      }
    }
  } */
}

void Input_SetWalkMotorPosition(unsigned short locate)
{
//#ifdef RT8600S
      nCurWalkLocate = (unsigned short)locate<<2;
//#else
  
 //     nCurWalkLocate = (unsigned short)locate*2;
//#endif
  
}
unsigned short Input_GetWalkMotorPosition(void)//
{

       return(nCurWalkLocate>>2);//4分频

}

//相反方向
void Input_SetCounterWalkMotorPosition(unsigned short locate) //8600
{
  return;
  //nCounterCurWalkLocate = (unsigned short)locate;
  //nCounterCurWalkLocate *= 2;
}
/*
unsigned short Input_GetCounterWalkMotorPosition(void) //8600
{
  return(nCounterCurWalkLocate/2);
}
*/
void Input_SetAxisMotorPosition(unsigned short locate)//3D机芯镜向坐标
{
  nCurAxisLocate = (unsigned short)locate;
}
unsigned short Input_GetAxisMotorPosition(void)
{
  return(nCurAxisLocate);
}

//相反方向
void Input_SetCounterAxisMotorPosition(unsigned short locate) //8600
{
  nCounterCurAxisLocate = (unsigned short)locate;
  nCounterCurAxisLocate *= 2;
}
unsigned short Input_GetCounterAxisMotorPosition(void) //8600
{
  return(nCounterCurAxisLocate/2);
}

void Input_Initial_IO(void)
{
  /*
#ifdef RT8600S
  GPIO_PinModeSet(INPUT_WALK_PULSE_PORT, INPUT_WALK_PULSE_BIT, INPUT_WALK_PULSE_MODE, 1);
                     // GPIO_PinModeSet(INPUT_KNEAD_MAX_PORT, INPUT_KNEAD_MAX_BIT, INPUT_KNEAD_MAX_MODE, 1);
                     // GPIO_PinModeSet(INPUT_KNEAD_MID_PORT, INPUT_KNEAD_MID_BIT, INPUT_KNEAD_MID_MODE, 1);
                     // GPIO_PinModeSet(INPUT_KNEAD_MIN_PORT, INPUT_KNEAD_MIN_BIT, INPUT_KNEAD_MIN_MODE, 1);
                      
                    
                        
                      //GPIO_IntConfig(INPUT_KNEAD_MAX_PORT, INPUT_KNEAD_MAX_BIT, false, true, true);
                      //GPIO_IntConfig(INPUT_KNEAD_MID_PORT, INPUT_KNEAD_MID_BIT, false, true, true);
                      //GPIO_IntConfig(INPUT_KNEAD_MIN_PORT, INPUT_KNEAD_MIN_BIT, false, true, true);
                      
                      
                      //GPIO_PinModeSet(INPUT_LEG_FOOT_PORT, INPUT_LEG_FOOT_BIT, INPUT_LEG_FOOT_MODE, 1);
  
#else  */
  GPIO_PinModeSet(INPUT_WALK_PULSE_PORT, INPUT_WALK_PULSE_BIT, INPUT_WALK_PULSE_MODE, 1);
 // GPIO_IntConfig(INPUT_WALK_PULSE_PORT, INPUT_WALK_PULSE_BIT, false, true, true);
  
  
  GPIO_PinModeSet(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT, INPUT_BACK_PULSE_MODE, 1);
//  GPIO_IntConfig(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT, false, true, true);
  
  
  
  
 // NVIC_EnableIRQ(GPIO_ODD_IRQn);
//  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
//#endif
  
              //        GPIO_PinModeSet(INPUT_KNEAD_MAX_PORT, INPUT_KNEAD_MAX_BIT, INPUT_KNEAD_MAX_MODE, 1);
              //        GPIO_PinModeSet(INPUT_KNEAD_MID_PORT, INPUT_KNEAD_MID_BIT, INPUT_KNEAD_MID_MODE, 1);
              //        GPIO_PinModeSet(INPUT_KNEAD_MIN_PORT, INPUT_KNEAD_MIN_BIT, INPUT_KNEAD_MIN_MODE, 1);
 //  GPIO_PinModeSet(INPUT_SHOULDER_PULSE_PORT, INPUT_SHOULDER_PULSE_BIT, INPUT_SHOULDER_PULSE_MODE, 1);
  
                    //  GPIO_PinModeSet(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT, INPUT_BACK_PULSE_MODE, 1);
                      //GPIO_IntConfig(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT, false, true, true);
                    //  GPIO_PinModeSet(INPUT_AXIS_PULSE_PORT, INPUT_AXIS_PULSE_BIT, INPUT_AXIS_PULSE_MODE, 1);
                     // GPIO_IntConfig(INPUT_AXIS_PULSE_PORT, INPUT_AXIS_PULSE_BIT, false, true, true);
                     // GPIO_PinModeSet(INPUT_AXIS_SW_PORT, INPUT_AXIS_SW_BIT, INPUT_AXIS_SW_MODE, 1);
                    //  GPIO_PinModeSet(INPUT_VOUT_PORT, INPUT_VOUT_BIT, INPUT_VOUT_MODE, 1);
                      
                    //  GPIO_PinModeSet(INPUT_BACK_UP_PORT, INPUT_BACK_UP_BIT, INPUT_BACK_UP_MODE, 1); 
                    //  GPIO_PinModeSet(INPUT_BACK_DOWN_PORT, INPUT_BACK_DOWN_BIT, INPUT_BACK_DOWN_MODE, 1); 
                      
                    //  GPIO_PinModeSet(INPUT_LEG_UP_PORT, INPUT_LEG_UP_BIT, INPUT_LEG_UP_MODE, 1); 
                    //  GPIO_PinModeSet(INPUT_LEG_DOWN_PORT, INPUT_LEG_DOWN_BIT, INPUT_LEG_DOWN_MODE, 1); 
                      
                    //  GPIO_PinModeSet(INPUT_WALK_UP_PORT, INPUT_WALK_UP_BIT, INPUT_WALK_UP_MODE, 1); 
                    //  GPIO_PinModeSet(INPUT_WALK_DOWN_PORT, INPUT_WALK_DOWN_BIT, INPUT_WALK_DOWN_MODE, 1); 
                      
                    //  GPIO_PinModeSet(INPUT_ZERO_UP_PORT, INPUT_ZERO_UP_BIT, INPUT_ZERO_UP_MODE, 1); 
                    //  GPIO_PinModeSet(INPUT_ZERO_DOWN_PORT, INPUT_ZERO_DOWN_BIT, INPUT_ZERO_DOWN_MODE, 1); 
                      //  GPIO_PinModeSet(INPUT_WAVE_PORT, INPUT_WAVE_BIT, INPUT_WAVE_MODE, 1); 
                      
                      //  NVIC_EnableIRQ(GPIO_ODD_IRQn);
   // NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}
void Input_5ms_Int(void)
{
    tickAxisCount++;
    if(AxisMotor_IsRun())
    {
      tickAxisCount++;
    }
    b5msFlag = 1;
}

void Input_High(INPUT_ST* p)
{
  p->timer_L = 0;
  p->timer_H ++; 
  if((p->timer_H) >= 2)
  {
    p->timer_H = 2;
    p->flag = 1;
  }
}
void Input_Low(INPUT_ST* p)
{
  p->timer_H = 0;
  p->timer_L ++; 
  if((p->timer_L) >= 2)
  {
    p->timer_L = 2;
    p->flag = 0;
  }
}
/*

BITS InputData1;
#define bLegDownSwitch 	        InputData1.bD0   
#define bLegUpSwitch		InputData1.bD1   
#define bSlidePadUpSwitch	InputData1.bD2  
#define bSlidePadDownSwitch	InputData1.bD3   
#define bWalkDownSwitch 	InputData1.bD4   
#define bWalkUpSwitch           InputData1.bD5   
#define bBackUpSwitch		InputData1.bD6
#define bBackDownSwitch		InputData1.bD7
*/

unsigned short nCurWalkLocateadd;
unsigned short nCurWalkLocatedec;
void Input_Proce(void)
{
  unsigned char signal,switch_signal;
  static int counter;  
  static unsigned short nOldCurAxisLocate = 0;
  if(!b5msFlag) return;		
  b5msFlag = 0;
  if(++counter >= 4) bInputReady = true;
  
  if(UartLeg_Get3DMessage_RXStatus())
  {
    UartLeg_Clr_3DMessageStatus();
    UartLeg_3DMessageCopyData ();
    
    
  }
  

  nCurAxisLocate = UartLeg_Get3DPluse();//3D机芯镜向位置坐标
  if(nOldCurAxisLocate |= nCurAxisLocate);
  {
    AxisMotor_UpdataPosition();
    nOldCurAxisLocate = nCurAxisLocate;
  }
  
    signal = UartLeg_Get3DMassageSignal();//读取3D行程开关信号
  //		   7        6	     5              4	           3            2   1   0//敲击平位置=3,       4=肩膀位置
  //xxxx xxxx  --> 0 3DbackSwitch 3DfrontSwitch shoulderPosition MotorPosition Max Med Min
  
  if(signal&BIT2)
  {
    by_KneadPosition = KNEAD_WIDTH_MIN;    
    bKneadMin = 0;
    bKneadMid = 1; 
    bKneadMax = 1;
  }
  if(signal&BIT1)
  {
    by_KneadPosition = KNEAD_WIDTH_MED;    
    bKneadMin = 1;
    bKneadMid = 0; 
    bKneadMax = 1;
  }
  if(signal&BIT0)
  {
    by_KneadPosition = KNEAD_WIDTH_MAX;  
    bKneadMin = 1;
    bKneadMid = 1; 
    bKneadMax = 0;
  }
  
  KneadMotor_CalculateSpeed(by_KneadPosition);
  
  if(signal&BIT4)
  {
   // bVout = 1;
        bShould = 1;
        bShouldDelay =1;//nShouldDelay = 0;
  }
  else
  {
   // bVout = 0;
            bShould = 0;
            bShouldDelay =0;
    
  }
  
  if(signal&BIT6)
  {
    _3DfrontSwitch = 1;
  }
  else
  {
    _3DfrontSwitch = 0; 
  }
  if(signal&BIT5)
  {
    _3DBackSwitch = 1;
  }
  else
  {
    _3DBackSwitch = 0;
  }

  

  signal = SignalBoard_GetMassageSignal();
  //		   7        6	     5              4	           3            2   1   0
  //xxxx xxxx  --> 0 3DbackSwitch 3DfrontSwitch shoulderPosition MotorPosition Max Med Min
  
  //球不压为ON,默认为上拉   走到肩部位置以上释放揉捏球，球为OFF
//  if(signal&BIT0)
//  {
//   // bVout = 1;
//        bShould = 1;
//        bShouldDelay =1;//nShouldDelay = 0;
//        bVshoulder=0;
//  }
//  else//球按下为OFF,  
//  {
//   // bVout = 0;
//            bShould = 0;
//            bShouldDelay =0;
//             bVshoulder=1;
//  }
//    if(signal&BIT1)
//  {
//    by_KneadPosition = KNEAD_WIDTH_MED;    
//    bKneadMin = 1;
//    bKneadMid = 0; 
//    bKneadMax = 1;
//  }
//    if(signal&BIT2)
//  {
//    by_KneadPosition = KNEAD_WIDTH_MAX;  
//    bKneadMin = 1;
//    bKneadMid = 1; 
//    bKneadMax = 0;
//  }
//  if(signal&BIT3)
//  {
//    by_KneadPosition = KNEAD_WIDTH_MIN;    
//    bKneadMin = 0;
//    bKneadMid = 1; 
//    bKneadMax = 1;
//  }

   if(signal&BIT4)//ok
   {
     bLegUpSwitch=1;
     
   }
   else
   {
     bLegUpSwitch=0;
     
   }

  if(signal&BIT5)  
   {
     bLegDownSwitch=1;
     
   }
   else
   {
     bLegDownSwitch=0;
     
   }  
  
  
  //-----------------------------------------------------------
 if(signal&BIT6)//靠背限位被反向，原因不明

   {
    
     bBackUpSwitch =1;

   }
   else
   {
     bBackUpSwitch=0;

   }  
   
  if(signal&BIT7)

   {
     bBackDownSwitch=1;

   }
   else
   {
       bBackDownSwitch=0;

   }  
   //------------------------------------------------
     switch_signal = SignalBoard_GetSwitchSignal();

   if(switch_signal&BIT0)//ok
   {
     bWalkDownSwitch=1;
     
   }
   else
   {
     bWalkDownSwitch=0;
     
   }  
   if(switch_signal&BIT1)
   {
     bWalkUpSwitch=1;
     
   }
   else
   {
     bWalkUpSwitch=0;
     
   }  
   
   //--------------------------
 
 if(switch_signal&BIT7)
   {
     bSlidePadUpSwitch=1;
     
   }
   else
   {
     bSlidePadUpSwitch=0;
     
   }  
   

  if(switch_signal&BIT6)
   {
     bSlidePadDownSwitch=1;
     
   }
   else
   {
     bSlidePadDownSwitch=0;
     
   }    
   /*
     bSlidePadDownSwitch=0;
  bSlidePadUpSwitch=0;
       bWalkUpSwitch=0;
    bWalkDownSwitch=0;
         bBackDownSwitch=0;
       bBackUpSwitch=0;
          bLegDownSwitch=0;
               bLegUpSwitch=0;

          */
          
  //bLegStretchGroundSwitch?Input_High(&st_FlexGround):Input_Low(&st_FlexGround);
  bLegUpSwitch ? Input_High(&st_LegUp):Input_Low(&st_LegUp);
  bLegDownSwitch ? Input_High(&st_LegDown):Input_Low(&st_LegDown);
  
  bBackUpSwitch ? Input_High(&st_BackUp):Input_Low(&st_BackUp);
  
  bBackDownSwitch ? Input_High(&st_BackDown):Input_Low(&st_BackDown);			
  bSlidePadUpSwitch ? Input_High(&st_SlideUp):Input_Low(&st_SlideUp);
  bSlidePadDownSwitch ? Input_High(&st_SlideDown):Input_Low(&st_SlideDown);
  bWalkUpSwitch ? Input_High(&st_WalkUp):Input_Low(&st_WalkUp);
  bWalkDownSwitch ? Input_High(&st_WalkDown):Input_Low(&st_WalkDown);
  
//#ifndef RT8600S
  bWalkPulseCount = GPIO_PinInGet(INPUT_WALK_PULSE_PORT, INPUT_WALK_PULSE_BIT);
  if(bPreWalkPulseCount != bWalkPulseCount)
  { 
    if(WalkMotor_GetDirection() != WALK_MOTOR_GO_UP)//马达向下
    {
      if(nCurWalkLocate > 0 )
      {
        nCurWalkLocate-- ;
      }
      if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
      {
        Input_SetWalkMotorPosition(0);
      }
     nCurWalkLocateadd=0;
     nCurWalkLocatedec++;      
    }
    else
    {
      if(nCurWalkLocate < 2000)
      {
        nCurWalkLocate++ ;
      }
      if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
      {
        Input_SetWalkMotorPosition(TOP_POSITION);//1
      }
           nCurWalkLocateadd++;
     nCurWalkLocatedec=0; 
    }
  }
  bPreWalkPulseCount = bWalkPulseCount;
//#endif
  
  
}
unsigned int Input_GetVout(void)
{
    if(bShouldDelay==0) return BODY_TOUCHED; 
    return BODY_NO_TOUCHED;
}   



//return 0 or 1
unsigned int Input_GetKneadMax(void)
{
  return(bKneadMax);
}
//return 0 or 1
unsigned int Input_GetKneadMid(void)
{
    return(bKneadMid);
}
//return 0 or 1
unsigned int Input_GetKneadMin(void)
{
    return(bKneadMin);
}

/********用于工程模式下显示款中窄值*********/
uint8_t Input_Get_Mim_Max()
{
 if(bKneadMin==0) return 1;
 else if(bKneadMid==0) return 2;
 else  if(bKneadMax==0) return 3;
}

unsigned char Input_GetBalance(void)
{
  return(bBalance);
}
unsigned int Input_GetBackUpSwitch(void)
{
  return(st_BackUp.flag);
}                     
unsigned int Input_GetBackDownSwitch(void)
{
  return(st_BackDown.flag);
}                     
unsigned int Input_GetLegUpSwitch(void)
{
  return(st_LegUp.flag);
}                     
unsigned int Input_GetLegDownSwitch(void)
{
  return(st_LegDown.flag);
}

unsigned int Input_GetSlideForwardSwitch(void)
{
  bool enable = ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS);
  if(enable)
   //  return(st_SlideDown.flag);
     return(st_SlideUp.flag);
  else
    return(1);
}
unsigned int Input_GetSlideBackwardSwitch(void)
{
  bool enable = ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS);
  if(enable)
 //  return(st_SlideUp.flag);
    return(st_SlideDown.flag);
  else return(1);
}

unsigned int Input_GetKneadPosition(void)
{
  return (unsigned int)by_KneadPosition;
}

/*
unsigned int Input_GetAxisSW(void)
{
  return(st_AxisSW.flag);
}
*/
//unsigned int Input_GetFlexGroundSwitch(void)
//{
 // return(st_FlexGround.flag);
//}

unsigned int Input_GetMp3Status(void)
{
  return 1;
}
unsigned int Input_PowerCheck(void)
{
  return 1;
}

/*
bool Input_GetWalkChange(void)
{
  return(bWalkChange);
}

void Input_ClearWalkChange(void)
{
  bWalkChange = 0;
}
*/
unsigned int Input_GetWalkPosition(void)
{
  if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
    return WALK_MOTOR_AT_BOTTOM;
  if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
    return WALK_MOTOR_AT_TOP;
  else
    return WALK_MOTOR_AT_MID;
}
/*
//最前面位置
unsigned int Input_GetFlexOutSwitch(void)
{
  return(st_StretchOut.flag);
} 
//最后面位置
unsigned int Input_GetFlexInSwitch(void)
{
  return(st_StretchIn.flag);
}                     

unsigned int Input_GetFlexFootSwitch(void)
{
  return(st_Foot.flag);
} 

unsigned int Input_GetFlexAngleSwitch(void)
{
  return(st_Angle.flag);
} 
*/
unsigned int Input_GetWalkUpSwitch(void)
{
  return(st_WalkUp.flag);
}
unsigned int Input_GetWalkDownSwitch(void)
{
  return(st_WalkDown.flag);
}
unsigned int Input_GetReady(void)
{
    if(bInputReady) return 1;
    return 0;
}
bool Input_Get3DFrontSwitch(void)
{
  return(_3DfrontSwitch);
} 

bool Input_Get3DBackSwitch(void)
{
  return(_3DBackSwitch);
} 


//获取靠背电动缸的脉冲计数位置







