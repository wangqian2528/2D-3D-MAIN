#include "efm32_types.h"
#include "efm32_def.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "LegKnead_Uart.h"
#include "Roller_Uart.h"
#include "Flex_Uart.h"
#include "Valve.h"
#include "Input.h"
#include "backaction.h"
extern bool  bLegKneadEnableonly;
extern bool  bAUTO1AirEnable;
#define VALVE_DATA_LENGTH   3//3// 2

unsigned char ucSendData[VALVE_DATA_LENGTH];
unsigned char ucReceiveData[VALVE_DATA_LENGTH];
//unsigned char* pValveData = ucSendData;
unsigned char* pInputData = ucReceiveData;
unsigned int w_RollerCounter,bSholderEnable;
unsigned int SholderTime = 0;
static bool bAutoRoller;
//手动模式PWM设置
__no_init unsigned int w_RollerPWM;
//手动模式 速度和方式
unsigned char LegKneadSpeed;
unsigned char LegKneadMode;
static bool bValveFlag,bRollerFlag,bLegKneadFlag;
extern StretchStruct st_Stretch;

extern  StretchStruct st_GrowthStretch;

extern unsigned int  GrowthStepMaxA;//  
extern unsigned int  GrowthStepMaxB;//
extern __no_init unsigned char nBackMainRunMode,nBackSubRunMode ;


__no_init static unsigned char nKeyAirBagStrength;
//74595串并转换(4片74595)
BITS BITS_ValveData[3];
static bool bKneadLegEnable;
unsigned int w_KneadLegCount;
bool bLegAirBagOn;

bool bBackauto;
unsigned int tt_step;
void Valve_Initial_IO(void)
{
    USART_TypeDef *spi = VALVE_SPI;
    USART_InitSync_TypeDef InitSync_Init = VALVE_USART_INITSYNC;
    
    /* Clearing old transfers/receptions, and disabling interrupts */
    //spi->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
    //spi->IEN = 0;
    
    USART_InitSync(spi,&InitSync_Init);
    
    USART0->ROUTE = (USART0->ROUTE & ~_USART_ROUTE_LOCATION_MASK) | USART_ROUTE_LOCATION_LOC1;
    /* Enabling pins and setting location */
    spi->ROUTE |= USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_CLKPEN;
    
    /* Clear previous interrupts */
    //spi->IFC = _USART_IFC_MASK;
    
    
    
    /*
    USART_TypeDef *spi = VALVE_SPI;
    USART_InitSync_TypeDef InitSync_Init = VALVE_USART_INITSYNC;
    
    spi->CMD   = USART_CMD_MASTEREN | USART_CMD_TXEN ;
    spi->CTRL |= USART_CTRL_AUTOCS;
    
    spi->IEN = 0;
    
    spi->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_CLKPEN | VALVE_SPI_ROUTE_LOCAITON;
    
    spi->IFC = _USART_IFC_MASK;
    
    USART_InitSync(spi,&InitSync_Init);
    
    IO configuration */
    // GPIO_PinModeSet(VALVE_POWER_PORT,VALVE_POWER_BIT,VALVE_POWER_MODE,0);
 //   GPIO_PinModeSet(VALVE_LOAD_PORT,VALVE_LOAD_BIT,VALVE_LOAD_MODE,0);
    GPIO_PinModeSet(VALVE_CLK_PORT,VALVE_CLK_BIT,VALVE_CLK_MODE,0);
    GPIO_PinModeSet(VALVE_LATCH_PORT,VALVE_LATCH_BIT,VALVE_LATCH_MODE,1);
    GPIO_PinModeSet(VALVE_DATA_PORT,VALVE_DATA_BIT,VALVE_DATA_MODE,1);
   // GPIO_PinModeSet(VALVE_DATA_IN_PORT,VALVE_DATA_IN_BIT,VALVE_DATA_IN_MODE,1);
    GPIO_PinModeSet(VALVE_AIRPUMP1_PORT,VALVE_AIRPUMP1_BIT,VALVE_AIRPUMP1_MODE,0);
    GPIO_PinModeSet(VALVE_AIRPUMP2_PORT,VALVE_AIRPUMP2_BIT,VALVE_AIRPUMP2_MODE,0);
   // GPIO_PinModeSet(VALVE_AIRPUMP3_PORT,VALVE_AIRPUMP3_BIT,VALVE_AIRPUMP3_MODE,0);
   // GPIO_PinModeSet(VALVE_OZON_PORT,VALVE_OZON_BIT,VALVE_OZON_MODE,0);
    
//    Power_All_On();
    BITS_ValveData[0].nByte = 0;//0;
    BITS_ValveData[1].nByte =0;//0X0;
    BITS_ValveData[2].nByte =0;// 0;
}

void Valve_Initial_Data(void)
{
    w_RollerPWM = ROLLER_SPEED_STOP;
    LegKneadSpeed = LEG_KNEAD_SPEED_STOP;
    nKeyAirBagStrength = 0;
}


void Valve_SetData(void)
{
    GPIO_PinOutSet(VALVE_DATA_PORT,VALVE_DATA_BIT);
}
void Valve_ClearData(void)
{
    GPIO_PinOutClear(VALVE_DATA_PORT,VALVE_DATA_BIT);
}

void Valve_SetClock(void)
{
    GPIO_PinOutSet(VALVE_CLK_PORT,VALVE_CLK_BIT);
}
void Valve_ClearClock(void)
{
    GPIO_PinOutClear(VALVE_CLK_PORT,VALVE_CLK_BIT);
}

void Valve_ClearLatch(void)
{
    GPIO_PinOutClear(VALVE_LATCH_PORT,VALVE_LATCH_BIT);
}
void Valve_SetLatch(void)
{
    GPIO_PinOutSet(VALVE_LATCH_PORT,VALVE_LATCH_BIT);
}

void Valve_10ms_Int(void)
{
    bValveFlag = true;
    bRollerFlag = true;
    bLegKneadFlag = true;
    w_RollerCounter++;
    SholderTime++;
    
    if(bLegAirBagOn)
    {
      if(w_KneadLegCount >= 600)
      {
        bKneadLegEnable = true;
      }
      if(w_KneadLegCount < 600)
      {
        w_KneadLegCount++;  
      }
    }
    else
    {
      if(w_KneadLegCount < 200)
      {
        bKneadLegEnable = false;
      }
      if(w_KneadLegCount > 0) w_KneadLegCount--; 
    }  
}
void Valve_1ms_Int(void)
{
    //bValveFlag = true;
}

static unsigned char SPI_FlashWrite(unsigned char data)
{
    VALVE_SPI->TXDATA = data;
    while (!(VALVE_SPI->STATUS & USART_STATUS_TXC))
    {
    }
    return (uint8_t)(VALVE_SPI->RXDATA);
}

/*本程序10ms执行一次，放在主程序中，数据长度固定为4*/
void Valve_Send_Data(void)
{
    unsigned int i;
    unsigned char ucLength = VALVE_DATA_LENGTH;
    
    if(!bValveFlag) return;
    bValveFlag = false;
    
    //BITS_ValveData[2].nByte=0X80;
    //BITS_ValveData[1].nByte=0X00;//0X08 0X10  nc;
    //BITS_ValveData[0].nByte= 0X00;
    
    for(int i = 0; i < ucLength; i++)
    {
        *(ucSendData + i) = BITS_ValveData[i].nByte;
    }
    
  //  GPIO_PinOutSet(VALVE_LOAD_PORT,VALVE_LOAD_BIT);
    
    
    for(i = 0;i < ucLength;i++)
    {
        *(ucReceiveData + i) = SPI_FlashWrite(*(ucSendData + i));
    }
    
    for(i = 100;i > 0;i--) __no_operation();
    
    GPIO_PinOutSet(VALVE_LATCH_PORT,VALVE_LATCH_BIT);
    
    for( i = 100;i > 0;i--) __no_operation();
    
    GPIO_PinOutClear(VALVE_LATCH_PORT,VALVE_LATCH_BIT);
    
  //  GPIO_PinOutClear(VALVE_LOAD_PORT,VALVE_LOAD_BIT);
}
 //    unsigned char  untesta ;
   //   unsigned char  untestb;
      
    //       unsigned char  untestc ;
  //    unsigned char  untestd;
void LegFootAirBagAction(bool Enable,unsigned int action)//
{    
    if((Flex_GetDirection() == FLEX_TO_OUT) || (Flex_GetDirection() == FLEX_TO_IN))
    {

      
      if(bAUTO1AirEnable == FALSE)
      {
        Enable = FALSE;
      
      }
    }
    if(!Enable)
    {
        bFootHeelAirBagValve = VALVE_OFF;
        bRightFootAirBagValve = VALVE_OFF;
        bLeftFootAirBagValve = VALVE_OFF;
        bLegLeftAirBagValve = VALVE_OFF;		        
        bLegRightAirBagValve = VALVE_OFF;  	        
        bLegAirBagOn = false;
        return;
    }
    if(action & LEG_LEFT)
    {
        bLegRightAirBagValve = VALVE_ON ;
    }
    else
    {
        bLegRightAirBagValve = VALVE_OFF ;
    }
    
    if(action & LEG_RIGHT)
    {
        bLegLeftAirBagValve = VALVE_ON ;
    }
    else
    {
        bLegLeftAirBagValve = VALVE_OFF ;
    }
    if((action & LEG_LEFT)&&(action & LEG_RIGHT))
    {
      bLegAirBagOn = true;
    }
    else
    {
      bLegAirBagOn = false;
    }
    
    if(action & F_L_SIDE)
    {
        bLeftFootAirBagValve = VALVE_ON ;
    }
    else
    {
        bLeftFootAirBagValve = VALVE_OFF ;
    }
    if(action & F_HEEL)
    {
        bFootHeelAirBagValve = VALVE_ON ;
    }
    else
    {
        bFootHeelAirBagValve = VALVE_OFF ;
    }
    if(action & F_R_SIDE)
    {
        bRightFootAirBagValve = VALVE_ON ;
    }
    else
    {
        bRightFootAirBagValve = VALVE_OFF ;
    }
}


void BackWaistAirBagAction(bool Enable,unsigned int action)
{  
    if(!Enable)
    {
        bBackWaistRightUp	  =  VALVE_OFF ;
        bBackWaistRightDown   =  VALVE_OFF ;
        bBackWaistLeftUp      =  VALVE_OFF ;  
        bBackWaistLeftDown    =  VALVE_OFF ;
        return;
    }
    /*
    if(action & PE1)
    {
        Valve_BodyUpAirPumpACPowerOn();
    }
    else
    {
        Valve_BodyUpAirPumpACPowerOff();
    }
    */
    if(action & R_U_WAIST)
    {
        bBackWaistRightUp = VALVE_ON ;
    }
    else
    {
        bBackWaistRightUp = VALVE_OFF ;
    }
    if(action & R_D_WAIST)
    {
        bBackWaistRightDown = VALVE_ON ;
    }
    else
    {
        bBackWaistRightDown = VALVE_OFF ;
    }
    if(action & L_U_WAIST)
    {
        bBackWaistLeftUp = VALVE_ON ;
    }
    else
    {
        bBackWaistLeftUp = VALVE_OFF ;
    }
    if(action & L_D_WAIST)
    {
        bBackWaistLeftDown = VALVE_ON ;
    }
    else
    {
        bBackWaistLeftDown = VALVE_OFF ;
    }
    
    bLeftThighDownAir            =  VALVE_OFF ;// 
    bRightThighDownAir           =  VALVE_OFF ;
    
    
}
void SeatAirBagAction(bool Enable,unsigned int action)
{
    if(!Enable)
    {
        bLeftThighAirBagValve   =  VALVE_OFF ;
        bRightThighAirBagValve 	=  VALVE_OFF ;
        return;
    }
    /*
    if(action & PE2)
    {
        Valve_LegFootAirPumpACPowerOn();
    }
    else
    {
        Valve_LegFootAirPumpACPowerOff();
    }
    */
    if(action & R_THIGH)
    {
        bRightThighAirBagValve = VALVE_ON ;
    }
    else
    {
        bRightThighAirBagValve = VALVE_OFF ;
    }
    if(action & L_THIGH)
    {
        bLeftThighAirBagValve = VALVE_ON ;
    }
    else
    {
        bLeftThighAirBagValve = VALVE_OFF ;
    }
}

void ArmSholderAirBagAction(bool Enable,unsigned int action)
{    
    if(!Enable)
    {
        bRightArmUpAirBagValve1   =  VALVE_OFF ;
        bRightArmUpAirBagValve2   =  VALVE_OFF ;
        bRightArmUpAirBagValve3   =  VALVE_OFF ;
        bLeftArmUpAirBagValve1    =  VALVE_OFF ;
        bLeftArmUpAirBagValve2    =  VALVE_OFF ;
        bLeftArmUpAirBagValve3    =  VALVE_OFF ;
        bLeftSholderAirBagValve   =  VALVE_OFF ;
        bRightSholderAirBagValve  =  VALVE_OFF ;
        return;
    }
    
    unsigned short nCurWalkMotorLocate = Input_GetWalkMotorPosition();
    /*
    if(action & PE1)
    {
        Valve_BodyUpAirPumpACPowerOn();
    }
    else
    {
        Valve_BodyUpAirPumpACPowerOff();
    }
    */
    if(action & R_ARM_1)
    {
        bRightArmUpAirBagValve1 = VALVE_ON ;
    }
    else
    {
        bRightArmUpAirBagValve1 = VALVE_OFF ;
    }
    if(action & R_ARM_2)
    {
        bRightArmUpAirBagValve2 = VALVE_ON ;
    }
    else
    {
        bRightArmUpAirBagValve2 = VALVE_OFF ;
    }
    if(action & R_ARM_3)
    {
        bRightArmUpAirBagValve3 = VALVE_ON ;
    }
    else
    {
        bRightArmUpAirBagValve3 = VALVE_OFF ;
    }
    if(action & L_ARM_1)
    {
        bLeftArmUpAirBagValve1 = VALVE_ON ;
    }
    else
    {
        bLeftArmUpAirBagValve1 = VALVE_OFF ;
    }
    if(action & L_ARM_2)
    {
        bLeftArmUpAirBagValve2 = VALVE_ON ;
    }
    else
    {
        bLeftArmUpAirBagValve2 = VALVE_OFF ;
    }
    if(action & L_ARM_3)
    {
        bLeftArmUpAirBagValve3 = VALVE_ON ;
    }
    else
    {
        bLeftArmUpAirBagValve3 = VALVE_OFF ;
    }
    if(nCurWalkMotorLocate > 230 && bBackauto)  //机芯超过这个高度不允许动作
    {
        bLeftSholderAirBagValve = VALVE_OFF ;
        bRightSholderAirBagValve = VALVE_OFF ;
        return;
    }
    if(action & L_SHOLDER)
    {
        bLeftSholderAirBagValve = VALVE_ON ;
    }
    else
    {
        bLeftSholderAirBagValve = VALVE_OFF ;
    }
    if(action & R_SHOLDER)
    {
        bRightSholderAirBagValve = VALVE_ON ;
    }
    else
    {
        bRightSholderAirBagValve = VALVE_OFF ;
    }
    
}

void ArmAirBagAction(bool Enable,unsigned int action)
{    
    if(!Enable)
    {
        bRightArmUpAirBagValve1   =  VALVE_OFF ;
        bRightArmUpAirBagValve2   =  VALVE_OFF ;
        bRightArmUpAirBagValve3   =  VALVE_OFF ;
        bLeftArmUpAirBagValve1    =  VALVE_OFF ;
        bLeftArmUpAirBagValve2    =  VALVE_OFF ;
        bLeftArmUpAirBagValve3    =  VALVE_OFF ;
        return;
    }
    if(action & R_ARM_1)
    {
        bRightArmUpAirBagValve1 = VALVE_ON ;
    }
    else
    {
        bRightArmUpAirBagValve1 = VALVE_OFF ;
    }
    if(action & R_ARM_2)
    {
        bRightArmUpAirBagValve2 = VALVE_ON ;
    }
    else
    {
        bRightArmUpAirBagValve2 = VALVE_OFF ;
    }
    if(action & R_ARM_3)
    {
        bRightArmUpAirBagValve3 = VALVE_ON ;
    }
    else
    {
        bRightArmUpAirBagValve3 = VALVE_OFF ;
    }
    if(action & L_ARM_1)
    {
        bLeftArmUpAirBagValve1 = VALVE_ON ;
    }
    else
    {
        bLeftArmUpAirBagValve1 = VALVE_OFF ;
    }
    if(action & L_ARM_2)
    {
        bLeftArmUpAirBagValve2 = VALVE_ON ;
    }
    else
    {
        bLeftArmUpAirBagValve2 = VALVE_OFF ;
    }
    if(action & L_ARM_3)
    {
        bLeftArmUpAirBagValve3 = VALVE_ON ;
    }
    else
    {
        bLeftArmUpAirBagValve3 = VALVE_OFF ;
    }
}


bool AirBagGetNextStep(st_AirBag* pBag)  //bNextStep是真或假，即返回是否执行下一步骤
{
    bool bNextStep = FALSE;
    unsigned char counter = pBag->nAirBagCounter;
    switch(nKeyAirBagStrength)
    {
    case 1:
        if(counter > pBag->nCurKeepTime1)
        {
            bNextStep = TRUE ;
        }
        break ;
    case 2:
        if(counter > pBag->nCurKeepTime2)
        {
            bNextStep = TRUE ;
        }
        break ;
    case 3:
        if(counter > pBag->nCurKeepTime3)
        {
            bNextStep = TRUE ;
        }
        break ;
    case 4:
        if(counter > pBag->nCurKeepTime4)
        {
            bNextStep = TRUE ;
        }
        break ;
    case 5:
        if(counter > pBag->nCurKeepTime5)
        {
            bNextStep = TRUE ;
        }
        break ;
    }
    return (bNextStep);
}

void Valve_CloseAll(void)  
{
    BITS_ValveData[0].nByte = 0;
    BITS_ValveData[1].nByte = 0;
    BITS_ValveData[2].nByte = 0;
    Valve_LegFootAirPumpACPowerOff();
    Valve_BodyUpAirPumpACPowerOff();
}
void Valve_SetStretchUp(void)
{
  
      static int step = 0;
    Valve_LegFootAirPumpACPowerOff();
    Valve_BodyUpAirPumpACPowerOff();
    
bLeftArmUpAirBagValve3  = VALVE_OFF;          //左胳膊后面--
 bLeftArmUpAirBagValve2  = VALVE_OFF;           //左胳膊中间--
 bLeftArmUpAirBagValve1  = VALVE_OFF;	         //左胳膊前面
                  
 bRightArmUpAirBagValve1  = VALVE_OFF;          //右背腰下面
 bRightArmUpAirBagValve2  = VALVE_OFF;         //右背腰上面
bRightArmUpAirBagValve3 	 = VALVE_OFF;  //右肩

      bRightFootAirBagValve = VALVE_OFF;
      bLeftFootAirBagValve = VALVE_OFF;
      bFootHeelAirBagValve = VALVE_OFF;

  
  
    //Valve_LegFootAirPumpACPowerOff();
    //小腿气囊
    bLegLeftAirBagValve = VALVE_OFF;
    bLegRightAirBagValve = VALVE_OFF;
    //足部气囊
    //bRightFootAirBagValve = VALVE_OFF;
   // bLeftFootAirBagValve = VALVE_OFF;
    //bFootHeelAirBagValve = VALVE_OFF;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_OFF ;
    bRightThighAirBagValve 	=  VALVE_OFF ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    bBackWaistRightUp	  =  VALVE_OFF ;
    bBackWaistRightDown   =  VALVE_OFF ;
    bBackWaistLeftUp      =  VALVE_OFF ;  
    bBackWaistLeftDown    =  VALVE_OFF ;

    bLeftThighDownAir            =  VALVE_OFF ;
    bRightThighDownAir           =  VALVE_OFF ;
}

/*
void Valve_SetStretchUp(void)
{
  
 static int step = 0;
 Valve_LegFootAirPumpACPowerOn();
    //臂肩气囊
    switch(step)
    {
     case 0: 

      //足部气囊
      bRightFootAirBagValve = VALVE_ON;
      bLeftFootAirBagValve = VALVE_OFF;
      bFootHeelAirBagValve = VALVE_OFF;
      SholderTime = 0;
      step++;
      break;
     case 1: 
      if(SholderTime > 4000)
        step++;
      break;       
     case 2:

      //足部气囊
      bRightFootAirBagValve = VALVE_OFF;
      bLeftFootAirBagValve = VALVE_ON;
      bFootHeelAirBagValve = VALVE_OFF;
      step++;
      SholderTime = 0;
      break; 
     case 3: 
      if(SholderTime > 4000)
        step=0;
      break;       
                 
    }  
  
  
  
    //Valve_LegFootAirPumpACPowerOff();
    //小腿气囊
    bLegLeftAirBagValve = VALVE_OFF;
    bLegRightAirBagValve = VALVE_OFF;
    //足部气囊
    //bRightFootAirBagValve = VALVE_OFF;
   // bLeftFootAirBagValve = VALVE_OFF;
    //bFootHeelAirBagValve = VALVE_OFF;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_OFF ;
    bRightThighAirBagValve 	=  VALVE_OFF ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    bBackWaistRightUp	  =  VALVE_OFF ;
    bBackWaistRightDown   =  VALVE_OFF ;
    bBackWaistLeftUp      =  VALVE_OFF ;  
    bBackWaistLeftDown    =  VALVE_OFF ;

}
*/
void Valve_SetStretchCharge(unsigned int start)
{
    Valve_BodyUpAirPumpACPowerOn();
    Valve_LegFootAirPumpACPowerOn();
    static int step = 0;
    if(start)
    {
        step =  0; 
        SholderTime = 0;
    }
    //臂肩气囊
    switch(step)
    {
    case 0: bLeftSholderAirBagValve  = VALVE_ON ; 
    bRightSholderAirBagValve  = VALVE_ON ; 
    SholderTime = 0;
    step++;
    break;
    case 1: if(SholderTime > 500)
        step++;
        break;       
    case 2: bLeftSholderAirBagValve  = VALVE_ON ; 
    bRightSholderAirBagValve  = VALVE_ON ;
    step++;
    SholderTime = 0;
    break;       
    case 3: if(SholderTime > 200)
        step = 0;
        break;               
    }
    //小腿气囊
    bLegLeftAirBagValve = VALVE_ON;
    bLegRightAirBagValve = VALVE_ON;
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
//    bFootHeelAirBagValve = VALVE_ON;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_OFF ;
    bRightThighAirBagValve 	=  VALVE_OFF ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_ON ;
    bRightSholderAirBagValve  =  VALVE_ON ;
    //背腰气囊
    bBackWaistRightUp	  =  VALVE_ON ;
    bBackWaistRightDown   =  VALVE_ON ;
    bBackWaistLeftUp      =  VALVE_ON ;  
    bBackWaistLeftDown    =  VALVE_ON ;
    
    bLeftThighDownAir            =  VALVE_ON ;
    bRightThighDownAir           =  VALVE_ON ;    
    
}
void Valve_SetStretchChargedown(unsigned int start)
{
    Valve_BodyUpAirPumpACPowerOn();
    Valve_LegFootAirPumpACPowerOn();
    static int step = 0;
    if(start)
    {
        step =  0; 
        SholderTime = 0;
    }
    //臂肩气囊
    switch(step)
    {
    case 0: bLeftSholderAirBagValve  = VALVE_ON ; 
    bRightSholderAirBagValve  = VALVE_ON ; 
    SholderTime = 0;
    step++;
    break;
    case 1: if(SholderTime > 500)
        step++;
        break;       
    case 2: bLeftSholderAirBagValve  = VALVE_ON ; 
    bRightSholderAirBagValve  = VALVE_ON ;
    step++;
    SholderTime = 0;
    break;       
    case 3: if(SholderTime > 200)
        step = 0;
        break;               
    }
    //小腿气囊
    bLegLeftAirBagValve = VALVE_ON;
    bLegRightAirBagValve = VALVE_ON;
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
   // bFootHeelAirBagValve = VALVE_ON;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_ON ;
    bRightThighAirBagValve 	=  VALVE_ON ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_ON ;
    bRightSholderAirBagValve  =  VALVE_ON ;
    //背腰气囊
    bBackWaistRightUp	  =  VALVE_ON ;
    bBackWaistRightDown   =  VALVE_ON ;
    bBackWaistLeftUp      =  VALVE_ON ;  
    bBackWaistLeftDown    =  VALVE_ON ;
}
void Valve_SetStretchChargepre(unsigned int start)
{
    Valve_BodyUpAirPumpACPowerOn();
    Valve_LegFootAirPumpACPowerOn();
    static int step = 0;
    if(start)
    {
        step =  0; 
        SholderTime = 0;
    }
    //臂肩气囊
    switch(step)
    {
    case 0: bLeftSholderAirBagValve  = VALVE_ON ; 
    bRightSholderAirBagValve  = VALVE_ON ; 
    SholderTime = 0;
    step++;
    break;
    case 1: if(SholderTime > 500)
        step++;
        break;       
    case 2: bLeftSholderAirBagValve  = VALVE_ON ; 
    bRightSholderAirBagValve  = VALVE_ON ;
    step++;
    SholderTime = 0;
    break;       
    case 3: if(SholderTime > 200)
        step = 0;
        break;               
    }
    //小腿气囊
    bLegLeftAirBagValve = VALVE_OFF;
    bLegRightAirBagValve = VALVE_OFF;
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
  //  bFootHeelAirBagValve = VALVE_ON;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_ON ;
    bRightThighAirBagValve 	=  VALVE_ON ;  
    //胳膊气囊
    //bLeftSholderAirBagValve   =  VALVE_ON ;
    //bRightSholderAirBagValve  =  VALVE_ON ;
    //背腰气囊
    bBackWaistRightUp	  =  VALVE_OFF ;
    bBackWaistRightDown   =  VALVE_OFF ;
    bBackWaistLeftUp      =  VALVE_OFF ;  
    bBackWaistLeftDown    =  VALVE_OFF ;
    
    bRightThighDownAir	    =  VALVE_ON ;    
    bLeftThighDownAir       =  VALVE_ON ;
}
void Valve_SetStretchChargeATOUT(unsigned int start)
{
    Valve_BodyUpAirPumpACPowerOn();
    Valve_LegFootAirPumpACPowerOn();
    static int step = 0;
    if(start)
    {
        step =  0; 
        SholderTime = 0;
    }
    //臂肩气囊
    switch(step)
    {
    case 0: bLeftSholderAirBagValve  = VALVE_ON ; 
    bRightSholderAirBagValve  = VALVE_ON ; 
    SholderTime = 0;
    step++;
    break;
    case 1: if(SholderTime > 500)
        step++;
        break;       
    case 2: bLeftSholderAirBagValve  = VALVE_ON ; 
    bRightSholderAirBagValve  = VALVE_ON ;
    step++;
    SholderTime = 0;
    break;       
    case 3: if(SholderTime > 200)
        step = 0;
        break;               
    }
    //小腿气囊
    bLegLeftAirBagValve = VALVE_ON;
    bLegRightAirBagValve = VALVE_ON;
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_OFF;
    //大腿气囊
    bLeftThighAirBagValve       = VALVE_ON; //VALVE_ON ;
    bRightThighAirBagValve 	= VALVE_ON;// V/ALVE_ON ;  
    //胳膊气囊
    //bLeftSholderAirBagValve   =  VALVE_ON ;
    //bRightSholderAirBagValve  =  VALVE_ON ;
    //背腰气囊
    bBackWaistRightUp	  =  VALVE_OFF ;
    bBackWaistRightDown   =  VALVE_OFF ;
    bBackWaistLeftUp      =  VALVE_OFF ;  
    bBackWaistLeftDown    =  VALVE_OFF ;
    
    bRightThighDownAir	    =  VALVE_ON ;    
    bLeftThighDownAir       =  VALVE_ON ;
}
void Valve_SetStretchChargeATOUT2(unsigned int start)
{
    Valve_BodyUpAirPumpACPowerOn();
    Valve_LegFootAirPumpACPowerOn();
    static int step = 0;
    if(start)
    {
        step =  0; 
        SholderTime = 0;
    }
    //臂肩气囊
    switch(step)
    {
    case 0: bLeftSholderAirBagValve  = VALVE_ON ; 
    bRightSholderAirBagValve  = VALVE_ON ; 
    SholderTime = 0;
    step++;
    break;
    case 1: if(SholderTime > 500)
        step++;
        break;       
    case 2: bLeftSholderAirBagValve  = VALVE_ON ; 
    bRightSholderAirBagValve  = VALVE_ON ;
    step++;
    SholderTime = 0;
    break;       
    case 3: if(SholderTime > 200)
        step = 0;
        break;               
    }
    //小腿气囊
    bLegLeftAirBagValve = VALVE_OFF;
    bLegRightAirBagValve = VALVE_OFF;
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_OFF;
    //大腿气囊
    bLeftThighAirBagValve       =VALVE_OFF; // VALVE_ON ;
    bRightThighAirBagValve 	= VALVE_OFF;/// VALVE_ON ;  
    //胳膊气囊
    //bLeftSholderAirBagValve   =  VALVE_ON ;
    //bRightSholderAirBagValve  =  VALVE_ON ;
    //背腰气囊
    bBackWaistRightUp	  =  VALVE_OFF ;
    bBackWaistRightDown   =  VALVE_OFF ;
    bBackWaistLeftUp      =  VALVE_OFF ;  
    bBackWaistLeftDown    =  VALVE_OFF ;
}
void Valve_SetStretchChargeATOUTFootHeelOFF(unsigned int start)
{
    Valve_BodyUpAirPumpACPowerOn();
    Valve_LegFootAirPumpACPowerOn();
    static int step = 0;
    if(start)
    {
        step =  0; 
        SholderTime = 0;
    }
    //臂肩气囊
    switch(step)
    {
    case 0: bLeftSholderAirBagValve  = VALVE_ON ; 
    bRightSholderAirBagValve  = VALVE_ON ; 
    SholderTime = 0;
    step++;
    break;
    case 1: if(SholderTime > 500)
        step++;
        break;       
    case 2: bLeftSholderAirBagValve  = VALVE_ON ; 
    bRightSholderAirBagValve  = VALVE_ON ;
    step++;
    SholderTime = 0;
    break;       
    case 3: if(SholderTime > 200)
        step = 0;
        break;               
    }
    //小腿气囊
    bLegLeftAirBagValve = VALVE_OFF;
    bLegRightAirBagValve = VALVE_OFF;
    //足部气囊
    bRightFootAirBagValve = VALVE_OFF;
    bLeftFootAirBagValve = VALVE_OFF;
    bFootHeelAirBagValve = VALVE_OFF;
    //大腿气囊
    bLeftThighAirBagValve       = VALVE_OFF;// VALVE_ON ;
    bRightThighAirBagValve 	= VALVE_OFF;// VALVE_ON ;  
    //胳膊气囊
    //bLeftSholderAirBagValve   =  VALVE_ON ;
    //bRightSholderAirBagValve  =  VALVE_ON ;
    //背腰气囊
    bBackWaistRightUp	  =  VALVE_OFF ;
    bBackWaistRightDown   =  VALVE_OFF ;
    bBackWaistLeftUp      =  VALVE_OFF ;  
    bBackWaistLeftDown    =  VALVE_OFF ;
}
void Valve_SetStretchChargeOut(unsigned int start)
{
  Valve_SetStretchCharge(start);
   //小腿气囊
    bLegLeftAirBagValve = VALVE_OFF;
    bLegRightAirBagValve = VALVE_OFF;
}

void Valve_SetStretchHold(void)
{
   // static int step = 0;
    Valve_BodyUpAirPumpACPowerOn();
    Valve_LegFootAirPumpACPowerOn();
    //小腿气囊
    bLegLeftAirBagValve = VALVE_OFF;
    bLegRightAirBagValve = VALVE_OFF;
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_ON;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_ON ;
    bRightThighAirBagValve 	=  VALVE_ON ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    bBackWaistRightUp	  =  VALVE_OFF ;
    bBackWaistRightDown   =  VALVE_OFF ;
    bBackWaistLeftUp      =  VALVE_OFF ;  
    bBackWaistLeftDown    =  VALVE_OFF ;
}
void Valve_SetStretchHoldHeelOFF(void)
{
   // static int step = 0;
    Valve_BodyUpAirPumpACPowerOn();
    Valve_LegFootAirPumpACPowerOn();
    //小腿气囊
    bLegLeftAirBagValve = VALVE_ON;
    bLegRightAirBagValve = VALVE_ON;
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_OFF;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_OFF ;
    bRightThighAirBagValve 	=  VALVE_OFF ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    bBackWaistRightUp	  =  VALVE_OFF ;
    bBackWaistRightDown   =  VALVE_OFF ;
    bBackWaistLeftUp      =  VALVE_OFF ;  
    bBackWaistLeftDown    =  VALVE_OFF ;
    
    bLeftThighDownAir            =  VALVE_OFF ;
    bRightThighDownAir           =  VALVE_OFF ;
}
void Valve_SetStretchHoldHeelSCONDOFF(void)
{
   // static int step = 0;
    Valve_BodyUpAirPumpACPowerOn();
    Valve_LegFootAirPumpACPowerOn();
    //小腿气囊
    bLegLeftAirBagValve = VALVE_OFF;
    bLegRightAirBagValve = VALVE_OFF;
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_ON;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_OFF ;
    bRightThighAirBagValve 	=  VALVE_OFF ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    bBackWaistRightUp	  =  VALVE_OFF ;
    bBackWaistRightDown   =  VALVE_OFF ;
    bBackWaistLeftUp      =  VALVE_OFF ;  
    bBackWaistLeftDown    =  VALVE_OFF ;
}
void Valve_SetStretch_ON_OFF(uint16_t Valve_ON_OFF)
{
    Valve_BodyUpAirPumpACPowerOn();
    Valve_LegFootAirPumpACPowerOn();
    //小腿气囊
    bLegLeftAirBagValve = Valve_ON_OFF>>0;
    bLegRightAirBagValve = Valve_ON_OFF>>1;
    //足部气囊
    bRightFootAirBagValve = Valve_ON_OFF>>2;
    bLeftFootAirBagValve = Valve_ON_OFF>>3;
    bFootHeelAirBagValve = Valve_ON_OFF>4;
    //大腿气囊
    bLeftThighAirBagValve       =  Valve_ON_OFF>>5 ;
    bRightThighAirBagValve 	=  Valve_ON_OFF>>6 ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  Valve_ON_OFF>>7 ;
    bRightSholderAirBagValve  =  Valve_ON_OFF>>8 ;
    //背腰气囊
    bBackWaistRightUp	  =  Valve_ON_OFF>>9 ;
    bBackWaistRightDown   =  Valve_ON_OFF>>10 ;
    bBackWaistLeftUp      =  Valve_ON_OFF>>11 ;  
    bBackWaistLeftDown    =  Valve_ON_OFF>>12 ;
    
    bLeftThighDownAir     =  Valve_ON_OFF>>13 ;
    bRightThighDownAir    =  Valve_ON_OFF>>14 ;
}
unsigned char Valve_Level_Decrease(unsigned char by_Data)
{
    unsigned char retval;
    unsigned int w_Data;
    unsigned int mod;
    if(by_Data <= 5) 
    {
        retval = by_Data;
    }
    else
    {
        w_Data = by_Data;
        w_Data *= 10;
        mod = w_Data % 15;
        w_Data /= 15;
        if(mod > 7) w_Data++;
        by_Data = (unsigned char)w_Data;
        retval = by_Data;
    }
    return retval;
}

unsigned char Valve_Level_Increase(unsigned char by_Data)
{
    unsigned char retval;
    unsigned int w_Data;
    unsigned int mod;
    if(by_Data <= 5) 
    {
        retval = by_Data;
    }
    else
    {
        w_Data = by_Data;
        w_Data *= 15;
        w_Data /= 10;
        mod = w_Data % 10;
        if(mod > 5) w_Data++;
        if(w_Data > 255) w_Data = 255;
        by_Data = (unsigned char)w_Data;
        retval = by_Data;
    }
    return retval;
}

void Valve_SetEnableSholder(unsigned int enable) //当背部主运行模式为自动模式时置0，否则置1，实际上此函数未使用
{
    bSholderEnable = enable;
}
#define VALVE_CLOSE_ALL_AIRPUMP  0
#define VALVE_OPEN_ALL_AIRPUMP   1
#define VALVE_NORMAL   2
void Valve_Airpump_Ctrl(unsigned int ctrl)
{
  if(ctrl == VALVE_CLOSE_ALL_AIRPUMP)
  {
    Valve_BodyUpAirPumpACPowerOff();
    Valve_LegFootAirPumpACPowerOff();
    return;
  }
  if(ctrl == VALVE_OPEN_ALL_AIRPUMP)
  {
    Valve_BodyUpAirPumpACPowerOn();
    Valve_LegFootAirPumpACPowerOn();
    return;
  }
  
  if(bBackWaistRightUp ||
     bBackWaistRightDown ||
       bBackWaistLeftUp ||
         bBackWaistLeftDown ||
           bRightSholderAirBagValve ||
             bRightArmUpAirBagValve1 ||
               bRightArmUpAirBagValve2 ||
                 bRightArmUpAirBagValve3 ||
                   bLeftArmUpAirBagValve1 ||
                     bLeftArmUpAirBagValve2 ||
                       bLeftArmUpAirBagValve3 ||
                         bLeftSholderAirBagValve||bLeftThighAirBagValve ||
                  bRightThighAirBagValve)
  {
    Valve_BodyUpAirPumpACPowerOn();
  }
  else
  {
  //  Valve_BodyUpAirPumpACPowerOff();
	
  } 
    if(bLegLeftAirBagValve ||
        bLegRightAirBagValve ||
          bFootHeelAirBagValve ||
            bRightFootAirBagValve ||
              bLeftFootAirBagValve /*||
                bLeftThighAirBagValve ||
                  bRightThighAirBagValve*/)
  {
    Valve_LegFootAirPumpACPowerOn();
  }
  else
  {
   // Valve_LegFootAirPumpACPowerOff();
  }       
}
void Valve_Control(unsigned char nAirBagSwitch,st_AirBag* pBag,unsigned char level)
{
    bool bNextStep = FALSE;
    // static unsigned int SholdTime;
    
    /*
    if(!bSholderEnable)
    {
    if((Input_GetWalkMotorPosition() > 40 ))
    {
    bSholderAirBagValve = VALVE_OFF ;
}
}
    */
    if(nAirBagSwitch == VALVE_DISABLE)
    {
     //   Valve_Airpump_Ctrl(VALVE_CLOSE_ALL_AIRPUMP);
        switch(pBag->locate)
        {
        case AIRBAG_LOCATE_LEG_FOOT:        LegFootAirBagAction(FALSE,pBag->nCurPumpValveState); break;
        case AIRBAG_LOCATE_SEAT:            SeatAirBagAction(FALSE,pBag->nCurPumpValveState); break;
        case AIRBAG_LOCATE_BACK_WAIST:      BackWaistAirBagAction(FALSE,pBag->nCurPumpValveState); break;
        case AIRBAG_LOCATE_ARM_SHOLDER:     ArmSholderAirBagAction(FALSE,pBag->nCurPumpValveState); break;
        case AIRBAG_LOCATE_ARM:     ArmAirBagAction(FALSE,pBag->nCurPumpValveState); break;
        case AIRBAG_LOCATE_AUTO:
            LegFootAirBagAction(FALSE,pBag->nCurPumpValveState); 
            SeatAirBagAction(FALSE,pBag->nCurPumpValveState); 
            BackWaistAirBagAction(FALSE,pBag->nCurPumpValveState); 
            ArmSholderAirBagAction(FALSE,pBag->nCurPumpValveState); 
            break;
        case AIRBAG_LOCATE_ARM_SHOLDER_WAIST: 
            BackWaistAirBagAction(FALSE,pBag->nCurPumpValveState); 
            ArmSholderAirBagAction(FALSE,pBag->nCurPumpValveState);
            break;
        case AIRBAG_LOCATE_LEG_FOOT_SEAT: 
            LegFootAirBagAction(FALSE,pBag->nCurPumpValveState); 
            SeatAirBagAction(FALSE,pBag->nCurPumpValveState);   
            break;
            
        }
        return; 
    }
    if(pBag->init == TRUE)
    {
        bNextStep = TRUE;
    }
    else
    {
        bNextStep = AirBagGetNextStep(pBag);   
    }
    if(bNextStep == TRUE)
    {
        if(pBag->locate == AIRBAG_LOCATE_LEG_FOOT)
        {
            w_RollerCounter = 0;
        }
        if(pBag->init == TRUE)
        {
            pBag->init = FALSE;
            pBag->nCurAirBagStep = 0;
            pBag->nAirBagCounter = 0;
        }
        else
        {
            pBag->nCurAirBagStep++;
            tt_step=  pBag->nCurAirBagStep;
//			//==================================================
//			 if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && ( nBackSubRunMode == BACK_SUB_MODE_AUTO_10))
//                         {
//                   //          if((pBag->locate == AIRBAG_LOCATE_LEG_FOOT_SEAT)&&(st_GrowthStretch.active ==FALSE )&&(Data_Get_Time() >= 5)   )
//                             if(/*(pBag->locate == AIRBAG_LOCATE_LEG_FOOT_SEAT)&&*/(st_GrowthStretch.active ==FALSE )&&(Data_Get_Time() >= 5)   )
//                              {
//                                    //////////////////////////////
//                                    if((pBag->nCurAirBagStep ==  GrowthStepMaxA)||
//                                       (pBag->nCurAirBagStep ==  GrowthStepMaxA+GrowthStepMaxB))
//                                       
//                                    {
//                                      st_GrowthStretch.active = TRUE;
//                                      st_GrowthStretch.init = TRUE;
//                                    }
//                                    /////////////////////////////
//                              }
//                         }
			
			//=====================================================
            if(pBag->nCurAirBagStep >= pBag->nTotalSteps)
            {
                pBag->nCurAirBagStep = 0;
            }
        }
        pBag->nCurPumpValveState = pBag->pAirBagArray[pBag->nCurAirBagStep].nPumpValveState ;
        pBag->nCurKeepTime1 = pBag->pAirBagArray[pBag->nCurAirBagStep].nKeepTime1 ;
        pBag->nCurKeepTime3 = pBag->pAirBagArray[pBag->nCurAirBagStep].nKeepTime2 ;
        pBag->nCurKeepTime5 = pBag->pAirBagArray[pBag->nCurAirBagStep].nKeepTime3 ;
        
        if(pBag->nCurPumpValveState != ALL_DIS)
        {
            if(level  == 0)
            {
                pBag->nCurKeepTime1 = Valve_Level_Decrease(pBag->nCurKeepTime1);
                pBag->nCurKeepTime3 = Valve_Level_Decrease(pBag->nCurKeepTime3);
                pBag->nCurKeepTime5 = Valve_Level_Decrease(pBag->nCurKeepTime5);
            }
            if(level  == 2)
            {
                pBag->nCurKeepTime1 = Valve_Level_Increase(pBag->nCurKeepTime1);
                pBag->nCurKeepTime3 = Valve_Level_Increase(pBag->nCurKeepTime3);
                pBag->nCurKeepTime5 = Valve_Level_Increase(pBag->nCurKeepTime5);
            }
        }
        pBag->nCurKeepTime2 = Middle(pBag->nCurKeepTime1,pBag->nCurKeepTime3);
        pBag->nCurKeepTime4 = Middle(pBag->nCurKeepTime3,pBag->nCurKeepTime5);
        pBag->nAirBagCounter = 0;
        switch(pBag->locate)
        {
        case AIRBAG_LOCATE_LEG_FOOT:        LegFootAirBagAction(TRUE,pBag->nCurPumpValveState); break;
        case AIRBAG_LOCATE_SEAT:            SeatAirBagAction(TRUE,pBag->nCurPumpValveState); break;
        case AIRBAG_LOCATE_BACK_WAIST:      BackWaistAirBagAction(TRUE,pBag->nCurPumpValveState); break;
        case AIRBAG_LOCATE_ARM_SHOLDER:     ArmSholderAirBagAction(TRUE,pBag->nCurPumpValveState); break;
        case AIRBAG_LOCATE_ARM:             ArmSholderAirBagAction(TRUE,pBag->nCurPumpValveState); break;
        case AIRBAG_LOCATE_AUTO:
            LegFootAirBagAction(TRUE,pBag->nCurPumpValveState); 
            SeatAirBagAction(TRUE,pBag->nCurPumpValveState); 
            BackWaistAirBagAction(TRUE,pBag->nCurPumpValveState); 
            ArmSholderAirBagAction(TRUE,pBag->nCurPumpValveState); 
            break;
        case AIRBAG_LOCATE_ARM_SHOLDER_WAIST: 
            BackWaistAirBagAction(TRUE,pBag->nCurPumpValveState); 
            ArmSholderAirBagAction(TRUE,pBag->nCurPumpValveState);
            break;
        case AIRBAG_LOCATE_LEG_FOOT_SEAT: 
            LegFootAirBagAction(TRUE,pBag->nCurPumpValveState); 
            SeatAirBagAction(TRUE,pBag->nCurPumpValveState); 
            break;   
        }
       // Valve_Airpump_Ctrl(VALVE_NORMAL);
    }
     Valve_Airpump_Ctrl(VALVE_OPEN_ALL_AIRPUMP);
}

//24-31位为滚轮旋转方式
/*  0-1位为滚轮速度 00 停止 01 慢速 10 中速 11 高速
2-3位为滚轮旋转方式 01短间歇 02长间歇 10 连续
4-7位保留
*/

void Valve_LegKneadProce(unsigned char bLegKneadEnable,unsigned char Valve_Enable,st_AirBag* pBag)
{
  unsigned int speed,mode;
  if(bLegKneadFlag == 0) return;//10ms update flag
  bLegKneadFlag = 0;
  
  if(!bLegKneadEnable)
  {
          LegKnead_SetPower(LEG_KNEAD_OFF);
          LegKnead_SetSpeed(LEG_KNEAD_SPEED_STOP);
    
  }
  
 // bLegKneadEnable = 1; //test
 /**     if( !bKneadLegEnable)
      {
          LegKnead_SetPower(LEG_KNEAD_OFF);
          LegKnead_SetSpeed(LEG_KNEAD_SPEED_STOP);
          bLegKneadEnableonly = TRUE;
        return;    
      }
      LegKnead_SetPower(LEG_KNEAD_ON);*/
  if(Valve_Enable)  //自动揉搓模式
  {
      if( !bKneadLegEnable)
      {
          LegKnead_SetPower(LEG_KNEAD_OFF);
          LegKnead_SetSpeed(LEG_KNEAD_SPEED_STOP);
          bLegKneadEnableonly = TRUE;
        return;    
      }
      LegKnead_SetPower(LEG_KNEAD_ON);
    
    
    
    speed = ((pBag->nCurPumpValveState >> 21) & 0x03);
    switch(speed)
    {
    default:  
    case 0:LegKnead_SetSpeed(LEG_KNEAD_SPEED_STOP);  break ;  
    case 1:LegKnead_SetSpeed(LEG_KNEAD_SPEED_SLOW);  break ;
    case 2:LegKnead_SetSpeed(LEG_KNEAD_SPEED_MID);  break ;
    case 3:LegKnead_SetSpeed(LEG_KNEAD_SPEED_FAST);  break ;
    }
    mode = ((pBag->nCurPumpValveState >> 23) & 0x03);
    switch(mode)
    {
    case 0:LegKnead_SetMode(LEG_KNEAD_TO_IN);  break ;  
    case 1:LegKnead_SetMode(LEG_KNEAD_TO_OUT);  break ;
    case 2:LegKnead_SetMode(LEG_KNEAD_TO_SWAY);  break ;
    default:
    case 3:  break ;
    }   
    return; 
  }
  //手动滚轮模式
  LegKnead_SetPower(LEG_KNEAD_ON);
  switch(LegKneadSpeed)
  {
  default:  
  case 0:LegKnead_SetSpeed(LEG_KNEAD_SPEED_STOP);  break ;  
  case 1:LegKnead_SetSpeed(LEG_KNEAD_SPEED_SLOW);  break ;
  case 2:LegKnead_SetSpeed(LEG_KNEAD_SPEED_MID);  break ;
  case 3:LegKnead_SetSpeed(LEG_KNEAD_SPEED_FAST);  break ;
  }
  switch(LegKneadMode)
  {
  case 0:LegKnead_SetMode(LEG_KNEAD_TO_IN);  break ;  
  case 1:LegKnead_SetMode(LEG_KNEAD_TO_OUT);  break ;
  case 2:LegKnead_SetMode(LEG_KNEAD_TO_SWAY);  break ;
  default:
  case 3:  break ;
  }   
  
  
  
}

void Valve_FootRollerProce(unsigned char bRollerEnable,unsigned char Valve_Enable,st_AirBag* pBag)
{ 
  unsigned int speed,mode;
    if(bRollerFlag == 0) return;
    bRollerFlag = 0;
    if(!bRollerEnable)
    {
       Roller_SetSpeed(ROLLER_SPEED_STOP);
       return;
    }
    if(Valve_Enable)  //自动滚轮模式
     {
    bAutoRoller = true;   
    speed = ((pBag->nCurPumpValveState >> 25) & 0x03);
    switch(speed)
    {
    default:  
    case 0:Roller_SetSpeed(ROLLER_SPEED_STOP);  break ;  
    case 1:Roller_SetSpeed(ROLLER_SPEED_SLOW);  break ;
    case 2:Roller_SetSpeed(ROLLER_SPEED_MID);  break ;
    case 3:Roller_SetSpeed(ROLLER_SPEED_FAST);  break ;
    }
    mode = ((pBag->nCurPumpValveState >> 27) & 0x07);
    Roller_SetMode(mode);
    return; 
  } 
    bAutoRoller = false;  //手动滚轮模式
    Roller_SetMode(ROLLER_MODE_CON_IN);
    Roller_SetSpeed(w_RollerPWM);
}

void Valve_SetRollerPWM(unsigned char level)
{
    switch(level)
    {
    case 0:  w_RollerPWM = ROLLER_SPEED_STOP; break;
    case 1:  w_RollerPWM = ROLLER_SPEED_SLOW; break;
    case 2:  w_RollerPWM = ROLLER_SPEED_MID;  break;
    case 3:  w_RollerPWM = ROLLER_SPEED_FAST; break;
    }
}

unsigned char Valve_GetRollerLevel(void)
{
    unsigned char level = 0;
    if(w_RollerPWM == ROLLER_SPEED_SLOW) level = 1;
    if(w_RollerPWM == ROLLER_SPEED_MID) level = 2;
    if(w_RollerPWM == ROLLER_SPEED_FAST) level = 3;
    return level;
}

void Valve_SetLegKneadSpeed(unsigned char speed)
{
  if(speed > LEG_KNEAD_SPEED_FAST) return;
  LegKneadSpeed = speed;
}
unsigned char Valve_GetLegKneadSpeed(void)
{
    return(LegKneadSpeed);
}

unsigned char Valve_GetAirBagStrength(void)
{
    return(nKeyAirBagStrength);
}

void Valve_SetAirBagStrength(unsigned char strength)
{
    nKeyAirBagStrength = strength;
}

void Valve_AddAirBagStrength(void)
{
    nKeyAirBagStrength++;
    if(nKeyAirBagStrength > 5) nKeyAirBagStrength =1;
}
//小腿和臀部
void Valve_LegFootAirPumpACPowerOn(void)
{
    GPIO_PinOutSet(VALVE_AIRPUMP2_PORT,VALVE_AIRPUMP2_BIT);
}
void Valve_LegFootAirPumpACPowerOff(void)
{
    GPIO_PinOutClear(VALVE_AIRPUMP2_PORT,VALVE_AIRPUMP2_BIT);
}
//臂肩，背腰，
void Valve_BodyUpAirPumpACPowerOn(void)
{
    GPIO_PinOutSet(VALVE_AIRPUMP1_PORT,VALVE_AIRPUMP1_BIT);
}
void Valve_BodyUpAirPumpACPowerOff(void)
{
    GPIO_PinOutClear(VALVE_AIRPUMP1_PORT,VALVE_AIRPUMP1_BIT);
}

void Valve_Test_Set_Data(unsigned int ValveTestData)
{
    BITS_ValveData[0].nByte = (unsigned char)ValveTestData;
    BITS_ValveData[1].nByte = (unsigned char)(ValveTestData >> 8);
    BITS_ValveData[2].nByte = (unsigned char)(ValveTestData >> 16);
}

void Valve_SetBackMode(int backauto) //背部自动模式时置1，否则置0
{
    bBackauto = (bool)backauto;
}

void  Valve_OzonOn(void)
{
     // GPIO_PinOutSet(VALVE_OZON_PORT,VALVE_OZON_BIT);
}
void  Valve_OzonOff(void)
{
   //  GPIO_PinOutClear(VALVE_OZON_PORT,VALVE_OZON_BIT);
}

int Valve_RollerIsAuto(void)
{
 return (bAutoRoller) ;
}


/******************************************************************************/
uint8_t nValveAlone=0;
uint16_t nValveAloneTime;
void Valve_Alone()
{
 if(nValveAlone!=1) 
 {
   nValveAloneTime = 0;
   return;
 }
 
 ArmSholderAirBagAction(true,L_SHOLDER|R_SHOLDER);
 SeatAirBagAction(true,L_THIGH|R_THIGH);
 
// BackWaistAirBagAction(true,0);
 //LegFootAirBagAction(true,0);
 if(nValveAloneTime>300)//8S
 {
  nValveAlone = 0; //退出，由数组控制气囊
 }
}
/******************************************************************************/