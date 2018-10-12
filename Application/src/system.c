//#include "efm32.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "EFM32_def.h"
#include "EFM32_types.h"
#include "em_letimer.h"
#include "em_vcmp.h"
#include "system.h"
#include "Timer.h"
#include "LegMotor.h"
#include "BackPad.h"
#include "WalkMotor.h"
#include "SlideMotor.h"
#include "em_gpio.h"
#include "input.h"
#include "Valve.h"
#include "Data_Cul.h"
#include "IndicateLED.h"
#include "ADC_Scan.h"
#include "LED_RGB.h"        
#include "KneadMotor.h"        
#include "Walkmotor.h"        
#include "AxisMotor.h"        
#include "em_wdog.h"
#include "HandUart_New.h"
#include "signalUart_new.h"
#include "VoiceUart.h"
#include "WaistHot.h"
#include "Flex_Uart.h"
#include "UartLeg.h"
#include "SingleLine.h"
//-----------新程序增加参数（system.c文件使用）start---------
//bool TimeCountStart;
//int ProgTime;
extern unsigned int bkneadTime;
extern unsigned char bkneadTimeFlag;
extern unsigned char WalkTimeFlag;
extern unsigned int WalkTime;
extern bool bkneadStopTimeFlag;
extern unsigned int bkneadStopTime;
extern bool _3D_TimeStopFlag;
unsigned short _3D_TotalTime;
//extern bool WalkStopCounterFlag;
//unsigned int WalkStopCounter;
bool ValveTimeFlag;
extern bool _3D_PosStop_TimeFlag;
unsigned char _3D_PosStop_Time;
extern bool pwm_time_flag;
extern unsigned char pwm_time;
unsigned char ValveTime;
//-------------------------------end---------------------------

extern void main_30ms_int(void);
extern void main_50ms_int(void);
extern void main_10ms_int(void);
extern void main_100ms_int(void);

extern void main_5ms_int(void);
extern void main_200ms_int(void);
bool  bTimer2MS;
unsigned int sysCounter=0;

extern unsigned long by_moni_cmd_tm;

uint16_t SYS_Back_time;

void System_Initial_IO(void)
{
    CHIP_Init();
    CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);  /** main system clock - internal RC 28MHz*/
    SystemCoreClockUpdate();
    CMU_ClockEnable(cmuClock_HFPER,true);  /** High frequency peripheral clock */
    CMU_ClockEnable(cmuClock_CORELE, true);/* Enable CORELE clock */
    CMU_ClockEnable(cmuClock_GPIO,true);   /** General purpose input/output clock. */
    CMU_ClockEnable(cmuClock_USART0,true); //165和valve
    CMU_ClockEnable(cmuClock_USART1,true); //与小腿通讯接口
    CMU_ClockEnable(cmuClock_USART2,true); //wifi接口
    CMU_ClockEnable(cmuClock_UART0,true);  //手控器接口
    CMU_ClockEnable(cmuClock_UART1,true);  //语音模块接口
    CMU_ClockEnable(cmuClock_LEUART1,true);  //机芯接口
    CMU_ClockEnable(cmuClock_ADC0, true);  
    CMU_ClockEnable(cmuClock_TIMER0,true); 
    CMU_ClockEnable(cmuClock_TIMER1,true);
    CMU_ClockEnable(cmuClock_TIMER2,true);
    CMU_ClockEnable(cmuClock_TIMER3,true);
    //CMU_ClockEnable(cmuClock_PRS, true);
    CMU_ClockEnable(cmuClock_DMA, true);        /* Enable DMA clock */
    //CMU_ClockEnable(cmuClock_LEUART0, true);    /* Enable LEUART1 clock */
    SysTick_Config(SystemCoreClock / 1000); //set 1ms interupt using systick
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO); //LFA选择内部32768时钟
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFRCO);
    //GPIO_PinModeSet(TEST_PORT, TEST_BIT, TEST_MODE, 1);
    CMU_ClockEnable(cmuClock_LETIMER0, true);  
    //LETIMER_setup();
    /* Enable underflow interrupt */  
    LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);  
    /* Enable LETIMER0 interrupt vector in NVIC*/
    NVIC_EnableIRQ(LETIMER0_IRQn);
    
    CMU_IntClear(CMU_IFC_LFXORDY);
    CMU_IntEnable(CMU_IEN_LFXORDY);
    NVIC_EnableIRQ(CMU_IRQn);
    CMU_OscillatorEnable(cmuOsc_LFXO,1,0);
    
    VCMP_Init_TypeDef vcmp =
    {
        true,                               /* Half bias current */
        0,                                  /* Bias current configuration */
        true,                               /* Enable interrupt for falling edge */
        false,                              /* Enable interrupt for rising edge */
        vcmpWarmTime256Cycles,              /* Warm-up time in clock cycles */
        vcmpHyst20mV,                       /* Hysteresis configuration */
        1,                                  /* Inactive comparator output value */
        false,                              /* Enable low power mode */
        VCMP_VoltageToLevel(2.5), /* Trigger level */
        false                               /* Enable VCMP after configuration */
    };
    /* Initialize VCMP */
    CMU_ClockEnable(cmuClock_VCMP, true);
    VCMP_Init(&vcmp);
    
    /* Enable VCMP interrupt lines */
    NVIC_EnableIRQ(VCMP_IRQn);
    VCMP_IntEnable(VCMP_IEN_EDGE | VCMP_IEN_WARMUP);
    
    /* Enable VCMP and wait for warm-up complete */
    VCMP_Enable();  
}
//1ms interupt

bool bFlag1ms;
void SysTick_Handler(void)
{
    static BYTE by_Time5ms = 0;
    static BYTE by_Time10ms = 0;
    static BYTE by_Time50ms = 0;
    static BYTE by_Time100ms = 0;
    static BYTE by_Time200ms = 0;

  static uint16_t Time_1s; 
  Time_1s++;
  if(Time_1s>=1000)
  {
   Time_1s =0;
   Data_Flag_Int();
  }
  SingleLine_TimeOut();  
    
    
    bFlag1ms = 1;
  //  sysCounter++;
    if(sysCounter>=2)
    {
        UartLeg_1ms_int();
        sysCounter=0;
    }
    else
    {
      sysCounter++;
    }
    
  //  Valve_1ms_Int();
 //   VoiceUart_1msInt();
    Input_Back_Pulse1MS();
       
 // UartLeg_1ms_int();//不可以放在这里，数据传输会漏帧

  //  UartLeg_RX_TimeoutInt();
    
    if(by_Time5ms >= 4)
    { 
      
  by_Time5ms=0;
      
        Valve_10ms_Int();
        Input_5ms_Int();
        Axis_5ms_Int();
      //  main_5ms_int();
        
        
    }                          
    else ++by_Time5ms;
    if(by_Time10ms >= 9)
    { 
        by_Time10ms = 0;  
        WDOG_Feed();

        SlideMotor_10ms_Int();
        BackMotor_10ms_int();
        LegMotor_10ms_int();
        KneadMotor_10ms_Int();
        main_10ms_int();
        LED_RGB_10ms_Int();
  
        SignalBoard_10msInt();//读取电机行程 开关计数
        AxisMotor_10msInt();
        WalkMotor_10msInt();
        Problem_10ms_Int();
        KnockMotor_10msInt();
        
         UartLeg_10msInt();
          if(_3D_PosStop_TimeFlag)
        {
           _3D_PosStop_Time++;
        }
        if(bkneadTimeFlag)
        {
          bkneadTime++;
        }
        if(WalkTimeFlag)
        {
          WalkTime++;
        }
        
    }                          
    else ++by_Time10ms;
    

    
    if(by_Time50ms >= 50)
    {                          
        by_Time50ms = 0;         
        main_50ms_int();

    }                          
    else ++by_Time50ms;
    
    if(by_Time100ms >= 100)
    {                          
        by_Time100ms = 0;  
        nValveAloneTime++;
        LED_RGB_100ms_Int();
        WaistHeat_100ms_Int();
     //   VoiceUart_100msInt();

        AxisMotor_100msInt();
        Timer_Flag_100ms_Int();
        Flex_100msInt();
	
        if(bkneadStopTimeFlag)
        {
          bkneadStopTime++;
        }
        if(ValveTimeFlag)
        {
          ValveTime++;
        }
        main_100ms_int();//100ms上传手控器一次 数据给
        
	 if(by_moni_cmd_tm>1)by_moni_cmd_tm--;   // 减到1不减。
	
	
    }                          
    else ++by_Time100ms;
  if(by_Time200ms >= 200)
  {                          
    by_Time200ms = 0;
    main_200ms_int();
    SYS_Back_time++;
  }                          
  else ++by_Time200ms;
}

void System_Delay_us(uint32_t ulData)
{
    while(ulData > 0)
    {
      __NOP();__NOP();__NOP();__NOP();__NOP();
      __NOP();__NOP();__NOP();__NOP();__NOP();
      __NOP();__NOP();__NOP();__NOP();__NOP();
      __NOP();__NOP();__NOP();__NOP();__NOP();
      ulData--;
    }
}

void CMU_IRQHandler(void)
{
    NVIC_DisableIRQ(CMU_IRQn);
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);  //选择外部32768时钟
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);  //选择外部32768时钟
    CMU_OscillatorEnable(cmuOsc_LFRCO,0,0);  //禁止内部32768时钟，省电 
}
/**************************************************************************//**
* @brief LETIMER0_IRQHandler
* Interrupt Service Routine for LETIMER
* 中断时间1秒钟
*****************************************************************************/
void LETIMER0_IRQHandler(void)
{ 
    /* Clear LETIMER0 underflow interrupt flag */
    LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
   // Data_Flag_Int();
    SingleLine_Rec();
}


unsigned int  System_GetCounter(void)
{ 
    return(sysCounter);
}

void System_clr_Counter(void)
{
  
  sysCounter=0;
}
/***************************************************************************//**
* @brief
*   VCMP interrupt handler, triggers on EDGE and WARMUP events
******************************************************************************/
extern unsigned int password;
void VCMP_IRQHandler()
{
    /* Execute on WARMUP interrupt */
    if (VCMP->IF & VCMP_IF_WARMUP)
    {
        /* Enable Low Power Reference */
        // VCMP_LowPowerRefSet(true);
        
        /* Clear interrupt flag */
        VCMP_IntClear(VCMP_IFC_WARMUP);
    }
    
    /* Execute on EDGE interrupt */
    if (VCMP->IF & VCMP_IF_EDGE)
    {
        /* Low voltage warning */
        password = 0;  //低电压清除password
        
        /* Clear interrupt flag */
        VCMP_IntClear(VCMP_IFC_EDGE);
    }
}

