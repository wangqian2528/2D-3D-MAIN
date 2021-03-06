#ifndef __MAIN_H__
#define __MAIN_H__

#include "comm.h"
#include "temperature.h"

#define  SELECT_3D  1   //#define  3D_MACHINE

#define MACHINEID             1
#define C_TIMER_TEMP        0
#define C_TIMER_RUN        1
#define C_TIMER_SLOW        2
#define C_TIMER_500MS       3
#define C_TIMER_WAVE_START  4
#define C_TIMER_5           5
#define C_TIME_RUBBING      6       //用于搓背程序
#define C_TIMER_INDICATE    7

#define  C_TIMER_ENG1   3    //用于工程模式
#define  C_TIMER_ENG2   0    //用于工程模式

#define A_STATE_IDLE          1
#define A_STATE_SETTLE        2
#define A_STATE_WAIT_COMMAND  3
#define A_STATE_PROBLEM       4    
#define A_STATE_WAIT_MEMORY   5
#define A_STATE_RUN           6
#define A_TEST                7

#define M_NONE          1
#define M_NEXT          2
#define	M_IDLE          3
#define	M_RUN           4
#define M_PROBLEM       5
#define	M_WAIT_COMMAND  6
#define	M_ENG           7
#define	M_SETTLE        8
#define	M_TEST          9

#define ZERO_POSITION_RESET         0  //所有的电动缸复位
#define ZERO_POSITION1              1  //第一个零重力点  
#define ZERO_POSITION2              2  //第二个零重力点  
#define ZERO_POSITION_STRETCH_UP    3  //向上拉退  
#define ZERO_POSITION_STRETCH_DOWN  4  //向下拉退  

#define MASSAGE_LEG_QUICK_POSITION   400

#define MASSAGE_BACK_OPTIMAL_POSITION		400//
#define MASSAGE_BACK_OPTIMAL_MED_POSITION       800//
#define MASSAGE_BACK_OPTIMAL1_POSITION		1150//



#define MASSAGE_LEG_OPTIMAL_POSITION		800//
#define MASSAGE_LEG_OPTIMAL_MED_POSITION        900//
#define MASSAGE_LEG_OPTIMAL1_POSITION		1040//
 

#define POSITION_CTRL_OFFSET                     30//  
#define POSITION_DISPLAY_OFFSET                  80






#define   BODY_DETECT_PREPARE           0
#define   BODY_DETECT_WALK_POSITION     1
#define   BODY_DETECT_KNEAD_MIN              2
#define   BODY_DETECT_KNEAD_MAX              3
#define   BODY_DETECT_3D_FORWARD              4
#define   BODY_DETECT_3D_4              5
#define   BODY_DETECT_DATA_CHECK        6
#define   BODY_DETECT_UP_AUTO           7
#define   BODY_DETECT_DATA_REFRESH      8
#define   BODY_DETECT_ADJ               9
#define   BODY_DETECT_WALK_POSITION_1   10
#define   BODY_DETECT_WALK_POSITION_2   11
#define   BODY_DETECT_OVER              12

#define   DETECT_NO_START           0
#define   DETECT_SHOULDER           1
#define   DETECT_3D                 2
#define   DETECT_FINISH             3
#define   DETECT_INITIAL            4 
#define   SHOULDER_DETECT_FINISH    5


//---------------------------------------------------------------
//#define MASSAGE_BACK_ROCK_POSITION        600//1500
#define MASSAGE_LEG_ROCK_POSITION        1040// 1200

#define MASSAGE_DIFFERENT_POSITION        50

//摇摆功能相关宏定义
#define RockDisable                   false
#define RockEnable                    true
#define ExitRock                      0
#define EnterRock                     1
#define StartRock                     0
#define LieDown                       1
#define LieUP                         2

//------------------------------------------------------------












/*
#define   BODY_DETECT_PREPARE           0
#define   BODY_DETECT_WALK_POSITION     1
#define   BODY_DETECT_KNEAD_MIN         2
#define   BODY_DETECT_KNEAD_MAX         3
#define   BODY_DETECT_3D0               4
#define   BODY_DETECT_3D1               5
#define   BODY_DETECT_3D2               6
#define   BODY_DETECT_3D3               7
#define   BODY_DETECT_3D4               8
#define   BODY_DETECT_DATA_REFRESH      9
#define   BODY_DETECT_ADJ               10
#define   BODY_DETECT_OVER              11

#define   DETECT_NO_START           0
#define   DETECT_SHOULDER           1
#define   DETECT_3D                 2
#define   DETECT_FINISH             3
#define   DETECT_INITIAL            4 
#define   SHOULDER_DETECT_FINISH    5*/
enum
{
  MASSAGE_POWER_ON_POSITION=0,
   MASSAGE_RESET_POSITION=1,    // 按摩椅复位位置，前滑电动缸收回，靠背电动缸在最高，小腿电动缸在最低，也是关机后的位置
   MASSAGE_INIT_POSITION=2,     // 按摩椅初始位置，前滑电动缸在最前，靠背电动缸在最高，小腿电动缸在最低
   MASSAGE_OPTIMAL_POSITION=3, //前滑电动缸在最前，靠背电动缸和小腿电动缸在自动程序初始位置
   MASSAGE_OPTIMAL2_POSITION, //前滑电动缸在最前，靠背电动缸和小腿电动缸在自动程序初始位置之后
   MASSAGE_OPTIMAL_MED_POSITION, //
   MASSAGE_UNKNOW_POSITION, //
   MASSAGE_MAX_POSITION,      //按摩椅最平位置 前滑电动缸在最前，靠背电动缸在最低，小腿电动缸在最高
   MASSAGE_ANY_POSITION,  
   MASSAGE_QUICK_POSITION,
   MASSAGE_MEMORY_POSITION  
     
};

void Main_Initial_IO(void);
void Main_Idle(void);
void Main_Settle(void);
void Main_WaitCommand(void);
void Main_Work(void);
void Main_Problem(void);

void Main_Update(void);
//void Main_Start_DIY_Manual(void);
void DIY_walkRefreshen(void);

void focredFinisih(void);



void Wrick_Mode_Massage_Pointer_Control_Start(void);

void Wrick_Mode_Massage_Pointer_Control_Proc(void);
void HeatProgram(void);
void HeatProgram_temperatue(void);


void Main_Settle_1ST(void);
void main_30ms_int(void);


void main_100ms_int(void);

#endif
