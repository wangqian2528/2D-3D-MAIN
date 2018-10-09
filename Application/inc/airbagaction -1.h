//气囊常数定义
#include "valve.h"
#define ARM_AIRBAG_EXIST	0

//用于计算部位组合
#define LEG_FOOT_AIRBAG_MASK	      0x01
#define SEAT_AIRBAG_MASK		      0x02
#define ARM_AIRBAG_MASK			      0x04
#define AUTO_AIRBAG_MASK		      0x08
/*
#define AIRBAG_LOCATE_NONE			  0x00
#define AIRBAG_LOCATE_LEG_FOOT		  0x01
#define AIRBAG_LOCATE_BODY_UP		  0x02    
#define AIRBAG_LOCATE_ARM			  0x04
#define AIRBAG_LOCATE_AUTO			  0x08
*/
//气囊强度定义
#define AIRBAG_STRENGTH_0	0
#define AIRBAG_STRENGTH_1	1
#define AIRBAG_STRENGTH_2	2
#define AIRBAG_STRENGTH_3	3
#define AIRBAG_STRENGTH_4	4
#define AIRBAG_STRENGTH_5	5
/*
//为了显示方便,对气囊做简易编号
#define F1D		FOOT1_DIS		//脚根
#define F1C		FOOT1_CHR
#define FHEEL   FOOT1_CHR
#define F0D		FOOT0_DIS		//脚底 8301改为足左侧
#define F0C		FOOT0_CHR
#define FLeftSide FOOT0_CHR
#define F2D		FOOT2_DIS		//脚侧 8301改为足右侧
#define F2C		FOOT2_CHR
#define FRightSide FOOT2_CHR
#define L0D		LEG0_DIS		//小腿上底
#define L0C		LEG0_CHR
#define LUpBottom LEG0_CHR
#define L1D		LEG1_DIS		//小腿上侧
#define L1C		LEG1_CHR
#define LUpSide LEG1_CHR
#define L2D		LEG2_DIS		//小腿下底
#define L2C		LEG2_CHR
#define LDownBottom LEG2_CHR
#define L3D		LEG3_DIS		//小腿下侧
#define L3C		LEG3_CHR
#define LDownSide LEG3_CHR

#define BACK_LEG_STRETCH_TIME 120 //复位之间
#define STRETCH_STEP1 {PE_BODY|LEFT_THIGH_CHR|RIGHT_THIGH_CHR|STRETCH_UP,200,200,200}//夹紧臀腰小腿上升靠背后躺,直到小腿上升到最高点或15秒
#define STRETCH_STEP2 {PE_LEG|FLeftSide|FRightSide|LUpSide|LDownSide|LEFT_THIGH_CHR|RIGHT_THIGH_CHR|STRETCH_STOP|ROLLER_SLOW_CON,50,70,90} //小腿充气，小腿好靠背停止
#define STRETCH_STEP3 {FLeftSide|FRightSide|LUpSide|LDownSide|STRETCH_DOWN|ROLLER_SLOW_CON,100,100,100}//拉腿，靠背停止，小腿和足部气囊充气
#define STRETCH_STEP4 {PE_LEG|FLeftSide|FRightSide|LUpSide|LDownSide|FHEEL|LUpBottom|LDownBottom|STRETCH_STOP|ROLLER_FAST_INT,30,30,30}//拉腿和靠背停止3秒钟，小腿和足部气囊充气
#define STRETCH_STEP5 {ALL_DIS|STRETCH_STOP,10,10,10}//停止两秒钟全部放气
#define STRETCH_STEP6 {STRETCH_RESET|ROLLER_SLOW_INT,120,120,120}//靠背复位
*/

/*
#define ROLLER_INTERMITTENT       0x04000000   //间歇旋转
#define ROLLER_SEMI_CIRCLE        0x08000000   //旋转半圈
#define ROLLER_CONTINUOUS         0x0c000000   //连续旋转
#define ROLLER_STOP	      0x00000000//0b00000000
#define ROLLER_SLOW_INT	  0x05000000//0b00000101
#define ROLLER_SLOW_SEM	  0x09000000//0b00001001
#define ROLLER_SLOW_CON	  0x0d000000//0b00001101
#define ROLLER_MID_INT	  0x06000000//0b00000110
#define ROLLER_MID_SEM	  0x0a000000//0b00001010
#define ROLLER_MID_CON	  0x0e000000//0b00001110
#define ROLLER_FAST_INT	  0x07000000//0b00000111
#define ROLLER_FAST_SEM	  0x0b000000//0b00001011
#define ROLLER_FAST_CON	  0x0f000000//0b00001111

#define ROLLER_PHASE	  0x10000000//0b00010000
*/
                           
const struct  AirBagStruct AirBagModeArmSholderBackWaist[] =
{
  
  //顺手推
  {ALL_DIS,20,20,20},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,30,50,60},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,15,30,35},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,15,25,35},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_1_CHR,15,25,35},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,15,25,35},
  {ALL_DIS,10,10,10},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,30,40,60},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,15,25,35},
  {PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,15,25,35},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_1_CHR,15,25,35},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,15,25,35},
  
  //肩膀推1
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,40,80,100},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,30,40,60},
  {ALL_DIS,6,12,18},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,15,20,30},
  {ALL_DIS,6,12,18},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,15,20,30},
  {ALL_DIS,6,12,18},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,15,20,30},
  
  
  //顺手敲
  {PE_ARM|LEFT_ARM_3_CHR,30,50,60},
  
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,8,15,22},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,8,15,22},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,8,15,22},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,8,15,22},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,8,15,22},
  
  
  {PE_ARM|LEFT_ARM_1_CHR,30,50,60},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,15,22},
  {PE_ARM|LEFT_ARM_1_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,15,22},
  {PE_ARM|LEFT_ARM_1_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,15,22},
  {PE_ARM|LEFT_ARM_1_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,15,22},
  {PE_ARM|LEFT_ARM_1_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,15,22},
 ////////////////////////////////////////////////////////////no 
 /* {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,15,22},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,15,22},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,15,22},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,15,22},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,15,22},
  */
  
  
  
  
  //肩膀推1
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,40,80,100},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,30,40,60},
  {ALL_DIS,6,12,18},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,15,20,30},
  {ALL_DIS,6,12,18},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,15,20,30},
  {ALL_DIS,6,12,18},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,15,20,30},
//左右摇摆
  {PE_ARM|R_U_WAIST|R_D_WAIST,30,45,60},
  {PE_ARM|L_U_WAIST|L_D_WAIST,30,45,60},
  {PE_ARM|R_U_WAIST|R_D_WAIST,30,45,60},
  {PE_ARM|L_U_WAIST|L_D_WAIST,30,45,60},
  {PE_ARM|R_U_WAIST|R_D_WAIST,30,45,60},
  {PE_ARM|L_U_WAIST|L_D_WAIST,30,45,60},
  
  {ALL_DIS,5,7,9},
  {PE_ARM|RIGHT_ARM_3_CHR,30,40,60},
  
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,8,10,12},
  
  {PE_ARM|RIGHT_ARM_1_CHR,30,40,60},
  
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_1_CHR|RIGHT_ARM_2_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_1_CHR|RIGHT_ARM_2_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_1_CHR|RIGHT_ARM_2_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_1_CHR|RIGHT_ARM_2_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_1_CHR|RIGHT_ARM_2_CHR,8,10,12},
  //肩膀推1

  //肩膀推2 
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,50,80,100},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,30,40,60},
  {ALL_DIS,3,4,5},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,6,8,10},
  {ALL_DIS,3,4,5},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,6,8,10},
  {ALL_DIS,3,4,5},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,6,8,10},
  {ALL_DIS,3,4,5},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,6,8,10},
  {ALL_DIS,3,4,5},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,6,8,10},
  {ALL_DIS,3,4,5},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,6,8,10},
  
  //左右摇摆
  {PE_ARM|R_U_WAIST|R_D_WAIST,30,45,60},
  {PE_ARM|L_U_WAIST|L_D_WAIST,30,45,60},
  {PE_ARM|R_U_WAIST|R_D_WAIST,30,45,60},
  {PE_ARM|L_U_WAIST|L_D_WAIST,30,45,60},
  {PE_ARM|R_U_WAIST|R_D_WAIST,30,45,60},
  {PE_ARM|L_U_WAIST|L_D_WAIST,30,45,60},
  
  //肩膀推3
  {ALL_DIS,20,15,10},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,50,80,100},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,30,40,60},
  {PE_ARM|RIGHT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|LEFT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|RIGHT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|LEFT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|RIGHT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|LEFT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|RIGHT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|LEFT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|RIGHT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|LEFT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {ALL_DIS,10,12,15},
 };
















const struct  AirBagStruct AirBagModeArmSholder[] =
{
  
  //顺手推
  {ALL_DIS,10,10,10},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,30,40,60},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,15,25,35},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,15,25,35},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_1_CHR,15,25,35},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,15,25,35},
  {ALL_DIS,10,10,10},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,30,40,60},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,15,25,35},
  {PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,15,25,35},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_1_CHR,15,25,35},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,15,25,35},
  
  //肩膀推1
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,50,80,100},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,30,40,60},
  {ALL_DIS,6,12,18},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,15,20,30},
  {ALL_DIS,6,12,18},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,15,20,30},
  {ALL_DIS,6,12,18},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,15,20,30},
  
  
  //顺手敲
  {PE_ARM|LEFT_ARM_3_CHR,30,40,60},
  
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,8,10,12},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,8,10,12},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,8,10,12},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,8,10,12},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,8,10,12},

  {PE_ARM|LEFT_ARM_1_CHR,30,40,60},
  
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_1_CHR|LEFT_ARM_2_CHR,8,10,12},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_1_CHR|LEFT_ARM_2_CHR,8,10,12},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_1_CHR|LEFT_ARM_2_CHR,8,10,12},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_1_CHR|LEFT_ARM_2_CHR,8,10,12},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_1_CHR|LEFT_ARM_2_CHR,8,10,12},
  
//  {PE_ARM|LEFT_ARM_1_CHR,2,3,4},
//  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,4,5,6},
//  {PE_ARM|LEFT_ARM_1_CHR,2,3,4},
//  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,4,5,6},
//  {PE_ARM|LEFT_ARM_1_CHR,2,3,4},
//  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,4,5,6},
//  {PE_ARM|LEFT_ARM_1_CHR,2,3,4},
//  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,4,5,6},
/*  
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,10,12},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,10,12},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,10,12},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,10,12},
  {PE_ARM|LEFT_ARM_2_CHR,4,6,8},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,8,10,12},
  */
//  {PE_ARM|LEFT_ARM_2_CHR,2,3,4},
//  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,4,5,6},
//  {PE_ARM|LEFT_ARM_2_CHR,2,3,4},
//  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,4,5,6},
//  {PE_ARM|LEFT_ARM_2_CHR,2,3,4},
//  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,4,5,6},
//  {PE_ARM|LEFT_ARM_2_CHR,2,3,4},
//  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,4,5,6},
//  {PE_ARM|LEFT_ARM_2_CHR,2,3,4},
//  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,4,5,6},
  //肩膀推1
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,50,80,100},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,30,40,60},
  {ALL_DIS,6,12,18},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,15,20,30},
  {ALL_DIS,6,12,18},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,15,20,30},
  {ALL_DIS,6,12,18},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,15,20,30},  
  
  {ALL_DIS,5,7,9},
  {PE_ARM|RIGHT_ARM_3_CHR,30,40,60},
  
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,8,10,12},
  
  //{PE_ARM|RIGHT_ARM_2_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,4,5,6},
  //{PE_ARM|RIGHT_ARM_2_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,4,5,6},
  //{PE_ARM|RIGHT_ARM_2_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,4,5,6},
  //{PE_ARM|RIGHT_ARM_2_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,4,5,6},
  //{PE_ARM|RIGHT_ARM_2_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,4,5,6},
  
  {PE_ARM|RIGHT_ARM_1_CHR,34,36,38},
  {PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_1_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_1_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_1_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_1_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,8,10,12},
  
  //{PE_ARM|RIGHT_ARM_1_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,4,5,6},
  //{PE_ARM|RIGHT_ARM_1_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,4,5,6},
  //{PE_ARM|RIGHT_ARM_1_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,4,5,6},
  //{PE_ARM|RIGHT_ARM_1_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,4,5,6},
  //{PE_ARM|RIGHT_ARM_1_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,4,5,6},
/*
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,8,10,12},
  {PE_ARM|RIGHT_ARM_2_CHR,4,6,8},
  {PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,8,10,12},
*/
  //{PE_ARM|RIGHT_ARM_2_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,4,5,6},
  //{PE_ARM|RIGHT_ARM_2_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,4,5,6},
  //{PE_ARM|RIGHT_ARM_2_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,4,5,6},
  //{PE_ARM|RIGHT_ARM_2_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,4,5,6},
  //{PE_ARM|RIGHT_ARM_2_CHR,2,3,4},
  //{PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,4,5,6},
  
  
  
  //肩膀推2 
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,50,80,100},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,30,40,60},
  {ALL_DIS,3,4,5},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,6,8,10},
  {ALL_DIS,3,4,5},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,6,8,10},
  {ALL_DIS,3,4,5},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,6,8,10},
  {ALL_DIS,3,4,5},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,6,8,10},
  {ALL_DIS,3,4,5},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,6,8,10},
  {ALL_DIS,3,4,5},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,6,8,10},
  
  //肩膀推3
  {ALL_DIS,20,15,10},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,50,80,100},
  {PE_ARM|LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,30,40,60},
  {PE_ARM|RIGHT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|LEFT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|RIGHT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|LEFT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|RIGHT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|LEFT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|RIGHT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|LEFT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|RIGHT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {PE_ARM|LEFT_SHOULDER_CHR,5,6,7},
  {LEFT_SHOULDER_CHR|RIGHT_SHOULDER_CHR,1,2,3},
  {ALL_DIS,10,12,15},
 
};
//此程序只用在拉退程序中
const struct  AirBagStruct AirBagModeArm[] =
{
  //顺手推
  {ALL_DIS,5,6,7},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,30,40,50},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,20,30,40},
  {PE_ARM|LEFT_ARM_2_CHR|LEFT_ARM_1_CHR,20,30,40},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_1_CHR,20,30,40},
  {PE_ARM|LEFT_ARM_3_CHR|LEFT_ARM_2_CHR,20,30,40},
  {ALL_DIS,5,6,7},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,30,40,50},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,20,30,40},
  {PE_ARM|RIGHT_ARM_2_CHR|RIGHT_ARM_1_CHR,20,30,40},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_1_CHR,20,30,40},
  {PE_ARM|RIGHT_ARM_3_CHR|RIGHT_ARM_2_CHR,20,30,40},
  
{ALL_DIS,10,12,15},
} ;

const struct AirBagStruct AirBagModeBackWaist[] =
{
  {ALL_DIS,20,20,20},
  {PE1|R_U_WAIST|R_D_WAIST|L_U_WAIST|L_D_WAIST,30,45,60},
  //左右摇摆
  {PE1|R_U_WAIST|R_D_WAIST,30,45,60},
  {PE1|L_U_WAIST|L_D_WAIST,30,45,60},
  {PE1|R_U_WAIST|R_D_WAIST,30,45,60},
  {PE1|L_U_WAIST|L_D_WAIST,30,45,60},
  {PE1|R_U_WAIST|R_D_WAIST,30,45,60},
  {PE1|L_U_WAIST|L_D_WAIST,30,45,60},
  {PE1|R_U_WAIST|R_D_WAIST,30,45,60},
  {PE1|L_U_WAIST|L_D_WAIST,30,45,60},
  {PE1|R_U_WAIST|R_D_WAIST,30,45,60},
  {PE1|L_U_WAIST|L_D_WAIST,30,45,60},
  //左侧放开,右侧上下敲击
  {PE1|R_U_WAIST,20,25,30},
  {PE1|R_D_WAIST,20,25,30},
  {PE1|R_U_WAIST,20,25,30},
  {PE1|R_D_WAIST,20,25,30},
  {PE1|R_U_WAIST,20,25,30},
  {PE1|R_D_WAIST,20,25,30},
  {PE1|R_U_WAIST,20,25,30},
  {PE1|R_D_WAIST,20,25,30},
  //右侧放开,左侧上下敲击
  {PE1|L_U_WAIST,20,25,30},
  {PE1|L_D_WAIST,20,25,30},
  {PE1|L_U_WAIST,20,25,30},
  {PE1|L_D_WAIST,20,25,30},
  {PE1|L_U_WAIST,20,25,30},
  {PE1|L_D_WAIST,20,25,30},
  {PE1|L_U_WAIST,20,25,30},
  {PE1|L_D_WAIST,20,25,30}
};
const struct AirBagStruct AirBagModeSeat[] =
{
  {ALL_DIS,20,20,20},
  {PE2|R_THIGH|L_THIGH,30,50,70},
  {ALL_DIS,30,30,30},
  {PE2|R_THIGH|L_THIGH,30,50,70},
  {ALL_DIS,30,30,30},
  {PE2|R_THIGH|L_THIGH,30,50,70},
  {ALL_DIS,30,30,30},
  {PE2|R_THIGH|L_THIGH,30,50,70},
  {ALL_DIS,30,30,30},
  
  {PE2|L_THIGH,30,50,70},
  {PE2|R_THIGH,30,50,70},
  {PE2|L_THIGH,30,50,70},
  {PE2|R_THIGH,30,50,70},
  {PE2|L_THIGH,30,50,70},
  {PE2|R_THIGH,30,50,70},
  {PE2|L_THIGH,30,50,70},
  {PE2|R_THIGH,30,50,70},
  {PE2|L_THIGH,30,50,70},
  {PE2|R_THIGH,30,50,70},
 
} ;

const struct AirBagStruct AirBagModeLegFootSeat[] =
{
  {ALL_DIS,20,20,20},
 //揉腿向里转  F_HEEL
  //A
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,40,60,80},//准备
  
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,10,30,50},//准备
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,30,30,30},//准备
  
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_OUT,50,70,100},
  {ALL_DIS,20,15,10},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_OUT,20,40,70},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_OUT,30,30,30}, 
  {ALL_DIS,20,15,10},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_OUT,50,70,100},
  {ALL_DIS,20,15,10},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_OUT,20,40,70},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_OUT,30,30,30},
  {ALL_DIS,20,15,10},
 //脚左右摆动
  {PE2|F_R_SIDE|ROLLER_MID,20,35,50},
  {PE2|F_L_SIDE|ROLLER_FAST,20,35,50},
  {PE2|F_R_SIDE|ROLLER_MID,20,35,50},
  {PE2|F_L_SIDE|ROLLER_FAST,20,35,50},
  {PE2|F_R_SIDE|ROLLER_MID,20,35,50},
  {PE2|F_L_SIDE|ROLLER_FAST,20,35,50},
//  {ALL_DIS,20,20,20},
// 揉腿向外转  |F_HEEL
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,40,60,80},//准备
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID|KNEAD_MID_OUT,10,20,30},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_OUT,30,30,30},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID|KNEAD_MID_OUT,20,30,40},
  
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID,10,20,30},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_OUT,30,30,30},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID|KNEAD_MID_OUT,20,30,40},
  
  {ALL_DIS,7,7,7},
// 揉腿向里转    
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,40,60,80},//准备
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID|KNEAD_MID_IN,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_IN,20,20,20}, 
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_IN,30,30,30},
  
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID,10,20,30},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_IN,30,30,30},
  
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID,10,20,30},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_IN,30,30,30},
  
  {ALL_DIS,7,7,7},
 
//脚左右摆动  
  {PE2|F_L_SIDE|L_THIGH|ROLLER_SLOW,20,35,50},
  {PE2|F_R_SIDE|R_THIGH|ROLLER_MID,20,35,50},
  {PE2|F_L_SIDE|L_THIGH|ROLLER_FAST,20,35,50},
  {PE2|F_R_SIDE|R_THIGH|ROLLER_FAST,20,35,50},
  {PE2|F_L_SIDE|L_THIGH|ROLLER_MID,20,35,50},
  {PE2|F_R_SIDE|R_THIGH|ROLLER_SLOW,20,35,50},
  //{ALL_DIS,20,20,20}, 
  
  {PE2|R_THIGH|L_THIGH,50,60,70},
  {PE2|R_THIGH,50,60,70},
  {PE2|L_THIGH,50,60,70},
   
//  揉腿摇摆
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,40,60,80},//准备
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_SWAY,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_SWAY,20,20,20},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_SWAY,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_SWAY,30,30,30},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_SWAY,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_SWAY,30,30,30},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_SWAY,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_SWAY,30,30,30},
  {ALL_DIS,7,7,7},
  
 //捏脚后跟
  {PE2|F_L_SIDE|F_R_SIDE|ROLLER_FAST|ROLLER_S_RUB,20,30,40},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_FAST|ROLLER_S_RUB,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE|ROLLER_FAST|ROLLER_S_RUB,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_FAST|ROLLER_S_RUB,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE|ROLLER_FAST|ROLLER_S_RUB,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_FAST|ROLLER_S_RUB,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE|ROLLER_FAST|ROLLER_S_RUB,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL,20,30,40},
  {ALL_DIS,30,30,30},
  
  //左右屁股摇摆
  {PE2|R_THIGH|L_THIGH,50,60,70},
  {PE2|R_THIGH,50,60,70},
  {PE2|L_THIGH,50,60,70},
  {PE2|R_THIGH,50,60,70},
  {PE2|L_THIGH,50,60,70},
  {ALL_DIS,20,20,20},
  //B
  
  
  
  
};




const struct AirBagStruct AirBagModeLegFoot[] =
{
  //{ALL_DIS,20,20,20},
 /*
  {PE2|LEG_LEFT|LEG_RIGHT,70,70,70},
  {PE2,70,70,70},
  {PE2|LEG_LEFT|LEG_RIGHT,70,70,70},
  {PE2,70,70,70},
  {PE2|LEG_LEFT|LEG_RIGHT,70,70,70},
  {PE2,70,70,70},
  {PE2|LEG_LEFT|LEG_RIGHT,70,70,70},
  {PE2,70,70,70},
  {PE2|LEG_LEFT|LEG_RIGHT,70,70,70},
  {PE2,70,70,70},
  */
  
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL,50,70,90},
  
//左右推
  {PE2|F_L_SIDE,20,35,50},
  {PE2|F_R_SIDE,20,35,50},
  {PE2|F_L_SIDE,20,35,50},
  {PE2|F_R_SIDE,20,35,50},
  {PE2|F_L_SIDE,20,35,50},
  {PE2|F_R_SIDE,20,35,50},
  {PE2|F_L_SIDE,20,35,50},
  {PE2|F_R_SIDE,20,35,50},
  {PE2|F_L_SIDE,20,35,50},
  {PE2|F_R_SIDE,20,35,50},
     
// 揉腿向外转  
  {PE2|LEG_LEFT|LEG_RIGHT,30,30,30},
  {PE2|LEG_LEFT|LEG_RIGHT|KNEAD_MID_OUT,40,50,60},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|KNEAD_MID_OUT,40,50,60},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|KNEAD_MID_OUT,40,50,60},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|KNEAD_MID_OUT,40,50,60},
  {ALL_DIS,7,7,7},
// 揉腿向里转  
  {PE2|LEG_LEFT|LEG_RIGHT|KNEAD_MID_IN,40,50,60},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|KNEAD_MID_IN,40,50,60},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|KNEAD_MID_IN,40,50,60},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|KNEAD_MID_IN,40,50,60},
  {ALL_DIS,7,7,7},

//夹住脚 后面顶
  {PE2|F_L_SIDE|F_R_SIDE,20,30,40},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL,20,35,45},
  {PE2|F_L_SIDE|F_R_SIDE,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE,20,30,40},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL,20,35,45},
  {PE2|F_L_SIDE|F_R_SIDE,5,5,5},
  
  //  揉腿摇摆
  {PE2|LEG_LEFT|LEG_RIGHT|KNEAD_FAST_SWAY,40,50,60},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|KNEAD_FAST_SWAY,40,50,60},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|KNEAD_FAST_SWAY,40,50,60},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|KNEAD_FAST_SWAY,40,50,60},
  //{KNEAD_FAST_SWAY,10,10,10},
  {ALL_DIS,20,20,20},
  

  /*
  {ALL_DIS,20,20,20},
  {PE2|PE1|F_L_SIDE|F_R_SIDE,30,30,30},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON,3,3,3},
  
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON,3,3,3},
  
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_MID_CON,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON|ROLLER_PHASE,3,3,3},
  {PE2|PE1|F_L_SIDE|F_R_SIDE|ROLLER_MID_CON,3,3,3},
 */
} ;


//=====================================================================


//GROWTH AIR
const struct AirBagStruct AirBagModeLegFootSeat_Growth[] =
{
 // {ALL_DIS,20,20,20},
 //揉腿向里转  F_HEEL
 //A 
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,40,60,80},//准备
  
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,10,30,50},//准备
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,30,30,30},//准备
  
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_OUT,50,70,100},
  {ALL_DIS,20,15,10},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_OUT,20,40,70},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_OUT,30,30,30}, 
  {ALL_DIS,20,15,10},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_OUT,50,70,100},
  {ALL_DIS,20,15,10},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_OUT,20,40,70},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_OUT,30,30,30},
  {ALL_DIS,20,15,10},
 //脚左右摆动
  {PE2|F_R_SIDE|ROLLER_MID,20,35,50},
  {PE2|F_L_SIDE|ROLLER_FAST,20,35,50},
  {PE2|F_R_SIDE|ROLLER_MID,20,35,50},
  {PE2|F_L_SIDE|ROLLER_FAST,20,35,50},
  {PE2|F_R_SIDE|ROLLER_MID,20,35,50},
  {PE2|F_L_SIDE|ROLLER_FAST,20,35,50},
  {ALL_DIS,20,20,20},
// 揉腿向外转  |F_HEEL
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,40,60,80},//准备
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID|KNEAD_MID_OUT,10,20,30},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_OUT,30,30,30},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID|KNEAD_MID_OUT,20,30,40},
  
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID,10,20,30},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_OUT,30,30,30},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID|KNEAD_MID_OUT,20,30,40},
  
  {ALL_DIS,7,7,7},
// 揉腿向里转    
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,40,60,80},//准备
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID|KNEAD_MID_IN,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_IN,20,20,20}, 
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_IN,30,30,30},
  
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID,10,20,30},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_IN,30,30,30},
  
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID,10,20,30},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_IN,30,30,30},
  
  {ALL_DIS,7,7,7},
 
//脚左右摆动  
  {PE2|F_L_SIDE|L_THIGH|ROLLER_SLOW,20,35,50},
  {PE2|F_R_SIDE|R_THIGH|ROLLER_MID,20,35,50},
  {PE2|F_L_SIDE|L_THIGH|ROLLER_FAST,20,35,50},
  {PE2|F_R_SIDE|R_THIGH|ROLLER_FAST,20,35,50},
  {PE2|F_L_SIDE|L_THIGH|ROLLER_MID,20,35,50},
  {PE2|F_R_SIDE|R_THIGH|ROLLER_SLOW,20,35,50},
  //{ALL_DIS,20,20,20}, 
  
  {PE2|R_THIGH|L_THIGH,50,60,70},
  {PE2|R_THIGH,50,60,70},
  {PE2|L_THIGH,50,60,70},
   
//  揉腿摇摆
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,40,60,80},//准备
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_SWAY,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_SWAY,20,20,20},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_SWAY,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_SWAY,30,30,30},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_SWAY,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_SWAY,30,30,30},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_SWAY,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_SWAY,30,30,30},
  {ALL_DIS,7,7,7},
  
 //捏脚后跟
  {PE2|F_L_SIDE|F_R_SIDE|ROLLER_FAST|ROLLER_S_RUB,20,30,40},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_FAST|ROLLER_S_RUB,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE|ROLLER_FAST|ROLLER_S_RUB,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_FAST|ROLLER_S_RUB,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE|ROLLER_FAST|ROLLER_S_RUB,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_FAST|ROLLER_S_RUB,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE|ROLLER_FAST|ROLLER_S_RUB,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL,20,30,40},
//  {ALL_DIS,30,30,30},
  
  //左右屁股摇摆
  {PE2|R_THIGH|L_THIGH,50,60,70},
  {PE2|R_THIGH,50,60,70},
  {PE2|L_THIGH,50,60,70},
  {PE2|R_THIGH,50,60,70},
  {PE2|L_THIGH,50,60,70},
  {ALL_DIS,20,20,20},
  
  //B
  
   {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,40,60,80},//准备
  
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,10,30,50},//准备
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,30,30,30},//准备
  
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_OUT,50,70,100},
  {ALL_DIS,20,15,10},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_OUT,20,40,70},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_OUT,30,30,30}, 
  {ALL_DIS,20,15,10},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_OUT,50,70,100},
  {ALL_DIS,20,15,10},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_OUT,20,40,70},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_OUT,30,30,30},
  {ALL_DIS,20,15,10},
 //脚左右摆动
  {PE2|F_R_SIDE|ROLLER_MID,20,35,50},
  {PE2|F_L_SIDE|ROLLER_FAST,20,35,50},
  {PE2|F_R_SIDE|ROLLER_MID,20,35,50},
  {PE2|F_L_SIDE|ROLLER_FAST,20,35,50},
  {PE2|F_R_SIDE|ROLLER_MID,20,35,50},
  {PE2|F_L_SIDE|ROLLER_FAST,20,35,50},
  {ALL_DIS,20,20,20},
// 揉腿向外转  |F_HEEL
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,40,60,80},//准备
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID|KNEAD_MID_OUT,10,20,30},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_OUT,30,30,30},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID|KNEAD_MID_OUT,20,30,40},
  
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID,10,20,30},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_OUT,30,30,30},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW,20,20,20},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID|KNEAD_MID_OUT,20,30,40},
  
  {ALL_DIS,7,7,7},
// 揉腿向里转    
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,40,60,80},//准备
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID|KNEAD_MID_IN,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_IN,20,20,20}, 
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_IN,30,30,30},
  
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID,10,20,30},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_IN,30,30,30},
  
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_MID,10,20,30},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_MID|KNEAD_MID_IN,30,30,30},
  
  {ALL_DIS,7,7,7},
 
//脚左右摆动  
  {PE2|F_L_SIDE|L_THIGH|ROLLER_SLOW,20,35,50},
  {PE2|F_R_SIDE|R_THIGH|ROLLER_MID,20,35,50},
  {PE2|F_L_SIDE|L_THIGH|ROLLER_FAST,20,35,50},
//  {PE2|F_R_SIDE|R_THIGH|ROLLER_FAST,20,35,50},
//  {PE2|F_L_SIDE|L_THIGH|ROLLER_MID,20,35,50},
//  {PE2|F_R_SIDE|R_THIGH|ROLLER_SLOW,20,35,50},
  //{ALL_DIS,20,20,20}, 
  
  {PE2|R_THIGH|L_THIGH,50,60,70},
  {PE2|R_THIGH,50,60,70},
  {PE2|L_THIGH,50,60,70},
   
//  揉腿摇摆
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE,40,60,80},//准备
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_SWAY,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_SWAY,20,20,20},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_SWAY,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_SWAY,30,30,30},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_SWAY,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_SWAY,30,30,30},
  {ALL_DIS,7,7,7},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|ROLLER_SLOW|KNEAD_FAST_SWAY,20,30,40},
  {PE2|LEG_LEFT|LEG_RIGHT|F_R_SIDE|F_L_SIDE|F_HEEL|ROLLER_SLOW|KNEAD_FAST_SWAY,30,30,30},
  {ALL_DIS,7,7,7},
  
 //捏脚后跟
  {PE2|F_L_SIDE|F_R_SIDE|ROLLER_FAST|ROLLER_S_RUB,20,30,40},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_FAST|ROLLER_S_RUB,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE|ROLLER_FAST|ROLLER_S_RUB,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_FAST|ROLLER_S_RUB,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE|ROLLER_FAST|ROLLER_S_RUB,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL|ROLLER_FAST|ROLLER_S_RUB,6,10,14},
  {PE2|F_L_SIDE|F_R_SIDE|ROLLER_FAST|ROLLER_S_RUB,5,5,5},
  {PE2|F_L_SIDE|F_R_SIDE|F_HEEL,20,30,40},
//  {ALL_DIS,30,30,30},
  
  //左右屁股摇摆
  {PE2|R_THIGH|L_THIGH,50,60,70},
  {PE2|R_THIGH,50,60,70},
  {PE2|L_THIGH,50,60,70},
  {PE2|R_THIGH,50,60,70},
  {PE2|L_THIGH,50,60,70},
  {ALL_DIS,20,20,20}, 

} ;


//const struct AirBagStruct AirBagModeLegFoot_GrowthA[] =
//{
 

 
//} ;



//AirBagModeLegFoot_GrowthA[] =
//const struct AirBagStruct AirBagModeLegFoot_GrowthB[] =
//{


//} ;

