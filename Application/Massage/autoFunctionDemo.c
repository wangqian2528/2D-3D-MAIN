

const WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO autoFunctionDemo[] = 
{
     // 1 ����ģʽ��2 ��о��λ 3 �г̣�2�ж�λʱ�г������壩
     //4 ����   5  ����Ȧ��  6 �û�  7 �û�ʱ�� 8 �û�ֹͣʱ��
    // 9 �ٶȣ��û����������ٶȣ� 10 3D���   11 3D λ�� 12 3D����ʱ��  13 3D ֹͣʱ��
    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
    
    {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_STOP_AT_MIN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
    
    
    {BACK_SUB_MODE_PRESS,SHOULDER_ADD_PULSE,40,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,20},
//    
//    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_4,_3D_SPEED_5,20},
//    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_2,_3D_SPEED_5,20},
//    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_4,_3D_SPEED_5,20},
//    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_2,_3D_SPEED_5,0},
    
    {BACK_SUB_MODE_PRESS,SHOULDER_ADD_PULSE,0, KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_2,_3D_PARK,0,0,20},
    {BACK_SUB_MODE_PRESS,SHOULDER_ADD_PULSE,40,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,20},
    {BACK_SUB_MODE_PRESS,SHOULDER_ADD_PULSE,0, KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,20},
    
    //=========================���ϲ�=�ַ�2-ָѹ=================================================================================--42
    {BACK_SUB_MODE_PRESS,SHOULDER_ADD_PULSE,10,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_1,0},
    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
//1
//eighty
    {BACK_SUB_MODE_KNEAD,WALK_ADD_PULSE,5,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_1,0},
    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
    //29
    {BACK_SUB_MODE_KNEAD,WALK_ADD_PULSE,5,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
    {BACK_SUB_MODE_KNEAD,WALK_ADD_PULSE,5,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
//7    
    ///////////////////////////////////////////////////////////////////////
    //=========================��������-�β�-3dָѹ-ָѹ=================================--12
    {BACK_SUB_MODE_KNEAD,WALK_PWM,0,KNEAD_RUN_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_2,_3D_PARK,0,0,0},
    //////////////////////////////////////////////////test start////////////////////////////////////////////////////////////
    
    {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_1,0},
    
    
    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
    
    {BACK_SUB_MODE_PRESS,SHOULDER_ADD_PULSE ,5,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
    //12
    {BACK_SUB_MODE_PRESS,SHOULDER_ADD_PULSE,10 ,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},

    {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK  ,100,KNEAD_RUN       ,0,KNOCK_STOP ,0,0,SPEED_4,_3D_PROGRAM,_3D_4,_3D_SPEED_5,0},
    //50
    ///////A
    {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,160, KNEAD_STOP_AT_MIN, 0, KNOCK_RUN, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,6,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
//27    
    
    //�û���ȥ  ��������
    
    {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,0, KNEAD_STOP_AT_MIN, 0, KNOCK_RUN, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,6,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
//29


    { BACK_SUB_MODE_KNOCK, WALK_LOCATE_SHOULDER, 0, KNEAD_STOP_AT_MIN, 0, KNOCK_RUN, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
//30   
    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_NeckMed, 0, KNEAD_RUN, 0, KNOCK_STOP, 0, 0, SPEED_2, _3D_PROGRAM, AXIS_STRONGEST, _3D_SPEED_2, 0 },
    { BACK_SUB_MODE_KNEAD, RUN_UP_DOWN, 0, KNEAD_FITFUL, 6, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },
    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_SHOULDER, 0, KNEAD_RUN, 0, KNOCK_STOP, 0, 0, SPEED_2, _3D_PROGRAM, AXIS_MIDDLE, _3D_SPEED_3, 0 },
    { BACK_SUB_MODE_KNEAD, RUN_UP_DOWN, 0, KNEAD_FITFUL, 4, KNOCK_STOP, 0, 0, SPEED_2, _3D_PROGRAM, AXIS_STRONGEST, _3D_SPEED_2, 0 },
                        //34                  
//=========================================�羱������-�ַ�1-=========================================--3 
  //WALK_LOCATE_ABSULATE  
    {BACK_SUB_MODE_PRESS,WALK_LOCATE_3D_PRESS,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_AUTO_WALK,1,_3D_SPEED_1,0}, 
    {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PARK,50,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_5,_3D_PARK,0,0,0}, 
    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_PARK, 0, KNEAD_RUN, 4, KNOCK_STOP, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
    { BACK_SUB_MODE_KNEAD, RUN_UP_DOWN, 0, KNEAD_TOGGLE, 4, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },
    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_ABSULATE, 40, KNEAD_RUN, 0, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },
    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_PARK, 0, KNEAD_RUN, 4, KNOCK_STOP, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
    { BACK_SUB_MODE_KNEAD, RUN_UP_DOWN, 0, KNEAD_TOGGLE, 4, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },
    //{BACK_SUB_MODE_PRESS,WALK_LOCATE_3D_PRESS,120,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_AUTO_WALK,1,_3D_SPEED_1,0},
    {BACK_SUB_MODE_PRESS,WALK_LOCATE_3D_PRESS,SHOULDER_POSITION,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_AUTO_WALK,1,_3D_SPEED_1,0},
};
/***********************************************************����************************************************************************/
//const WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO autoFunctionDemo[] = 
//{
//   
//    //=========================���ϲ�=�ַ�2-ָѹ=================================================================================--42
//    {BACK_SUB_MODE_PRESS,SHOULDER_ADD_PULSE,10,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_1,0},
//    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
////1
////eighty
//    {BACK_SUB_MODE_KNEAD,WALK_ADD_PULSE,5,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_1,0},
//    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
////3   
//
//    //29
//    {BACK_SUB_MODE_KNEAD,WALK_ADD_PULSE,5,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
//    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
//    {BACK_SUB_MODE_KNEAD,WALK_ADD_PULSE,5,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
//    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
////7    
//    ///////////////////////////////////////////////////////////////////////
//    //=========================��������-�β�-3dָѹ-ָѹ=================================--12
//    {BACK_SUB_MODE_KNEAD,WALK_PWM,0,KNEAD_RUN_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_2,_3D_PARK,0,0,0},
//    //////////////////////////////////////////////////test start////////////////////////////////////////////////////////////
//    
//    {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_1,0},
//    
//    
//    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
//    
//    {BACK_SUB_MODE_PRESS,SHOULDER_ADD_PULSE ,5,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
//    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
//    //12
//    {BACK_SUB_MODE_PRESS,SHOULDER_ADD_PULSE,10 ,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
//    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
//
//  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK        ,100,KNEAD_RUN       ,0,KNOCK_STOP ,0,0,SPEED_4,_3D_PROGRAM,_3D_4,_3D_SPEED_5,0},
//    //50
//    ///////A
//    {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,160, KNEAD_STOP_AT_MIN, 0, KNOCK_RUN, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
//    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,6,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
////27    
//    
//    //�û���ȥ  ��������
//    
//    {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,0, KNEAD_STOP_AT_MIN, 0, KNOCK_RUN, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
//      {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,6,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
////29
//
//
//{ BACK_SUB_MODE_KNOCK, WALK_LOCATE_SHOULDER, 0, KNEAD_STOP_AT_MIN, 0, KNOCK_RUN, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
////30   
// 
//    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_NeckMed, 0, KNEAD_RUN, 0, KNOCK_STOP, 0, 0, SPEED_2, _3D_PROGRAM, AXIS_STRONGEST, _3D_SPEED_2, 0 },
//    { BACK_SUB_MODE_KNEAD, RUN_UP_DOWN, 0, KNEAD_FITFUL, 6, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },
//    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_SHOULDER, 0, KNEAD_RUN, 0, KNOCK_STOP, 0, 0, SPEED_2, _3D_PROGRAM, AXIS_MIDDLE, _3D_SPEED_3, 0 },
//    { BACK_SUB_MODE_KNEAD, RUN_UP_DOWN, 0, KNEAD_FITFUL, 4, KNOCK_STOP, 0, 0, SPEED_2, _3D_PROGRAM, AXIS_STRONGEST, _3D_SPEED_2, 0 },
//                        //34                  
////=========================================�羱������-�ַ�1-=========================================--3 
//  //WALK_LOCATE_ABSULATE  
//    {BACK_SUB_MODE_PRESS,WALK_LOCATE_3D_PRESS,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_AUTO_WALK,1,_3D_SPEED_1,0}, 
//    {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PARK,50,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_5,_3D_PARK,0,0,0}, 
//    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_PARK, 0, KNEAD_RUN, 4, KNOCK_STOP, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
//    { BACK_SUB_MODE_KNEAD, RUN_UP_DOWN, 0, KNEAD_TOGGLE, 4, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },
//    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_ABSULATE, 40, KNEAD_RUN, 0, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },
//    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_PARK, 0, KNEAD_RUN, 4, KNOCK_STOP, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
//    { BACK_SUB_MODE_KNEAD, RUN_UP_DOWN, 0, KNEAD_TOGGLE, 4, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },
// 
//
//
//
//    //{BACK_SUB_MODE_PRESS,WALK_LOCATE_3D_PRESS,120,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_AUTO_WALK,1,_3D_SPEED_1,0},
//    {BACK_SUB_MODE_PRESS,WALK_LOCATE_3D_PRESS,SHOULDER_POSITION,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_AUTO_WALK,1,_3D_SPEED_1,0},
//};

/***********************************************************************************************/
const WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO autoFunctionDemo5[] = 
{
   
    
    
    
    
    
    
    
    
    
    
  //WALK_LOCATE_ABSULATE  
    
    
    {BACK_SUB_MODE_PRESS,WALK_LOCATE_3D_PRESS,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_AUTO_WALK,1,_3D_SPEED_1,0}, 
    {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PARK,50,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_5,_3D_PARK,0,0,0}, 
    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_PARK, 0, KNEAD_RUN, 3, KNOCK_STOP, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
    { BACK_SUB_MODE_KNEAD, RUN_UP_DOWN, 0, KNEAD_TOGGLE, 3, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },
    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_ABSULATE, 40, KNEAD_RUN, 0, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },
    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_PARK, 0, KNEAD_RUN, 2, KNOCK_STOP, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
    { BACK_SUB_MODE_KNEAD, RUN_UP_DOWN, 0, KNEAD_TOGGLE, 2, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },
 



    {BACK_SUB_MODE_PRESS,WALK_LOCATE_3D_PRESS,120,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_AUTO_WALK,1,_3D_SPEED_1,0},
    {BACK_SUB_MODE_PRESS,WALK_LOCATE_3D_PRESS,SHOULDER_POSITION,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_AUTO_WALK,1,_3D_SPEED_1,0},
//8

                                              //=========================�����β�3dָѹ==================================================================================-26
                                              /*    {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_TO_SIDE,0,KNOCK_STOP,0,0,SPEED_3,_3D_SIDE_TOGGLE,0,_3D_SPEED_1,0},
                                                  
                                                  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_9_10,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
                                                  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_TO_SIDE,0,KNOCK_STOP,0,0,SPEED_3,_3D_SIDE_TOGGLE,0,_3D_SPEED_1,0},
                                                  //10
                                                  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_8_10,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
                                                  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_TO_SIDE,0,KNOCK_STOP,0,0,SPEED_3,_3D_SIDE_TOGGLE,0,_3D_SPEED_1,0},
                                                  
                                                  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_7_10,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
                                                  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_TO_SIDE,0,KNOCK_STOP,0,0,SPEED_3,_3D_SIDE_TOGGLE,3,_3D_SPEED_1,0},
                                              //14
                                              
                                              //=========================���²��ַ�2-ָѹ==================================================================================--33
                                                  //tweenty
                                                  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_6_10,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
                                                  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_TO_SIDE,0,KNOCK_STOP,0,0,SPEED_3,_3D_SIDE_TOGGLE,0,_3D_SPEED_1,0},
                                                  
                                                  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_STOP_AT_MIN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,_3D_SPEED_1,0},
                                                  
                                                  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_2_10,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
                                                  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_TO_SIDE,0,KNOCK_STOP,0,0,SPEED_3,_3D_SIDE_TOGGLE,0,_3D_SPEED_1,0},
                                                  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_1_10,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
                                                  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_TO_SIDE,0,KNOCK_STOP,0,0,SPEED_3,_3D_SIDE_TOGGLE,0,_3D_SPEED_1,0},
                                                  //21
                                                  {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
                                                  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_TO_SIDE,0,KNOCK_STOP,0,0,SPEED_3,_3D_SIDE_TOGGLE,0,_3D_SPEED_1,0},
                                                 */ 
    
    //=========================���ϲ�=�ַ�2-ָѹ=================================================================================--42
    {BACK_SUB_MODE_PRESS,SHOULDER_ADD_PULSE,10,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_1,0},
    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
//10
//eighty
    {BACK_SUB_MODE_KNEAD,WALK_ADD_PULSE,5,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_1,0},
    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
//12    
                                //{BACK_SUB_MODE_KNEAD,WALK_ADD_PULSE,5,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_1,0},
                                //{BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
    //29
    {BACK_SUB_MODE_KNEAD,WALK_ADD_PULSE,5,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
    {BACK_SUB_MODE_KNEAD,WALK_ADD_PULSE,5,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
    {BACK_SUB_MODE_KNEAD,STATE_RUN_LOCATE_POSITION,0,KNEAD_MED_STOP_MED,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
//16    
    ///////////////////////////////////////////////////////////////////////
    //=========================��������-�β�-3dָѹ-ָѹ=================================--12
    {BACK_SUB_MODE_KNEAD,WALK_PWM,0,KNEAD_RUN_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_2,_3D_PARK,0,0,0},
    //////////////////////////////////////////////////test start////////////////////////////////////////////////////////////
    
    {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_1,0},
    
    
    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
    
    {BACK_SUB_MODE_PRESS,SHOULDER_ADD_PULSE ,5,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
    //21
    {BACK_SUB_MODE_PRESS,SHOULDER_ADD_PULSE,10 ,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
    
    {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_1,0},
    
    {BACK_SUB_MODE_PRESS,WALK_DEC_PULSE,20,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0}, 
//25    
    //{BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,60,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PARK,0,0,0},
    
    //{BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MED,3,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_1,0},
    //50
    ///////A
    {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,160, KNEAD_STOP_AT_MIN, 0, KNOCK_RUN, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
    {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,4,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
//27    
    
                        //{BACK_SUB_MODE_PRESS,WALK_LOCATE_ABSULATE,150,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PARK,0,0,0},
                        //{BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
                        
                        //{BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUBBING,6,KNOCK_PWM,0,0,SPEED_7,_3D_PARK,0,0,0},
//52    
    
    //�û���ȥ  ��������
    
    {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,0, KNEAD_STOP_AT_MIN, 0, KNOCK_RUN, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
      {BACK_SUB_MODE_KNEAD,STOP_WALK_KNEAD,0,KNEAD_MAX_MIN,3,KNOCK_STOP,0,0,SPEED_7,_3D_KNEAD,0,0,0},
//29


{ BACK_SUB_MODE_KNOCK, WALK_LOCATE_SHOULDER, 0, KNEAD_STOP_AT_MIN, 0, KNOCK_RUN, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
//30   
 
    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_NeckMed, 0, KNEAD_RUN, 0, KNOCK_STOP, 0, 0, SPEED_2, _3D_PROGRAM, AXIS_STRONGEST, _3D_SPEED_2, 0 },
    { BACK_SUB_MODE_KNEAD, RUN_UP_DOWN, 0, KNEAD_FITFUL, 6, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },
    { BACK_SUB_MODE_KNEAD, WALK_LOCATE_SHOULDER, 0, KNEAD_RUN, 0, KNOCK_STOP, 0, 0, SPEED_2, _3D_PROGRAM, AXIS_MIDDLE, _3D_SPEED_3, 0 },
    { BACK_SUB_MODE_KNEAD, RUN_UP_DOWN, 0, KNEAD_FITFUL, 4, KNOCK_STOP, 0, 0, SPEED_2, _3D_PROGRAM, AXIS_STRONGEST, _3D_SPEED_2, 0 },
                        //34
                        
                        //=========================================�羱������-�ַ�1-=========================================--3
                           // { BACK_SUB_MODE_KNEAD, WALK_SHOULDER_WAIST_2_10, 0, KNEAD_RUN, 0, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },
                           // { BACK_SUB_MODE_KNEAD, WALK_LOCATE_PARK, 0, KNEAD_RUN, 4, KNOCK_STOP, 0, 0, SPEED_3, _3D_PARK,0, 0, 0 },
                           // { BACK_SUB_MODE_KNEAD, RUN_UP_DOWN, 0, KNEAD_TOGGLE, 3, KNOCK_STOP, 0, 0, SPEED_2, _3D_PARK,0, 0, 0 },

};