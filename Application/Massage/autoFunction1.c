
//��չ��Ħ�������Ȱ�Ħ���е�����
const WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO AutoFunction1[] = 
{	
  //�ഷͬ�����粿�����٣�
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PressNeck,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_1_10,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MED,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MED,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  
  //ָѹ����//10
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_TOP,0,KNEAD_STOP_AT_MIN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //{BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,50,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
 {BACK_SUB_MODE_PRESS,WALK_LOCATE_NeckMed,0,KNEAD_STOP_AT_MIN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
 
  
  //�������
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //�걳  
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},

  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,80,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //�������㴷��//54
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,20,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,30,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //�ȴ�
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_PARK,2,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MED,1,KNOCK_RUN_WIDTH,20,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,30,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //�ȴ�
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_PARK,2,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //���󶨵�
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,50,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //�ֲ��ഷͬ��//63
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,40,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,80,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,40,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  
} ;
/*************************************************************/