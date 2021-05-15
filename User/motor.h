#ifndef __MOTOR_H
#define __MOTOR_H

#include "userlib.h"
#include "rc.h"
#include "ahrs.h"
#include "flight_mode.h"
#include "helicopter.h"
#include "pid.h"

#define PWM_OUT_NUM 8
#define INI_PWM 1500
#define Min_PWM_Out  800 //800    //us
#define Max_PWM_Out  2500//2200   //us

#define RAD_TO_PWM 603.744f

typedef struct
{
	TIM_HandleTypeDef* htimA;
	TIM_HandleTypeDef* htimB;
	char *name;
	s16 PwmOff[PWM_OUT_NUM];
}sMOT_CFG;

typedef struct
{
	TIM_HandleTypeDef* htimA;
	TIM_HandleTypeDef* htimB;
	char *name;
	
	bool  Update;     //����  --
    eSTA  Sta;        //״̬  --
    eERR  Err;        //������Ϣ  --
	bool  UnLock;     //����
	float  rotate_speed;    //��ǰת��
	float  last_rotate_speed;    //��ʱ�̵�׼ת��
	float rotate_raw;
	int  pwm_throttle_base;    //�л�ʱ���������Ļ���
	int auto_throttle_width_increase;   //���������
	int set_speed;     //�趨������ת��
	int gear_ratio;   //���ֱ�
  int lim_speed;  //�޶��ٶ�
	
	float   pwm_temp;   //���pwm
	u16 PWM[PWM_OUT_NUM];
	u16 PWM_OBS[PWM_OUT_NUM];
	s16 PwmOff[PWM_OUT_NUM];
}sMOT;

#define MOT_NUM 1
extern int SPEED;
extern sMOT mot[MOT_NUM];

void Mot_Init_Para(sMOT *ele,sMOT_CFG *elecfg);
bool Mot_Init(sMOT *ele);
void Mot_Ctr(sMOT *ele);
float rotate_get(sMOT *ele1,sEXIT *ele2 );
#endif
