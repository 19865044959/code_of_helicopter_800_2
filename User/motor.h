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
	
	bool  Update;     //更新  --
    eSTA  Sta;        //状态  --
    eERR  Err;        //错误信息  --
	bool  UnLock;     //解锁
	float  rotate_speed;    //当前转速
	float  last_rotate_speed;    //上时刻的准转速
	float rotate_raw;
	int  pwm_throttle_base;    //切换时，控制器的基点
	int auto_throttle_width_increase;   //控制器输出
	int set_speed;     //设定主旋翼转速
	int gear_ratio;   //齿轮比
  int lim_speed;  //限定速度
	
	float   pwm_temp;   //输出pwm
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
