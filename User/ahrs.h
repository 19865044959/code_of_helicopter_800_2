#ifndef __AHRS_H
#define __AHRS_H

#include "mpu6050.h"
#include "adis16365.h"
#include "hmc5883.h"
#include "gps.h"
#include "ms5611.h"
#include "command.h"
#include "exit.h"
#include "pid.h"	//��ʱ��PI����

#define USE_UWB 
#define HARD_FILT(x,y) (0.9f*(x)+0.1f*(y))

typedef enum {FLY_HEL=0,FLY_FIX=1}eFLY;

typedef struct
{
	char *name;
	float halfDt;
	float AngOff[3];
	eFLY  mode;
}sAHRS_CFG;

typedef struct
{
	bool Update;
	float dt;
	arm_matrix_instance_f32 matA,mat33,matB;
	float LLS[3][3];
	
	char *name;
	sTIM Tim;        //��ʱ��
	u32  Time;
	float halfDt;
	float Cb2n[3][3];
	float AngOff[3]; //ƫ����
	
	float DatAcc[3];
	float DatMag[3];
	float DatGyr[3];
	float pqr[3];    //������
	float Ang[3];    //�ںϺ�ĽǶ�
	float Ang1[3];    //�ںϺ�ĽǶ�
	float Ang_Init[3]; //���ټƺʹ����Ƴ�ʼ���ó����ĽǶ�
	float AccB[3];   //��������ϵ���ٶ�
	float Acc[3];    //NED���ٶ�
	float Uvw[3];    //NED�ٶ�
	float Xyz[3];    //NEDλ��
	float AccH[3];
	float UvwH[3];
	float XyzH[3];
	float Fac;       //��б������
	float w,x,y,z;   //��Ԫ��
	float w1,x1,y1,z1;   //��Ԫ��
	float q_init[4]; //��ʼ����Ԫ��
	float q_init_temp[4]; //��ʼ����Ԫ��
	bool  q_init_flag; //��ʼ����Ԫ����־λ
	float P_k[49];//EKF_Attitude������һ��Ԥ����
	float G;
	float x_p[7];//��һʱ��EKFһ��Ԥ����
	int circle_loop;
	float exInt,eyInt,ezInt;
	u32   ahrs_update_time;
	bool  gps_ins_update_flag;
}sAHRS;
   
#define AHRS_NUM 2

extern sAHRS ahrs[AHRS_NUM];

void Ahrs_Init_Para(sAHRS *ele,sAHRS_CFG *elecfg);
bool Ahrs_Init(sAHRS *ele);
bool Ahrs_Calc(sAHRS *ele);
bool Ahrs_Calc_EKF(sAHRS *ele);
#endif
