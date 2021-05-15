#ifndef __AHRS_H
#define __AHRS_H

#include "mpu6050.h"
#include "adis16365.h"
#include "hmc5883.h"
#include "gps.h"
#include "ms5611.h"
#include "command.h"
#include "exit.h"
#include "pid.h"	//临时，PI参数

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
	sTIM Tim;        //计时器
	u32  Time;
	float halfDt;
	float Cb2n[3][3];
	float AngOff[3]; //偏移量
	
	float DatAcc[3];
	float DatMag[3];
	float DatGyr[3];
	float pqr[3];    //陀螺仪
	float Ang[3];    //融合后的角度
	float Ang1[3];    //融合后的角度
	float Ang_Init[3]; //加速计和磁力计初始化得出来的角度
	float AccB[3];   //机体坐标系加速度
	float Acc[3];    //NED加速度
	float Uvw[3];    //NED速度
	float Xyz[3];    //NED位置
	float AccH[3];
	float UvwH[3];
	float XyzH[3];
	float Fac;       //倾斜角因子
	float w,x,y,z;   //四元数
	float w1,x1,y1,z1;   //四元数
	float q_init[4]; //初始化四元数
	float q_init_temp[4]; //初始化四元数
	bool  q_init_flag; //初始化四元数标志位
	float P_k[49];//EKF_Attitude误差矩阵一步预测量
	float G;
	float x_p[7];//上一时刻EKF一步预测量
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
