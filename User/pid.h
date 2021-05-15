#ifndef __PID_H
#define __PID_H

#include "userlib.h"
#include "helicopter.h"

typedef struct
{
	float setpoint;		// �趨ֵ
	float Kp,Ki,Kd,Kb;
	float eLimit;
	float iLimit;
	float dLimit;
	float filter_para;
}sPID_CFG;

typedef struct
{
	float setpoint;		// �趨ֵ
	float feedback;		// ����ֵ
	float lastfeedback; // ��һ������ֵ
	float error;		// ���
	float lastError;    // ��һ�����
	float integral;		// ��ǰ����ֵ
	float eLimit;    //����޷�
	float iLimit;		// �����޷�ֵ
	float dLimit;   //΢���޷�
	float Kp;			// ����ϵ�� proportional gain
	float Ki;			// ����ϵ�� integral gain
	float Kd;			// ΢��ϵ�� differential gain
	float Kb;           // ΢������ϵ��
	float pout;			// (debugging)
	float iout;			// (debugging)
	float dout;			// (debugging)
	float output;		// ��ǰPID�����
	float signal;		// ��Ծ�ź�
	float filter_para; //��ͨ�˲�����
}sPID;

#define PID_NUM 18

extern sPID pid[PID_NUM];
extern sPID *pidRolRate1,*pidPitRate1,*pidAcc_z;
extern sPID *pidRol1,*pidPit1,*pidYaw1;
extern sPID *pidVelX,*pidVelY,*pidVelZ;
extern sPID *pidX,*pidY,*pidZ;
extern sPID *pidRolRate1_FF,*pidPitRate1_FF;
extern sPID *pidTmp;
extern sPID *pidmotor;
void Pid_Init_Para(sPID *ele,sPID_CFG *elecfg);
void Pid_Reset(sPID *ele);
float Pid_Position(sPID* ele, float SetXYZ, float FeedBackXYZ);
float Pid_Speed(sPID* ele, float SetSpeed, float FeedBackSpeed);
float Pid_Angle(sPID* ele, float SetAngle, float FeedBackAngle);
float Pid_RateAngle(sPID* ele, float SetRateAngle, float FeedBackRateAngle);
float Pid_PositionSpeed(sPID* ele, float SetPosition, float FeedBackPosition, float FeedBackSpeed);
double Pid_Controller(sPID *ele);
void Low_pass_filter(sPID *ele,float error);
void Integral_Init(void);
float Pid_tmp(sPID *ele);
float Pid_motor(sPID* ele, float SetSpeed, float FeedBackSpeed);
#endif
