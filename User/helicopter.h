#ifndef _HELICOPTER_H
#define _HELICOPTER_H

#include "rc.h"
#include "pid.h"

typedef enum
{
	Flybar = 0,									//有副翼
	Flybarless = 1,							//无副翼 遥控器要用H-1模式
}eHeli_type;

typedef enum
{
	MODE_MANUAL = 0x00U,          //手控
	MODE_ATTITUDE  = 0x01U,       //姿态
	MODE_HIGH_HOLD = 0x02U,       //定高
	MODE_POSITION_HOLD = 0x03U,   //定点
	MODE_TARGET = 0x04U,          //航线
}eMODE;

typedef struct
{
	int heli_type;//直升机类型
	int control_mode;//控制模式
	int loop_count;//下传数据用的程序循环次数
	int Mau_Loop_count;//手控模式程序循环次数
	int Att_Loop_count;//姿态模式程序循环次数
	int Pos_Loop_count;//定点模式程序循环次数
	
	int manual_servo_width[4];//手控遥控器值，用来采集中位点
	int auto_servo_width[8];//自控状态下的舵机输出值
	int auto_servo_width_temp[3];//无副翼加耦解算的舵机缓存值
	int couple_servo_width[3];//加耦后的舵机输出值
	int output_servo_width[8];//最后输出的舵机输出值
	int amplify;
		
	int pwm_roll_mid;//roll 中位点
	int pwm_pitch_mid;//pitch 中位点
	int pwm_yaw_mid;//yaw 中位点
	int pwm_coll_mid;//coll 中位点
		
	int auto_roll_control_mode;
	int auto_pitch_control_mode;
	int auto_yaw_control_mode;
	int auto_coll_control_mode;
	
	float roll_command;
	float pitch_command;
	float yaw_command;
	float roll_rate_command;
	float pitch_rate_command;
	float yaw_rate_command;
	float roll_command_from_rc;
	float pitch_command_from_rc;
	float yaw_command_from_rc;
	float yaw_rate_command_from_rc;
	float roll_command_from_outloop;
	float pitch_command_from_outloop;
	float yaw_rate_command_from_outloop;
	float roll_rate_feedforward;
	float pitch_rate_feedforward;
	float yaw_rate_feedforward;
	float roll_rate_feedforward_bf;
	float pitch_rate_feedforward_bf;
	float yaw_rate_feedforward_bf;
	
	float pos_command_ned[3];
	float pos_feedback_ned[3];
	float pos_error_ned[3];
	float vel_command_ned[3];
	float vel_command_ned_last[3];
	float vel_command_ned_error_old[3];
	float vel_intergrate_ned_error[3];
	float vel_ned_ff[3];
	float vel_feedback_ned[3];
	float acc_command_ned[3];
	float acc_command_body[3];
	float acc_command_body_last[3];
	float acc_command_body_error_old[3];
	float acc_command_d_error_old;
	float acc_intergrate_d_error;
	
	float roll_feedback;
	float pitch_feedback;
	float yaw_feedback;
	float roll_rate_feedback;
	float pitch_rate_feedback;
	float yaw_rate_feedback;
	
	float angle_error_ef[3];
	float angle_error_bf[3];
	float output[4];//控制量
	
	u8  motor_mode;   //恒速模式
}sHeli;

typedef struct limits
{
	float ProErrorLimits[2];
	float VelErrorLimits[2];
	float IntStateLimits[2];
	float OutErrorLimits[2];
	float int_state;
} sLIMITS;

#define CONTROL_HZ 100
#define CONTROL_DT 0.01f
#define G_zinit 9.84f
extern sHeli Heli;
extern sLIMITS N_limits,E_limits,D_limits,Vel_N_limits,Vel_E_limits,Vel_D_limits,roll_limits,pitch_limits,yaw_limits,coll_limits,roll_rate_limits,pitch_rate_limits;

void Heli_Init(sHeli *ele);
void Flybar_DecoupingMatrix(sHeli *ele,sRC *rc);
void Flybar_CoupingMatrix(sHeli *ele);
void Flybarless_H1_DecoupingMatrix(sHeli *ele,sRC *rc);
void Flybarless_CoupingMatrix(sHeli *ele);
void Flybar_servos_widths_output(sHeli *ele);
void Flybarless_servos_widths_output(sHeli *ele);

#endif
