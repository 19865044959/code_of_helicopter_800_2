#ifndef _HELICOPTER_H
#define _HELICOPTER_H

#include "rc.h"
#include "pid.h"

typedef enum
{
	Flybar = 0,									//�и���
	Flybarless = 1,							//�޸��� ң����Ҫ��H-1ģʽ
}eHeli_type;

typedef enum
{
	MODE_MANUAL = 0x00U,          //�ֿ�
	MODE_ATTITUDE  = 0x01U,       //��̬
	MODE_HIGH_HOLD = 0x02U,       //����
	MODE_POSITION_HOLD = 0x03U,   //����
	MODE_TARGET = 0x04U,          //����
}eMODE;

typedef struct
{
	int heli_type;//ֱ��������
	int control_mode;//����ģʽ
	int loop_count;//�´������õĳ���ѭ������
	int Mau_Loop_count;//�ֿ�ģʽ����ѭ������
	int Att_Loop_count;//��̬ģʽ����ѭ������
	int Pos_Loop_count;//����ģʽ����ѭ������
	
	int manual_servo_width[4];//�ֿ�ң����ֵ�������ɼ���λ��
	int auto_servo_width[8];//�Կ�״̬�µĶ�����ֵ
	int auto_servo_width_temp[3];//�޸���������Ķ������ֵ
	int couple_servo_width[3];//�����Ķ�����ֵ
	int output_servo_width[8];//�������Ķ�����ֵ
	int amplify;
		
	int pwm_roll_mid;//roll ��λ��
	int pwm_pitch_mid;//pitch ��λ��
	int pwm_yaw_mid;//yaw ��λ��
	int pwm_coll_mid;//coll ��λ��
		
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
	float output[4];//������
	
	u8  motor_mode;   //����ģʽ
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
