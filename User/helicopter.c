#include "helicopter.h"
sHeli Heli;
sLIMITS N_limits,E_limits,D_limits,Vel_N_limits,Vel_E_limits,Vel_D_limits,roll_limits,pitch_limits,yaw_limits,coll_limits,roll_rate_limits,pitch_rate_limits;

void Set_Limits(void)
{
  N_limits.ProErrorLimits[0] = -1.00f;
	N_limits.ProErrorLimits[1] =  1.00f;
	N_limits.VelErrorLimits[0] = -10.0f; 
	N_limits.VelErrorLimits[1] =  10.0f;
	N_limits.IntStateLimits[0] = -0.8f;
	N_limits.IntStateLimits[1] =  0.8f;
	N_limits.OutErrorLimits[0] = -0.50f;
	N_limits.OutErrorLimits[1] =  0.50f;
	
	E_limits.ProErrorLimits[0] = -1.0f;
	E_limits.ProErrorLimits[1] =  1.0f;
	E_limits.VelErrorLimits[0] = -10.0f; 
	E_limits.VelErrorLimits[1] =  10.0f;
	E_limits.IntStateLimits[0] = -0.8f;
	E_limits.IntStateLimits[1] =  0.8f;
	E_limits.OutErrorLimits[0] = -0.461f;
	E_limits.OutErrorLimits[1] =  0.461f;
	
	D_limits.ProErrorLimits[0] =-5.0f;      
	D_limits.ProErrorLimits[1] = 5.0f;      
	D_limits.VelErrorLimits[0] = -8.0f; 
	D_limits.VelErrorLimits[1] =  8.0f;
	D_limits.IntStateLimits[0] = -2.0f;
	D_limits.IntStateLimits[1] =  2.0f;
	D_limits.OutErrorLimits[0] = -10.0f;
	D_limits.OutErrorLimits[1] =  10.0f;
	
  Vel_N_limits.ProErrorLimits[0] = -1.00f;
	Vel_N_limits.ProErrorLimits[1] =  1.00f;
	Vel_N_limits.VelErrorLimits[0] = -10.0f; 
	Vel_N_limits.VelErrorLimits[1] =  10.0f;
	Vel_N_limits.IntStateLimits[0] = -3.0f;
	Vel_N_limits.IntStateLimits[1] =  3.0f;
	Vel_N_limits.OutErrorLimits[0] = -0.50f;
	Vel_N_limits.OutErrorLimits[1] =  0.50f;
	
	Vel_E_limits.ProErrorLimits[0] = -1.0f;
	Vel_E_limits.ProErrorLimits[1] =  1.0f;
	Vel_E_limits.VelErrorLimits[0] = -10.0f; 
	Vel_E_limits.VelErrorLimits[1] =  10.0f;
	Vel_E_limits.IntStateLimits[0] = -3.0f;
	Vel_E_limits.IntStateLimits[1] =  3.0f;
	Vel_E_limits.OutErrorLimits[0] = -0.461f;
	Vel_E_limits.OutErrorLimits[1] =  0.461f;
	
	Vel_D_limits.ProErrorLimits[0] = -1.0f;
	Vel_D_limits.ProErrorLimits[1] =  1.0f;
	Vel_D_limits.VelErrorLimits[0] = -10.0f; 
	Vel_D_limits.VelErrorLimits[1] =  10.0f;
	Vel_D_limits.IntStateLimits[0] = -3.0f;
	Vel_D_limits.IntStateLimits[1] =  3.0f;
	Vel_D_limits.OutErrorLimits[0] = -2.0f;
	Vel_D_limits.OutErrorLimits[1] =  2.0f;
	
	roll_limits.ProErrorLimits[0] = -20.0f*D2R;
	roll_limits.ProErrorLimits[1] =  20.0f*D2R;
	roll_limits.VelErrorLimits[0] = -40.0f*D2R;
	roll_limits.VelErrorLimits[1] =  40.0f*D2R;
	roll_limits.IntStateLimits[0] = - 1.2f*D2R;
	roll_limits.IntStateLimits[1] =   1.2f*D2R;
	roll_limits.OutErrorLimits[0] = -150.0f*D2R;
	roll_limits.OutErrorLimits[1] =  150.0f*D2R;
        
	pitch_limits.ProErrorLimits[0] = -20.0f*D2R;
	pitch_limits.ProErrorLimits[1] =  20.0f*D2R;
	pitch_limits.VelErrorLimits[0] = -40.0f*D2R;
	pitch_limits.VelErrorLimits[1] =  40.0f*D2R;
	pitch_limits.IntStateLimits[0] = - 1.2f*D2R;
	pitch_limits.IntStateLimits[1] =   1.2f*D2R;
	pitch_limits.OutErrorLimits[0] = -150.0f*D2R;
	pitch_limits.OutErrorLimits[1] =  150.0f*D2R;
        
	yaw_limits.ProErrorLimits[0] = -30.0f*D2R;
	yaw_limits.ProErrorLimits[1] =  30.0f*D2R;
	yaw_limits.VelErrorLimits[0] = -100.0f*D2R;
	yaw_limits.VelErrorLimits[1] =  100.0f*D2R;
	yaw_limits.IntStateLimits[0] = -90.0f*D2R;
	yaw_limits.IntStateLimits[1] =  90.0f*D2R;
	yaw_limits.OutErrorLimits[0] = -30.0f*D2R;
	yaw_limits.OutErrorLimits[1] =  30.0f*D2R;
                 
	coll_limits.ProErrorLimits[0] = -2.0f;
	coll_limits.ProErrorLimits[1] =  2.0f;
	coll_limits.VelErrorLimits[0] = -0.5f;
	coll_limits.VelErrorLimits[1] =  0.5f;
	coll_limits.IntStateLimits[0] = -3.5f;
	coll_limits.IntStateLimits[1] =  3.5f;
	coll_limits.OutErrorLimits[0] = -6.0f;
	coll_limits.OutErrorLimits[1] =  6.0f;
        
	roll_rate_limits.ProErrorLimits[0] = -150.0f*D2R;
	roll_rate_limits.ProErrorLimits[1] =  150.0f*D2R;
	roll_rate_limits.VelErrorLimits[0] = -100.0f*D2R;
	roll_rate_limits.VelErrorLimits[1] =  100.0f*D2R;
	roll_rate_limits.IntStateLimits[0] = -90.0f*D2R;
	roll_rate_limits.IntStateLimits[1] =  90.0f*D2R;
	roll_rate_limits.OutErrorLimits[0] = -150.0f*D2R;
	roll_rate_limits.OutErrorLimits[1] =  150.0f*D2R;
        
	pitch_rate_limits.ProErrorLimits[0] = -150.0f*D2R;
	pitch_rate_limits.ProErrorLimits[1] =  150.0f*D2R;
	pitch_rate_limits.VelErrorLimits[0] = -100.0f*D2R;
	pitch_rate_limits.VelErrorLimits[1] =  100.0f*D2R;
	pitch_rate_limits.IntStateLimits[0] = -90.0f*D2R;
	pitch_rate_limits.IntStateLimits[1] =  90.0f*D2R;
	pitch_rate_limits.OutErrorLimits[0] = -150.0f*D2R;
	pitch_rate_limits.OutErrorLimits[1] =  150.0f*D2R;
}


void Heli_Init(sHeli *ele)
{
	ele->heli_type = Flybar;
	ele->pwm_coll_mid = 603;
	ele->pwm_pitch_mid = 990;
	ele->pwm_roll_mid = 1380;
	ele->pwm_yaw_mid = 1508;
	
	ele->motor_mode = 0 ;  //恒速模式
	//设置控制器限幅
	Set_Limits();
}

/***************************************************\
功能：
  舵机加耦、解耦
说明：
  1、分为两种直升机机型 1、有副翼 2、有副翼
注意：每架飞机都不一样，需要进行修改
\***************************************************/

void Flybar_DecoupingMatrix(sHeli *ele,sRC *rc)
{
	
	ele->manual_servo_width[0] = (-rc->PPM[5] + rc->PPM[0] + 2*rc->PPM[1]) / 3;
	ele->manual_servo_width[1] = (rc->PPM[5] + rc->PPM[0]) /2;
	ele->manual_servo_width[2] = (rc->PPM[5] - rc->PPM[0] + rc->PPM[1]) / 3;	
	
	ele->manual_servo_width[3] = rc->PPM[3];
	
	ele->auto_servo_width[0] = ele->manual_servo_width[1];//roll
	ele->auto_servo_width[1] = ele->manual_servo_width[0];//pitch
	ele->auto_servo_width[2] = rc->PPM[2];//throttle
	ele->auto_servo_width[3] = rc->PPM[3];//yaw
	ele->auto_servo_width[4] = rc->PPM[4];//gyro	
	ele->auto_servo_width[5] = ele->manual_servo_width[2];//coll	
	ele->auto_servo_width[6] = rc->PPM[6];//control_mode
	ele->auto_servo_width[7] = rc->PPM[7];//aux
}

void Flybar_CoupingMatrix(sHeli *ele)
{ 
	
	ele->couple_servo_width[2] = 0.5f*ele->auto_servo_width[1] + ele->auto_servo_width[0] - ele->auto_servo_width[5];
	ele->couple_servo_width[0] = -0.5f*ele->auto_servo_width[1] + ele->auto_servo_width[0] + ele->auto_servo_width[5];
	ele->couple_servo_width[1] = ele->auto_servo_width[1] + ele->auto_servo_width[5];
}

void Flybarless_H1_DecoupingMatrix(sHeli *ele,sRC *rc)
{
	ele->manual_servo_width[0] = rc->PPM[0];
	ele->manual_servo_width[1] = rc->PPM[1];
	ele->manual_servo_width[2] = rc->PPM[5];

	ele->manual_servo_width[3] = rc->PPM[3];
	
	ele->auto_servo_width[0] = rc->PPM[0];
	ele->auto_servo_width[1] = rc->PPM[1];
	
	ele->auto_servo_width[3] = rc->PPM[3];	
	ele->auto_servo_width[4] = rc->PPM[4];	
	ele->auto_servo_width[5] = rc->PPM[5];	
	ele->auto_servo_width[6] = rc->PPM[6];
	ele->auto_servo_width[7] = rc->PPM[7];		
}

void Flybarless_CoupingMatrix(sHeli *ele)
{
	ele->auto_servo_width_temp[0] = (ele->auto_servo_width[0]-1500) ;//roll 13/25
	ele->auto_servo_width_temp[1] = (ele->auto_servo_width[1]-1500) ;//pitch
	ele->auto_servo_width_temp[2] = (ele->auto_servo_width[5]-1500) * 3/5;//coll  
	
  int aileron_subtrim    =  30; // - 3 60 -10 ;pidPit1->Kp;
  int elevator_subtrim   =  80;//38 -70 -170 ;pidPitRate1->Ki;
  int collective_subtrim =  25; //30 -15 -10 ;pidPitRate1->Kp;

//	int aileron_subtrim    =  pidPit1->Kp; // - 3 60 -10 ;pidPit1->Kp;
//  int elevator_subtrim   =  pidPitRate1->Ki;//38 -70 -170 ;pidPitRate1->Ki;
//  int collective_subtrim =  pidPitRate1->Kp; //30 -15 -10 ;pidPitRate1->Kp;
	
	ele->couple_servo_width[0] = ele->auto_servo_width_temp[0] * sqrtf(3) /2 - ele->auto_servo_width_temp[1]/2 + ele->auto_servo_width_temp[2]  + 1500 + aileron_subtrim;
	ele->couple_servo_width[1] =  															               ele->auto_servo_width_temp[1]  + ele->auto_servo_width_temp[2]  + 1500 + elevator_subtrim;
	ele->couple_servo_width[2] = ele->auto_servo_width_temp[0] * sqrtf(3) /2 + ele->auto_servo_width_temp[1]/2 - ele->auto_servo_width_temp[2]  + 1500 + collective_subtrim;
}

void Flybar_servos_widths_output(sHeli *ele)
{
	ele->output_servo_width[0] = ele->couple_servo_width[0];
	ele->output_servo_width[1] = ele->couple_servo_width[1];
	ele->output_servo_width[2] = ele->auto_servo_width[2];
	ele->output_servo_width[3] = ele->auto_servo_width[3];
	ele->output_servo_width[4] = ele->auto_servo_width[4];
	ele->output_servo_width[5] = ele->couple_servo_width[2];
	ele->output_servo_width[6] = ele->auto_servo_width[6];
	ele->output_servo_width[7] = ele->auto_servo_width[7];

}

void Flybarless_servos_widths_output(sHeli *ele)
{

//	ele->output_servo_width[0] = rc->PPM[0];
//	ele->output_servo_width[1] = rc->PPM[1];
//	ele->output_servo_width[2] = rc->PPM[2];
//	ele->output_servo_width[3] = rc->PPM[3];
//	ele->output_servo_width[4] = rc->PPM[4];
//	ele->output_servo_width[5] = rc->PPM[5];
//	ele->output_servo_width[6] = rc->PPM[6];
//	ele->output_servo_width[7] = rc->PPM[7];
	
	ele->output_servo_width[0] = ele->couple_servo_width[0];
	ele->output_servo_width[1] = ele->couple_servo_width[1];
	ele->output_servo_width[2] = ele->auto_servo_width[2];
	ele->output_servo_width[3] = ele->auto_servo_width[3];
	ele->output_servo_width[4] = ele->auto_servo_width[4];
	ele->output_servo_width[5] = ele->couple_servo_width[2];
	ele->output_servo_width[6] = ele->auto_servo_width[6];
	ele->output_servo_width[7] = ele->auto_servo_width[7];
}
