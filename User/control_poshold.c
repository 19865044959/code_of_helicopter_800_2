#include "control_poshold.h"

/***************************************************\
功能：
  直升机定点模式
说明：
  1、分为两种直升机机型 1、有副翼 2、有副翼
\***************************************************/
void Pos_NED_Command(sHeli *ele)
{
	ele->pos_command_ned[0] = trajectory.cur_target_pos_ned[0];
	
	ele->pos_command_ned[1] = trajectory.cur_target_pos_ned[1];
	
	ele->pos_command_ned[2] = trajectory.cur_target_pos_ned[2];
}

void Pos_Vel_NED_Feedback(sHeli *ele)
{
	ele->pos_feedback_ned[0] = gps[0].NED[0];
	
	ele->pos_feedback_ned[1] = gps[0].NED[1];
	
	ele->pos_feedback_ned[2] = output_data_new.Pos[2];
	
	ele->vel_feedback_ned[0] = output_data_new.Vel[0];
	
	ele->vel_feedback_ned[1] = output_data_new.Vel[1];
	
	ele->vel_feedback_ned[2] = output_data_new.Vel[2];
}

void Pos_Vel_NED_Loop(sHeli *ele)
{
	#define SPEED_MAX 10
	float vel_command_norm;
	
	ele->pos_error_ned[0] = ele->pos_command_ned[0] - ele->pos_feedback_ned[0];
	ele->pos_error_ned[1] = ele->pos_command_ned[1] - ele->pos_feedback_ned[1];
	ele->pos_error_ned[2] = ele->pos_command_ned[2] - ele->pos_feedback_ned[2];
	
	ele->vel_command_ned[0] = ele->pos_error_ned[0] * pidX->Kp;
	ele->vel_command_ned[1] = ele->pos_error_ned[1] * pidY->Kp;
	ele->vel_command_ned[2] = -ele->pos_error_ned[2] * pidZ->Kp;
	
//	vel_target_x += vel_desired[0];
//	vel_target_y += vel_desired[1];
	//diff with 500
	ele->vel_ned_ff[0] = 0.78f;//0.78f
	ele->vel_ned_ff[1] = 0.78f;
	ele->vel_ned_ff[2] = 0.78f;
	
	ele->vel_command_ned[0] += ele->vel_ned_ff[0] * trajectory.cur_target_vel_ned[0];
	ele->vel_command_ned[1] += ele->vel_ned_ff[1] * trajectory.cur_target_vel_ned[1];
	ele->vel_command_ned[2] += ele->vel_ned_ff[2] * trajectory.cur_target_vel_ned[2];
	
	vel_command_norm = sqrt(ele->vel_command_ned[0] *ele->vel_command_ned[0] + ele->vel_command_ned[1] *ele->vel_command_ned[1]);
	if(vel_command_norm > SPEED_MAX)
	{
		ele->vel_command_ned[0] = ele->vel_command_ned[0] * SPEED_MAX /vel_command_norm;
		ele->vel_command_ned[1] = ele->vel_command_ned[1] * SPEED_MAX /vel_command_norm;
	}
}

void Vel_Acc_NED_Loop(sHeli *ele)
{
	float vel_command_ned_error[3];//,vel_target_y_error,vel_target_z_error;
	float d_vel_command_ned_error[3];//,d_vel_target_y_error,d_vel_target_z_error;
	float accel_feedforward_ned[3];//,accel_feedforward_y,accel_feedforward_z;
	//Get_Vel_Loop_PID();
	
	accel_feedforward_ned[0] = (ele->vel_command_ned[0] - ele->vel_command_ned_last[0])/0.01f;
	accel_feedforward_ned[1] = (ele->vel_command_ned[1] - ele->vel_command_ned_last[1])/0.01f;
	accel_feedforward_ned[2] = (ele->vel_command_ned[2] - ele->vel_command_ned_last[2])/0.01f;
	
	ele->vel_command_ned_last[0] = ele->vel_command_ned[0];
	ele->vel_command_ned_last[1] = ele->vel_command_ned[1]; 
	ele->vel_command_ned_last[2] = ele->vel_command_ned[2]; 
	
#define vel_x_alpha   0.557f//20hz
	vel_command_ned_error[0] = ele->vel_command_ned[0] - ele->vel_feedback_ned[0];
	d_vel_command_ned_error[0] = (vel_command_ned_error[0] - ele->vel_command_ned_error_old[0]) * vel_x_alpha;
	vel_command_ned_error[0] = ele->vel_command_ned_error_old[0] + d_vel_command_ned_error[0];
	ele->vel_command_ned_error_old[0] = vel_command_ned_error[0];
	
	ele->vel_intergrate_ned_error[0] = fConstrain(ele->vel_intergrate_ned_error[0] + vel_command_ned_error[0] * 0.01f,Vel_N_limits.IntStateLimits[0],Vel_N_limits.IntStateLimits[1]);

	ele->acc_command_ned[0] = vel_command_ned_error[0] * pidVelX->Kp + ele->vel_intergrate_ned_error[0] * pidVelX->Ki + accel_feedforward_ned[0];
		
#define vel_y_alpha  0.557f//20hz
	vel_command_ned_error[1] = ele->vel_command_ned[1] - ele->vel_feedback_ned[1];
	d_vel_command_ned_error[1] = (vel_command_ned_error[1] - ele->vel_command_ned_error_old[1]) * vel_x_alpha;
	vel_command_ned_error[1] = ele->vel_command_ned_error_old[1] + d_vel_command_ned_error[1];
	ele->vel_command_ned_error_old[1] = vel_command_ned_error[1];
	
	ele->vel_intergrate_ned_error[1] = fConstrain(ele->vel_intergrate_ned_error[1] + vel_command_ned_error[1] * 0.01f,Vel_E_limits.IntStateLimits[0],Vel_E_limits.IntStateLimits[1]);

	ele->acc_command_ned[1] = vel_command_ned_error[1] * pidVelY->Kp + ele->vel_intergrate_ned_error[1] * pidVelY->Ki + accel_feedforward_ned[1];

#define vel_z_alpha 0.557f//20hz
	vel_command_ned_error[2] = ele->vel_command_ned[2] - ele->vel_feedback_ned[2];
	d_vel_command_ned_error[2] = (vel_command_ned_error[2] - ele->vel_command_ned_error_old[2]) * vel_x_alpha;
	vel_command_ned_error[2] = ele->vel_command_ned_error_old[2] + d_vel_command_ned_error[2];
	ele->vel_command_ned_error_old[2] = vel_command_ned_error[2];
	
	ele->acc_command_ned[2] = fConstrain(vel_command_ned_error[2] * pidVelZ->Kp + accel_feedforward_ned[2],Vel_D_limits.OutErrorLimits[0],Vel_D_limits.OutErrorLimits[1]);

	ele->acc_command_body[0] =  ele->acc_command_ned[0]*cos(ahrs[1].Ang[2]) + ele->acc_command_ned[1]*sin(ahrs[1].Ang[2]);
	
	ele->acc_command_body[1] = -ele->acc_command_ned[0]*sin(ahrs[1].Ang[2]) + ele->acc_command_ned[1]*cos(ahrs[1].Ang[2]);
	
	ele->acc_command_body[2] = ele->acc_command_ned[2];
	
}

void Acc_NE_Roll_Pitch_Loop(sHeli *ele)
{
	float acc_forward,acc_right;
	float acc_norm;
	float acc_command_error[2];//,acc_command_y_error;
	
	
	acc_norm = ele->acc_command_body[0] * ele->acc_command_body[0] + ele->acc_command_body[1] * ele->acc_command_body[1];
	if ((acc_norm > G_zinit) && (acc_norm > 0.0f))
	{
			ele->acc_command_body[0] = G_zinit * ele->acc_command_body[0]/acc_norm;
			ele->acc_command_body[1] = G_zinit * ele->acc_command_body[1]/acc_norm;
	}

#define jerk_enable
#ifdef jerk_enable
	float max_delta_accel = 0.01f * 17; 
	float acc_command_x_change = ele->acc_command_body[0] - ele->acc_command_body_last[0];
	float acc_command_y_change = ele->acc_command_body[1] - ele->acc_command_body_last[1];
	
	float acc_target_change_norm = acc_command_x_change * acc_command_x_change + acc_command_y_change * acc_command_y_change;
	if (acc_target_change_norm > max_delta_accel)
	{
			acc_command_x_change = acc_command_x_change * max_delta_accel/acc_target_change_norm;
			acc_command_y_change = acc_command_y_change * max_delta_accel/acc_target_change_norm;
	}
#endif
	
#define acc_x_alpha 0.557f//20hz
#define acc_y_alpha 0.557f//20hz
	float acc_command_jerk_limited[2];
	acc_command_jerk_limited[0] =  ele->acc_command_body_last[0] + acc_command_x_change;
	acc_command_jerk_limited[1] =  ele->acc_command_body_last[1] + acc_command_y_change;
	
//	ele->acc_command_body_last[0] = acc_command_jerk_limited[0];
//	ele->acc_command_body_last[1] = acc_command_jerk_limited[1];
	
	acc_command_error[0] = acc_command_jerk_limited[0] - ele->acc_command_body_last[0];
	ele->acc_command_body[0] = ele->acc_command_body_last[0] + acc_command_error[0] * acc_x_alpha;
	ele->acc_command_body_last[0] = ele->acc_command_body[0];
			
	acc_command_error[1] = acc_command_jerk_limited[1]-ele->acc_command_body_last[1];
	ele->acc_command_body[1] = ele->acc_command_body_last[1] + acc_command_error[1] * acc_y_alpha;
	ele->acc_command_body_last[1] = ele->acc_command_body[1];
			
	acc_forward = ele->acc_command_body[0];
	acc_right = ele->acc_command_body[1];
			
	ele->pitch_command_from_outloop = fConstrain(atan(-acc_forward/G_zinit),pitch_limits.ProErrorLimits[0],pitch_limits.ProErrorLimits[1]);
	float cos_pitch_target=cos(ele->pitch_command_from_outloop);
	ele->roll_command_from_outloop = fConstrain(atan(acc_right*cos_pitch_target/G_zinit),roll_limits.ProErrorLimits[0],roll_limits.ProErrorLimits[1]);      
}

void Altitude_Acc_Loop(sHeli *ele)
{
#define acc_z_alpha 0.557f
	float acc_command_z_error;
	float d_acc_command_z_error;
 
	acc_command_z_error = ele->acc_command_body[2] - ahrs[1].Acc[2];
	d_acc_command_z_error = (acc_command_z_error - ele->acc_command_d_error_old) * acc_z_alpha;
	acc_command_z_error = ele->acc_command_d_error_old + d_acc_command_z_error;
	ele->acc_command_d_error_old = acc_command_z_error;
	
	ele->acc_intergrate_d_error = fConstrain(ele->acc_intergrate_d_error + acc_command_z_error * 0.01f,coll_limits.IntStateLimits[0],coll_limits.IntStateLimits[1]);
	
	ele->output[3] = fConstrain(acc_command_z_error * pidAcc_z->Kp + ele->acc_intergrate_d_error * pidAcc_z->Ki,coll_limits.OutErrorLimits[0],coll_limits.OutErrorLimits[1]);      
	
	if(ele->auto_coll_control_mode)
	{
		ele->auto_servo_width[5] = ele->pwm_coll_mid - ele->output[3] * COLL_AMPLIFY;
	}
}


void PosHold_Control_Step(sHeli *ele)
{
	Pos_NED_Command(ele);
	
	Pos_Vel_NED_Feedback(ele);
	
	Pos_Vel_NED_Loop(ele);
	
	Vel_Acc_NED_Loop(ele);
	
	Acc_NE_Roll_Pitch_Loop(ele);

  Altitude_Acc_Loop(ele);
}
void Pos_Vel_Loop_Init(sHeli *ele)
{
	Heli.acc_intergrate_d_error =0.0f;
	for(int i=0;i<3;i++)
	{
		Heli.vel_intergrate_ned_error[i] =0.0f;
		Heli.acc_command_body_last[i] = 0.0f;
		Heli.acc_command_body_error_old[i] = 0.0f;
		Heli.vel_command_ned_last[i] = 0.0f;
		Heli.vel_command_ned_error_old[i] = 0.0f;
	} 
}
	
void Pos_Hold_Init(sHeli *ele)
{
	ele->yaw_command = ahrs[1].Ang[2];
	
	ele->pwm_coll_mid = ele->auto_servo_width[5];
	
	target_receive_flag = false;
	
	Target_Command_Reset();
	
	Target_Queue_Reset();
	
	Trajectory_Reset(&trajectory);
	
	Pos_Vel_Loop_Init(ele);
}

void Trajectory_Target_Receive(void)
{
	if(target_receive_flag)
	{
		if (false == Target_Queue_Push_Target())			
			Trajectory_Brake_Down_Step(&trajectory);
		trajectory.first_receive_target ++;
		target_receive_flag = false;
	}
}

void Heli_Poshold_control(sHeli *ele)
{
	switch(ele->heli_type)
	{
		case Flybar:
			ele->Pos_Loop_count = (ele->Pos_Loop_count+1)>50000? 1000 : ele->Pos_Loop_count+1;//循环次数
			ele->loop_count = ele->Pos_Loop_count;
		  if (ele->Pos_Loop_count == 1)		Pos_Hold_Init(ele);
			Flybar_DecoupingMatrix(ele,rc);//解耦程序,为了采集中位点
			Trajectory_Target_Receive();
			Trajectory_Step(&trajectory,trajectory.trajectory_mod);
			PosHold_Control_Step(ele);
			Attitude_Control_Step(ele);
		  Flybar_CoupingMatrix(ele);//加耦程序
			Flybar_servos_widths_output(ele);//最终舵机输出
			break;
		case Flybarless:
			ele->Pos_Loop_count = (ele->Pos_Loop_count+1)>50000? 1000 : ele->Pos_Loop_count+1;//循环次数
			ele->loop_count = ele->Pos_Loop_count;
		  //初始化
		  if(ele->Pos_Loop_count == 1)	Pos_Hold_Init(ele);
			Flybarless_H1_DecoupingMatrix(ele,rc);//解耦程序,为了采集中位点
			Trajectory_Target_Receive();
			Trajectory_Step(&trajectory,trajectory.trajectory_mod);
			PosHold_Control_Step(ele);
			Attitude_Control_Step(ele);
			Flybarless_CoupingMatrix(ele);//加耦程序
			Flybarless_servos_widths_output(ele);//最终舵机输出
			break;
		default:
			break;
	}
}
