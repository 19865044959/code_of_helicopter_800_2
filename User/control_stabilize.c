#include "control_stabilize.h"
/***************************************************\
功能：
  直升机姿态模式
说明：
  1、分为两种直升机机型 1、有副翼 2、有副翼
\***************************************************/
void Angle_From_Rc_Or_Outloop(sHeli *ele)
{
	if(ele->control_mode == MODE_ATTITUDE)
	{
		ele->roll_command_from_rc = ((abs((ROLL_PWM_REMOTE_MID-ele->manual_servo_width[1])))>20.0f?(ele->manual_servo_width[1]-ROLL_PWM_REMOTE_MID):0.0f)/15.0f*D2R + (trim_roll_angle * D2R); // 15
	
		ele->pitch_command_from_rc = ((abs((ele->manual_servo_width[0]-PITCH_PWM_REMOTE_MID)))>20.0f?(ele->manual_servo_width[0]-PITCH_PWM_REMOTE_MID):0.0f)/15.0f*D2R + (trim_pitch_angle * D2R); // 15
	
		ele->yaw_rate_command_from_rc = (((abs((YAW_PWM_REMOTE_MID-ele->manual_servo_width[3]))))>70.0f?(YAW_PWM_REMOTE_MID-ele->manual_servo_width[3]):0.0f)/4.0f*D2R; // 4
	}
	else if(ele->control_mode == MODE_POSITION_HOLD)
	{
		ele->roll_command_from_rc = ele->roll_command_from_outloop + (trim_roll_angle * D2R);
	
		ele->pitch_command_from_rc = ele->pitch_command_from_outloop + (trim_pitch_angle * D2R);
	
		ele->yaw_rate_command_from_rc = ele->yaw_rate_command_from_outloop;

	}
}

void Ang_Loop_Feedback(sHeli *ele,sAHRS *ahrs)
{
	ele->roll_feedback = ahrs->Ang[0];
	
	ele->pitch_feedback = ahrs->Ang[1];
	
	ele->yaw_feedback = ahrs->Ang[2];
}

void Ang_Rate_Loop_Feedback(sHeli *ele,sAHRS *ahrs)
{
	ele->roll_rate_feedback = ahrs->pqr[0];
	
	ele->pitch_rate_feedback = ahrs->pqr[1];
}

void Angle_Loop(sHeli *ele)
{
	float rate_ef_desired;
	float rate_change_limit;
	
	rate_change_limit = ACCEL_ROLL_MAX * CONTROL_DT * D2R;//角加速度限制
	rate_ef_desired = sqrt_controller(ele->roll_command_from_rc - ele->roll_command, SMOOTHING_GAIN, ACCEL_ROLL_MAX * D2R);//根据当前遥控器给定目标值与当前控制器的目标值的差，计算角速度前馈
  ele->roll_rate_feedforward = fConstrain(rate_ef_desired, ele->roll_rate_feedforward - rate_change_limit, ele->roll_rate_feedforward+rate_change_limit);//计算出的角速度前馈限幅
	//ele->roll_rate_feedforward = 0 ;
	rate_change_limit = ACCEL_PITCH_MAX * CONTROL_DT * D2R;
	rate_ef_desired = sqrt_controller(ele->pitch_command_from_rc - ele->pitch_command, SMOOTHING_GAIN, ACCEL_PITCH_MAX * D2R);
  ele->pitch_rate_feedforward = fConstrain(rate_ef_desired, ele->pitch_rate_feedforward-rate_change_limit, ele->pitch_rate_feedforward+rate_change_limit);
	 //ele->pitch_rate_feedforward = 0 ;
	rate_change_limit = ACCEL_YAW_MAX * CONTROL_DT * D2R;
  ele->yaw_rate_feedforward += fConstrain(ele->yaw_rate_command_from_rc - ele->yaw_rate_feedforward,-rate_change_limit,rate_change_limit);
	//ele->yaw_rate_feedforward = 0 ;
	ele->roll_rate_feedforward_bf = ele->roll_rate_feedforward - sinf(ahrs[1].Ang[1]) * ele->yaw_rate_feedforward;
	ele->pitch_rate_feedforward_bf = cosf(ahrs[1].Ang[0])  * ele->pitch_rate_feedforward + sinf(ahrs[1].Ang[0]) * cosf(ahrs[1].Ang[1]) * ele->yaw_rate_feedforward; 
	ele->yaw_rate_feedforward_bf = -sinf(ahrs[1].Ang[0]) * ele->pitch_rate_feedforward + cosf(ahrs[1].Ang[1]) * cosf(ahrs[1].Ang[0]) * ele->yaw_rate_feedforward;

	update_ef_roll_pitch_angle_and_error(ele);
  Angle_error_ef_2_bf(ele);
	update_rate_bf_targets(ele);
}

void update_ef_roll_pitch_angle_and_error(sHeli *ele)
{
	ele->angle_error_ef[0] = fConstrain(ele->roll_command - ele->roll_feedback,-50.0f * D2R, 50.0f * D2R);
	ele->angle_error_ef[1] = fConstrain(ele->pitch_command - ele->pitch_feedback,-50.0f * D2R, 50.0f * D2R);
	ele->angle_error_ef[2] = ele->yaw_command - ele->yaw_feedback;
	
	if(ele->angle_error_ef[2] > 180 * D2R)
	{
		ele->angle_error_ef[2] = ele->angle_error_ef[2] - 2 * 180 * D2R;
	}
	else if (ele->angle_error_ef[2] < -180 * D2R)
	{
		ele->angle_error_ef[2] = ele->angle_error_ef[2] + 2 * 180 * D2R;
	}
	
	ele->roll_command += ele->roll_rate_feedforward * CONTROL_DT;
	ele->pitch_command += ele->pitch_rate_feedforward * CONTROL_DT;
	ele->yaw_command += ele->yaw_rate_feedforward * CONTROL_DT;
	
	if(ele->yaw_command  > 180 * D2R)
	{
		ele->yaw_command  = ele->yaw_command - 2 * 180 * D2R;
	}
	else if (ele->yaw_command  < -180 * D2R)
	{
		ele->yaw_command  = ele->yaw_command  + 2 * 180 * D2R;
	}
}

void Angle_error_ef_2_bf(sHeli *ele)
{
	ele->angle_error_bf[0] = ele->angle_error_ef[0] - sinf(ahrs[1].Ang[1]) * ele->angle_error_ef[2];
	ele->angle_error_bf[1] = cosf(ahrs[1].Ang[0])  * ele->angle_error_ef[1] + sinf(ahrs[1].Ang[0]) * cosf(ahrs[1].Ang[1]) * ele->angle_error_ef[2];
	ele->angle_error_bf[2] = -sinf(ahrs[1].Ang[0]) * ele->angle_error_ef[1] + cosf(ahrs[1].Ang[1]) * cosf(ahrs[1].Ang[0]) * ele->angle_error_ef[2];
}

void update_rate_bf_targets(sHeli *ele)
{
	 ele->roll_rate_command = sqrt_controller(ele->angle_error_bf[0], pidRol1->Kp, fConstrain(ACCEL_ROLL_MAX/2.0f * D2R,  AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN * D2R, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX * D2R)) + ele->roll_rate_feedforward_bf;
	 ele->pitch_rate_command = sqrt_controller(ele->angle_error_bf[1], pidPit1->Kp, fConstrain(ACCEL_PITCH_MAX/2.0f * D2R,  AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN * D2R, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX * D2R)) + ele->pitch_rate_feedforward_bf;
	 ele->yaw_rate_command = sqrt_controller(ele->angle_error_bf[2], pidYaw1->Kp, fConstrain(ACCEL_YAW_MAX/2.0f * D2R,  AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN * D2R, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX * D2R)) +  ele->yaw_rate_feedforward_bf;
}

float sqrt_controller(float error, float p, float second_ord_lim)
{
    float linear_dist = second_ord_lim/sq(p);

    if (error > linear_dist) {
        return sqrtf(2.0f*second_ord_lim*(error-(linear_dist/2.0f)));
    } else if (error < -linear_dist) {
        return -sqrtf(2.0f*second_ord_lim*(-error-(linear_dist/2.0f)));
    } else {
        return error*p;
    }
}

void Angle_Rate_Loop(sHeli *ele)
{
	 #define roll_rate_alpha 0.55f  //20Hz
	 #define pitch_rate_alpha 0.55f //20Hz
	 #define roll_rate_feedfoward_alpha 0.385f//10Hz
	 #define pitch_rate_feedfoward_alpha 0.385f//10Hz

	 double angle_rate_error;
	 double d_error;
	 double rate_FF;
	 double d_rate_FF;
	 
	 //滚转通道
	 rate_FF = ele->roll_rate_command * 0.08f;// pidAcc_z->Kp;//计算前馈量 原0.08
	 d_rate_FF = (rate_FF - pidRolRate1_FF->lastError) * roll_rate_feedfoward_alpha;//计算低通增量
	 pidRolRate1_FF->error = pidRolRate1_FF->lastError + d_rate_FF;
	 pidRolRate1_FF->lastError = pidRolRate1_FF->error;

	 angle_rate_error = ele->roll_rate_command - ele->roll_rate_feedback;//滚转角速度误差   
	 d_error = (angle_rate_error - pidRolRate1->lastError) * roll_rate_alpha;//低通滤波器解算出误差量
	 pidRolRate1->error = pidRolRate1->lastError + d_error;//在原来的基础上叠加误差量
	 pidRolRate1->error = fConstrain(pidRolRate1->error,-157.3f * D2R, 157.3f * D2R);//限幅
	 pidRolRate1->lastError = pidRolRate1->error;

//	 if(pidRolRate1->integral >0)
//		 pidRolRate1->integral-=(pidRolRate1->integral-0) * 0.01f;
//	 else
//		 pidRolRate1->integral-=(pidRolRate1->integral+0) * 0.01f;
	 
	 pidRolRate1->integral = fConstrain(pidRolRate1->integral + pidRolRate1->error * CONTROL_DT,-90.0f * D2R, 90.0f * D2R);
	 pidRolRate1->pout = pidRolRate1->error * pidRolRate1->Kp;
	 pidRolRate1->iout = pidRolRate1->integral * pidRolRate1->Ki;
	 
	 ele->output[0]= fConstrain(pidRolRate1->pout + pidRolRate1->iout + pidRolRate1_FF->error,-57.3f * D2R, 57.3f * D2R) ;
	 if(ele->auto_roll_control_mode == 1)
	 {
		 ele->auto_servo_width[0] = ele->pwm_roll_mid + ele->output[0] * RAD2PWM;
	 }
	 //俯仰通道
	 rate_FF = ele->pitch_rate_command * 0.09f;//pidAcc_z->Ki;//计算前馈量 原0.07
	 d_rate_FF = (rate_FF - pidPitRate1_FF->lastError) * roll_rate_feedfoward_alpha;//计算低通增量量
	 pidPitRate1_FF->error = pidPitRate1_FF->lastError + d_rate_FF;
	 pidPitRate1_FF->lastError = pidPitRate1_FF->error;
	 
	 angle_rate_error = ele->pitch_rate_command - ele->pitch_rate_feedback;//俯仰角速度误差   
	 d_error = (angle_rate_error - pidPitRate1->lastError) * pitch_rate_alpha;//低通滤波器解算出误差量
	 pidPitRate1->error = pidPitRate1->lastError + d_error;//在原来的基础上叠加误差量
	 pidPitRate1->error = fConstrain(pidPitRate1->error,-157.3f * D2R, 157.3f * D2R);//限幅
	 pidPitRate1->lastError =  pidPitRate1->error;
	 
//	 if(pidPitRate1->integral >0)
//		 pidPitRate1->integral-=(pidPitRate1->integral-0) * 0.01f;
//	 else
//		 pidPitRate1->integral-=(pidPitRate1->integral+0) * 0.01f;
	 
	 
	 pidPitRate1->integral = fConstrain(pidPitRate1->integral + pidPitRate1->error * CONTROL_DT,-90.0f * D2R, 90.0f * D2R);
	 pidPitRate1->iout = pidPitRate1->integral * pidPitRate1->Ki;
	 pidPitRate1->pout = pidPitRate1->error * pidPitRate1->Kp;
	 
	 ele->output[1]= fConstrain(pidPitRate1->pout + pidPitRate1->iout + pidPitRate1_FF->error,-57.3f * D2R, 57.3f * D2R) ;
	 if(ele->auto_pitch_control_mode == 1)
	 {
		 ele->auto_servo_width[1] = ele->pwm_pitch_mid + ele->output[1] * RAD2PWM;
	 }
	 //航向通道
	 if(ele->auto_yaw_control_mode == 1)
	 {
		 ele->auto_servo_width[3] = ele->pwm_yaw_mid + ele->yaw_rate_command/GAIN_REMOTE_TO_RATE;
	 }
}

void Attitude_Control_Step(sHeli *ele)
{
	Angle_From_Rc_Or_Outloop(ele);
	
	Ang_Loop_Feedback(ele,&ahrs[1]);
	
	Ang_Rate_Loop_Feedback(ele,&ahrs[1]);
	
	Angle_Loop(ele);
	
	Angle_Rate_Loop(ele);
	
  Avcs(ele,&adis[0]);//锁尾程序
}

void Heli_Attitude_control(sHeli *ele)
{
	switch(ele->heli_type)
	{
		case Flybar:
			ele->Att_Loop_count = (ele->Att_Loop_count+1)>50000? 1000 : ele->Att_Loop_count+1;
			ele->Pos_Loop_count = 0;
		  ele->loop_count = ele->Att_Loop_count;
		  if (ele->Att_Loop_count == 1)
				ele->yaw_command = ahrs[1].Ang[2];
			Flybar_DecoupingMatrix(ele,rc);//解耦程序,为了采集中位点
			Attitude_Control_Step(ele);
		  Flybar_CoupingMatrix(ele);
			Flybar_servos_widths_output(ele);//最终舵机输出
			break;
		case Flybarless:
			ele->Att_Loop_count = (ele->Att_Loop_count+1)>50000? 1000 : ele->Att_Loop_count+1;
			ele->Pos_Loop_count = 0;
			ele->loop_count = ele->Att_Loop_count;
		  if (ele->Att_Loop_count == 1)
				ele->yaw_command = ahrs[1].Ang[2];
			Flybarless_H1_DecoupingMatrix(ele,rc);//解耦程序
			Attitude_Control_Step(ele);
			Flybarless_CoupingMatrix(ele);
			Flybarless_servos_widths_output(ele);//最终舵机输出
			break;
		default:
			break;
	}
}
