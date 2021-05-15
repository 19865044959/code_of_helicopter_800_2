#ifndef _CONTROL_STABLIZE_H
#define _CONTROL_STABLIZE_H

#include "helicopter.h"
#include "ahrs.h"
#include "control_manual.h"
#include "pid.h"

#define ROLL_PWM_REMOTE_MID 1380    
#define PITCH_PWM_REMOTE_MID 990    
#define YAW_PWM_REMOTE_MID 1508     
#define trim_roll_angle  2.0f //2.0f
#define trim_pitch_angle -3.0f   //-3.0f
#define RAD2PWM 500.0f
#define ACCEL_ROLL_MAX 1100.0f
#define ACCEL_PITCH_MAX 1100.0f
#define ACCEL_YAW_MAX 270.0f
#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN 360.0f
#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX 720.0f
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN 90.0f
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX 360.0f
#define SMOOTHING_GAIN 7  //7
void Heli_Attitude_control(sHeli *ele);
void Attitude_Control_Step(sHeli *ele);
void Angle_Rate_Loop(sHeli *ele);
void Angle_Loop(sHeli *ele);
void update_rate_bf_targets(sHeli *ele);
void Angle_error_ef_2_bf(sHeli *ele);
void update_ef_roll_pitch_angle_and_error(sHeli *ele);
float sqrt_controller(float error, float p, float second_ord_lim);
void Ang_Rate_Loop_Feedback(sHeli *ele,sAHRS *ahrs);
void Ang_Loop_Feedback(sHeli *ele,sAHRS *ahrs);
void Angle_From_Rc_Or_Outloop(sHeli *ele);
#endif
