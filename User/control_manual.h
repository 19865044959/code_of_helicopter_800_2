#ifndef _CONTROL_MANUAL_H
#define _CONTROL_MANUAL_H

#include "motor.h"
#include "helicopter.h"
#include "rc.h"
#include "userlib.h"
#include "pid.h"
#include "adis16365.h"

#define COLL_PWM_INIT 				1200
#define AILERON_PWM_MID 			1500
#define ELEVATOR_PWM_MID 			1500
//#define YAW_PWM_MID  					1500 
#define OUTPUT_LIMIT 					500
//#define GAIN_REMOTE_TO_RATE 	-0.00864f
#define GAIN_REMOTE_TO_RATE 	-0.00944f
#define RATE_DECREASEMENT 		2.0f    
#define RATE_INCREASEMENT 		2.0f 
#define LIMIT_RATE   					5.236f
#define ALPHA 								1.0f 
#define ZERO_COLL_PWM 				478
#define MID_ROTATE            10500
#define YAW_DIRECTION  				- 

extern float desired_p,desired_q;
extern float test_kp,test_ki,test_target_rate,test_coll;
//extern float CR,BG,HG,HD;
void Heli_Manual_control(sHeli *ele);
void Avcs(sHeli *ele,sADIS *ahrs);
void FBL(sHeli *ele,sADIS *ahrs);
#endif
