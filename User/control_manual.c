#include "control_manual.h"
/***************************************************\
功能：
  直升机手控模式
说明：
  1、分为两种直升机机型 1、有副翼 2、有副翼
	2、包含无副翼系统FBL()与锁尾Avcs()
\***************************************************/
float desired_p,desired_q;
//float CR = 5.0f;
//float BG = 0.2f;
//float HG = 0.23f;
//float HD = 1.25f;

void FBL(sHeli *ele,sADIS *ahrs)
{
	
	static float CR = 4.0f;//5 4
	static float BG = 0.14f;//0.2 0.14
	static float HG = 0.13f;//0.4 0.13
	static float HD = 1.65f;//1.5  1.45
	static float HRP = 0;
	static float HPP = 0;
	
	static float p_r_bias = 0;//roll rate bias
	static float q_r_bias = 0;//pitch rate bias
//	static float roll_manual_mid,pitch_manual_mid;    
	
//	CR = pidPit1->Kp;
//	BG = pidPitRate1->Ki;
//	HG = pidPitRate1->Kp;
//	HD = pidVelX->Kp;
	
	if ((ele->Mau_Loop_count < 250) && (ele->Mau_Loop_count >= 50))
	{
		p_r_bias += adis->GyrFil[0];
		q_r_bias += adis->GyrFil[1];
//		roll_manual_mid += ele->auto_servo_width[0];
//		pitch_manual_mid += ele->auto_servo_width[1];
	}
	else if(ele->Mau_Loop_count < 251)
	{ 
		p_r_bias /= 200.0f;
		q_r_bias /= 200.0f;
//		roll_manual_mid/=200.0f;
//		pitch_manual_mid/=200.0f;
	}
	else if(ele->Mau_Loop_count < 257)
	{     
		ele->auto_servo_width[5]= COLL_PWM_INIT + 50;    
	}
	else if(ele->Mau_Loop_count < 277)//213
	{
		ele->auto_servo_width[5]= COLL_PWM_INIT - 50;
	}
	else if(ele->Mau_Loop_count < 297)//222
	{      
		ele->auto_servo_width[5]= COLL_PWM_INIT + 50;   
	}
	else if(ele->Mau_Loop_count < 317)//231
	{
		ele->auto_servo_width[5]= COLL_PWM_INIT - 50; 
	}
	else
	{
		#define AILERON_DIRECTION -
		#define ELEVATOR_DIRECTION +    
		
		float aileron_input,elevator_input;
		float aileron_output,elevator_output;
			
		aileron_input = fabs(AILERON_DIRECTION(ele->auto_servo_width[0] - AILERON_PWM_MID))>10?AILERON_DIRECTION(ele->auto_servo_width[0]-AILERON_PWM_MID):0;       
		elevator_input = fabs(ELEVATOR_DIRECTION(ele->auto_servo_width[1]-ELEVATOR_PWM_MID))>10?ELEVATOR_DIRECTION(ele->auto_servo_width[1]-ELEVATOR_PWM_MID):0;    
											
		desired_p = aileron_input / 500.0f * CR;            
		desired_q = elevator_input / 500.0f * CR;           

		HRP -= HRP * HD / (CONTROL_HZ);    
		HPP -= HPP * HD / (CONTROL_HZ);    
		HRP += (desired_p - (adis->GyrFil[0]- p_r_bias))*(180/PI/(CONTROL_HZ)*500/CR);          
		HPP += (desired_q - (adis->GyrFil[1]- q_r_bias))*(180/PI/(CONTROL_HZ)*500/CR);          

		aileron_output = HG*HRP + BG*aileron_input;   
		elevator_output = HG*HPP + BG*elevator_input;   
		 
		ele->auto_servo_width[0] = iConstrain(-aileron_output,-OUTPUT_LIMIT,OUTPUT_LIMIT) + AILERON_PWM_MID;
		ele->auto_servo_width[1] = iConstrain(elevator_output,-OUTPUT_LIMIT,OUTPUT_LIMIT) + ELEVATOR_PWM_MID;
	}
}

//#define _avcs_tuning_

float test_kp,test_ki,test_target_rate,test_coll;

void Avcs(sHeli *ele,sADIS *adis)
{	
	/*
	70 50 无风较好
	70 100 有风较好
	*/
	#define init_gain_P 90.0f
	#define init_gain_I 70.0f
	#define init_gain_D 0.0f
	
	#ifndef _avcs_tuning_
		#define gain_P   init_gain_P  
		#define gain_I   init_gain_I      
		#define gain_D   init_gain_D 
	#endif

  #ifdef _avcs_tuning_ 
    static float gain_P = init_gain_P;    
    static float gain_I = init_gain_I;     
    static float gain_D = init_gain_D;     
  #endif
  static float alp = 0.0f;
  static float bet = 0.0f;  	
  static float yaw_manual_mid;               
  static float r_bias;                
  static float PID_I_component = 0.0f;    
  static bool flag_breach_limit = 0.0f;
  static float rate_error=0.0f;
	static float _rate_error=0.0f;
	float raw_target_rate=0.0f;
	static float target_rate = 0.0f;
	float rate_error_for_I;
	float d_rate_error;
	int coll_PWM_inc;
	int rotate_inc;
	float coll_yaw_compensation_gain = 0.0f;
	float rotate_yaw_compensation_gain = 0.0f;
	int yaw_offset;
	int yaw_rotate_offset;
	float yaw_out;
//	rotate_yaw_compensation_gain = pidVelX->Kp ;
	if ((ele->Mau_Loop_count < 250) && (ele->Mau_Loop_count >= 250))
	{
		yaw_manual_mid += ele->auto_servo_width[3];
		r_bias += adis->GyrFil[2];
	}
	else if(ele->Mau_Loop_count < 251)
	{ 
		yaw_manual_mid = yaw_manual_mid/200.0f;
		r_bias = r_bias/200.0f;
	}
	else if(ele->Mau_Loop_count < 257)
	{
		if(yaw_manual_mid>1550 || yaw_manual_mid<1450)
					yaw_manual_mid = 1500;
	}
	else if(ele->Mau_Loop_count < 277)
	{
		ele->auto_servo_width[3] = yaw_manual_mid + 50;    
	}
	else if(ele->Mau_Loop_count < 297)
	{
		ele->auto_servo_width[3] = yaw_manual_mid - 50;
	}
	else 
	{
		int target_pwm=0;
		if(Heli.control_mode == MODE_MANUAL){
			target_pwm = ele->auto_servo_width[3] - yaw_manual_mid;
		}
		else{
			target_pwm = ele->auto_servo_width[3] - ele->pwm_yaw_mid;
		}
		
		if(ele->auto_servo_width[4]> 1500)
		{
			if(ele->control_mode == MODE_MANUAL)
			{
				if(target_pwm > 30)//23
					target_pwm = target_pwm - 30;
				else if(target_pwm < -30) 
					target_pwm = target_pwm + 30;
				else target_pwm = 0;
			}
		}
		if((ele->control_mode == MODE_MANUAL) || (ele->control_mode == MODE_ATTITUDE))
		{
			raw_target_rate = target_pwm * GAIN_REMOTE_TO_RATE;
		}
		else
		{
			raw_target_rate = target_pwm * GAIN_REMOTE_TO_RATE;
		}	
		if(fabs(raw_target_rate)>=fabs(target_rate))
		{
			float e_tr = raw_target_rate  - target_rate;
			if(e_tr > RATE_DECREASEMENT)
				target_rate += RATE_DECREASEMENT;
			else if(e_tr < -RATE_DECREASEMENT)
				target_rate -= RATE_DECREASEMENT; 
			else 
				target_rate += e_tr;
		}
		else
		{
			float e_tr = raw_target_rate  - target_rate;     
			if (e_tr > RATE_INCREASEMENT)
				target_rate += RATE_INCREASEMENT;
			else if(e_tr < -RATE_INCREASEMENT)
				target_rate -= RATE_INCREASEMENT; 
			else 
				target_rate += e_tr;
		}
		if (target_rate > LIMIT_RATE) 
			target_rate = LIMIT_RATE;   
		if (target_rate < -LIMIT_RATE) 
			target_rate = -LIMIT_RATE; 
			
		//yaw_rate_target = target_rate;
		test_target_rate = target_rate;
		//rate_error = (target_rate - (adis->GyrFil[2] - r_bias));
		rate_error = (target_rate - (adis->GyrFil_2nd[2] - r_bias));
		rate_error_for_I = rate_error * CONTROL_DT; 
		d_rate_error = (rate_error - _rate_error)*ALPHA;
		rate_error = d_rate_error + _rate_error;
		//d_rate_error = d_rate_error/0.02f;            
		_rate_error = rate_error;      

 //coll 补偿		
		coll_PWM_inc = ZERO_COLL_PWM - ele->auto_servo_width[5];
		coll_yaw_compensation_gain = -0.53f;//out_loop_Y_PID[0]
		yaw_offset = coll_yaw_compensation_gain * coll_PWM_inc;
	//	test_coll = yaw_offset;
		
//rotate 补偿  rc_yaw变小，往正方向旋转
		if(Heli.motor_mode == 2)  //仅当定速的时候加转速补偿
		{   
			  if(mot->rotate_speed < 13000.0f && mot->rotate_speed > 8000.0f)
				{
					rotate_inc = MID_ROTATE - mot->rotate_speed;
					rotate_yaw_compensation_gain = 0.0f; //2020.12.29 省去rotate补偿
					
					yaw_rotate_offset = rotate_yaw_compensation_gain * rotate_inc;
					
					if(yaw_rotate_offset > 40 )
						yaw_rotate_offset = 40;
					else if(yaw_rotate_offset < -40)
						yaw_rotate_offset = -40;
				}
				else yaw_rotate_offset = 0;
		}
		else yaw_rotate_offset = 0;
	  test_coll = yaw_rotate_offset;
		//yaw_offset = 0.0;
		
//			gain_P = pidAcc_z->Kp;
//			gain_I = pidAcc_z->Ki;
/****************************************************************************/	        
		if((ele->auto_servo_width[4]) <= 1500)
		{
			#ifdef _avcs_tuning_
				gain_P = 0.3240f * (1500 - ele->auto_servo_width[4]);
			#endif              
			PID_I_component = 0.0;
			flag_breach_limit = 0;
			
			yaw_out = rate_error * gain_P+ yaw_offset;           
			if(yaw_out > OUTPUT_LIMIT) 
			{    
					yaw_out = OUTPUT_LIMIT;
			}
			else if(yaw_out < -OUTPUT_LIMIT)
			{
					yaw_out = -OUTPUT_LIMIT;
			}
			if(ele->control_mode == MODE_MANUAL){
					ele->auto_servo_width[3] = yaw_manual_mid + YAW_DIRECTION(yaw_out);
			}
			else{
					ele->auto_servo_width[3] = ele->pwm_yaw_mid + YAW_DIRECTION(yaw_out);
			}
			
		}
		else
		{
			#ifdef _avcs_tuning_
				gain_I = 0.3240f * (ele->auto_servo_width[4] - 1500);
			#endif              
/************************????PI*****************************************/        
			float GS_P;
			float GS_I;  
			float GS_F;

			alp  = 0.2f;
			bet  = 0.2f;

			GS_P = fabs(rate_error)/3.14f;
			if (GS_P > 1.0f) GS_P = 1.0f;
			GS_P = 1.0f - alp*GS_P;
			GS_P = 1.0f;
			
			GS_I = fabs(rate_error_for_I)/3.14f;
			if (GS_I > 1.0f) GS_I = 1.0f;
			GS_I = 1.0f - bet*GS_I; 
			GS_I = 1.0f;
			
			GS_F = abs(coll_PWM_inc)/300.0f;
			if(GS_F > 1.0f) GS_F = 1.0f;

			GS_F = 1.0f;
			yaw_offset = GS_F * yaw_offset;           
			
			float I_increment;
			I_increment = GS_I*gain_I * rate_error_for_I;
			//如果没限幅，那么无论怎样变化，都增加到PID_I_component里面去
			//如果有限幅，只能往PID_I_component小的方向变化
			if(((!flag_breach_limit) || (PID_I_component>0&&I_increment<0) || (PID_I_component<0&&I_increment>0)))
			{
					PID_I_component += I_increment;
			}
			yaw_out = GS_P*gain_P * rate_error + PID_I_component+ yaw_offset + yaw_rotate_offset;// + coll_rate *in_loop_yaw_PID[1] + gain_D*d_rate_error;//*2.0*(1.0+abs(rate_error/6.0))        
			if(yaw_out > OUTPUT_LIMIT) 
			{
					flag_breach_limit = 1;
					yaw_out = OUTPUT_LIMIT;
			}
			else if(yaw_out < -OUTPUT_LIMIT)
			{
					flag_breach_limit = 1;
					yaw_out = -OUTPUT_LIMIT;
			} 
			else
			{
					flag_breach_limit = 0;  
			}       
			if(ele->control_mode == MODE_MANUAL){
					ele->auto_servo_width[3] = yaw_manual_mid + YAW_DIRECTION(yaw_out);
			}
			else{
					ele->auto_servo_width[3] = ele->pwm_yaw_mid + YAW_DIRECTION(yaw_out);
			}
		}
		test_kp = gain_P;
		test_ki = gain_I;
		test_target_rate = target_rate;
	}
}

void Heli_Manual_control(sHeli *ele)
{
	switch(ele->heli_type)
	{
		case Flybar:
			ele->Mau_Loop_count = (ele->Mau_Loop_count+1)>50000? 1000 : ele->Mau_Loop_count+1;//循环次数
			ele->Att_Loop_count = 0;
		  ele->Pos_Loop_count = 0;
			ele->loop_count = ele->Mau_Loop_count;
	   	Integral_Init();//控制器积分器清零
			Flybar_DecoupingMatrix(ele,rc);//解耦程序,为了采集中位点
	   	Avcs(ele,&adis[0]);//锁尾程序   2019.4.2
		  Flybar_CoupingMatrix(ele);//加耦程序
			Flybar_servos_widths_output(ele);//最终舵机输出
			break;
		case Flybarless:
			ele->Mau_Loop_count = (ele->Mau_Loop_count+1)>50000? 1000 : ele->Mau_Loop_count+1;//循环次数
		  ele->Att_Loop_count = 0;
		  ele->Pos_Loop_count = 0;
			ele->loop_count = ele->Mau_Loop_count;
			Integral_Init();//控制器积分器清零
		  Flybarless_H1_DecoupingMatrix(ele,rc);//解耦程序
			FBL(ele,&adis[0]);//无副翼电子系统
			Avcs(ele,&adis[0]);//锁尾程序
			Flybarless_CoupingMatrix(ele);//加耦程序
			Flybarless_servos_widths_output(ele);//最终舵机输出
		  
			break;
		default:
			break;
	}
}
