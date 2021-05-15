#include "motor.h"

/***************************************************\
���ܣ�
  ������������PWM����
˵����
  1��5ms��ʱ���ڣ����е������
  2��4��ʹ��
\***************************************************/

sMOT mot[MOT_NUM];
int SPEED=0;
static int rotate_speed_last = 10500;
static int countnum = 0;
static int temp_rotate = 10500;
static int ch = 0;
int show_temp_rotate = 0;
/******************���ܺ���****************/
void Mot_Init_Para(sMOT *ele,sMOT_CFG *elecfg)
{
	ele->htimA = elecfg->htimA;
	ele->htimB = elecfg->htimB;
	ele->name = elecfg->name;
	
	ele->Update = true;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	
	for(u8 i=0;i<PWM_OUT_NUM;i++)
	{
		ele->PWM[i] = INI_PWM;
		ele->PwmOff[i] = elecfg->PwmOff[i];
	}
	ele->set_speed = 1500.0f;
	ele->gear_ratio = 7;
	ele->lim_speed = 1600.0f;
}

bool Mot_Init(sMOT *ele)
{
	Dprintf("\r\n%s Init...\r\n",ele->name);
	HAL_TIM_Base_Start(ele->htimA); //PWM
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_1,ele->PWM[0]);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_2,ele->PWM[1]);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_3,ele->PWM[2]);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_4,ele->PWM[3]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_4,ele->PWM[4]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_3,ele->PWM[5]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_2,ele->PWM[6]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_1,ele->PWM[7]);
	HAL_TIM_PWM_Start(ele->htimA,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(ele->htimA,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(ele->htimA,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(ele->htimA,TIM_CHANNEL_4);
	HAL_TIM_Base_Start(ele->htimB); //PWM
	HAL_TIM_PWM_Start(ele->htimB,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(ele->htimB,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(ele->htimB,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(ele->htimB,TIM_CHANNEL_4);
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

void Mot_Ctr(sMOT *ele)  
{
	if(rc->Pack < 10 || rc->Mode == RC_MOT_LOCK)ele->UnLock = false;
	if(rc->Mode == RC_MOT_UNLOCK)ele->UnLock = true;
	if(ele->UnLock == false)    //�������
	{
		for(u8 i=0;i<PWM_OUT_NUM;i++)
			ele->PWM[i]=INI_PWM;
	}  
	rc->Mode = RC_NONE;
	
	/*ת�ٻ�ȡ*/
	rotate_get(ele,exitx);
	
	//��������Ϊ��ֵ
	
	if(Heli.motor_mode == 0) //����ģʽ������ֱ����ң��������
	{  
		  mot->pwm_temp = Heli.output_servo_width[2];
		  mot->pwm_throttle_base = Heli.output_servo_width[2];
		  Pid_Reset(pidmotor);
			if((rc->Key[3] == 1) && (Heli.output_servo_width[2] <= ( rc->LOW_THR - (rc->LOW_THR - rc->HIG_THR) * 0.40f)))                
				{ //����Ҳ࿪�ش��ϣ��������Ź���20%����ô�ӷǶ�����Ϊ����
				Heli.motor_mode = 2 ; //  ����ģʽ
				countnum = 0;
	    }
	}
	else if (Heli.motor_mode == 2)  // ����ģʽ�������ɿ��������
	{
		//������PI���ڳ���
//		if(rc->Key[2] == 1){
//			ele->auto_throttle_width_increase = Pid_motor( pidmotor , 10500 , ele->rotate_speed);
//			show_temp_rotate = 10500;
//		}
//		else{
//			if(ch >= 0 && ch < 400){
//				temp_rotate -= 10;
//				ele->auto_throttle_width_increase = Pid_motor( pidmotor , temp_rotate , ele->rotate_speed);
//			}
//			else if(ch >= 400 && ch < 900){
//				temp_rotate += 10;
//				ele->auto_throttle_width_increase = Pid_motor( pidmotor , temp_rotate , ele->rotate_speed);
//			}
//			else if(ch >=900 && ch < 1000){
//				temp_rotate -= 10;
//				ele->auto_throttle_width_increase = Pid_motor( pidmotor , temp_rotate , ele->rotate_speed);
//			}
//			else if(ch == 1000){
//				ch = 0;
//				temp_rotate = 10500;
//			}
//			ch++;
//			show_temp_rotate = temp_rotate;
//		}

		//ԭ��PI����
		/*
			2020.12.23�����
			Ŀ�ģ�Ϊ���˳��ڶ��ٺ�ķ���ת���е�ͻ��ת�ٵ�
			�˳����ȣ��������뱾��֮��Ĳ�ֵ����900ʱ����Ϊ���εõ���ele->rotate_speed�Ǵ�㣬��������������ת�ٱ�Ϊrotate_speed_last
		*/
		
		if(countnum > 300){//�ڽ��붨�ٺ�3s֮��Ž����˲�
			if((rotate_speed_last - ele->rotate_speed > 900) || (rotate_speed_last - ele->rotate_speed < -900)){
				ele->rotate_speed = rotate_speed_last;
			}
		}
		if(countnum < 500){//�˶�ʹ��countnum�ӵ�500�Ժ��ٱ仯
			countnum++;
		}
		rotate_speed_last = ele->rotate_speed;
		
		ele->auto_throttle_width_increase = Pid_motor( pidmotor , ele->set_speed  * ele->gear_ratio , ele->rotate_speed);
		mot->pwm_temp =  mot->pwm_throttle_base - ele->auto_throttle_width_increase;
		if(Heli.output_servo_width[2]  > ( rc->LOW_THR - (rc->LOW_THR - rc->HIG_THR) * 0.40f  ))   //�ڶ���ģʽʱ�����������»ص��л�ʱ��������ֵ���л�Ϊ����
		{
			Heli.motor_mode = 0;
		}
	}
	
	
	//��ת����Ϊ��ֵ
//	if(Heli.motor_mode == 0) //����ģʽ������ֱ����ң��������
//	{  
//		  
//		  mot->pwm_temp = Heli.output_servo_width[2];
//		  mot->pwm_throttle_base = Heli.output_servo_width[2];
//		  Pid_Reset(pidmotor);
//			if(rc->Key[3] == 1                //�ֶ��л�����ģʽ
// 	 //  Heli.output_servo_width[2]  < ( rc->LOW_THR - (rc->LOW_THR - rc->HIG_THR) * 0.2f  ) )  //���Ŵﵽ��ֵ�л� ע�����                            //����ģʽ�����                         
//	      &&(ele->rotate_speed > ele->set_speed* ele->gear_ratio  * 0.5f && ele->rotate_speed < 13000.0f) )    // ת��,С��13000����Ϊû���ź�ʱ��ת�����Ϊ15000
//      	{
//	       	Heli.motor_mode = 2 ; //  ����ģʽ
//	      }
//		
//	}
//	else if(Heli.motor_mode == 1)   //����ģʽ
//	{
//		 mot->pwm_temp = Heli.output_servo_width[2];
//		if (ele->rotate_speed < ele->set_speed* ele->gear_ratio  * 0.4f)
//			  Heli.motor_mode = 0;
//	}
//	else if (Heli.motor_mode == 2)  // ����ģʽ�������ɿ��������
//	{
//		
//		ele->auto_throttle_width_increase = Pid_motor( pidmotor , ele->set_speed  * ele->gear_ratio , ele->rotate_speed) ;
//		mot->pwm_temp =  mot->pwm_throttle_base - ele->auto_throttle_width_increase;
//		if(Heli.output_servo_width[2] > mot->pwm_throttle_base + 20 )   //�ڶ���ģʽʱ�����������»ص��л�ʱ��������ֵ���л�Ϊ����
//		{
//			Heli.motor_mode = 1;
//		}
//	}

		ele->PWM[0] = iConstrain(Heli.output_servo_width[0],800,2200);//Heli.output_servo_width[0] 1364
		ele->PWM[1] = iConstrain(Heli.output_servo_width[1],800,2200); //Heli.output_servo_width[1] 1474
		//ele->PWM[2]=CtrlIO->pwm_temp;
		ele->PWM[2] = iConstrain(mot->pwm_temp,1000,2200);
		ele->PWM[3] = iConstrain(Heli.output_servo_width[3],800,2200);
		ele->PWM[4] = iConstrain(Heli.output_servo_width[4],800,2200);
		ele->PWM[5] = iConstrain(Heli.output_servo_width[5],800,2200);//Heli.output_servo_width[5] 1403
		//ele->PWM[5]=CtrlIO->pwm_temp;
		ele->PWM[6] = iConstrain(Heli.output_servo_width[6],800,2200);
		ele->PWM[7] = iConstrain(Heli.output_servo_width[7],800,2200);

//	for(int i=0;i<8;i++)
//	ele->PWM[i]=hl;
//	ele->PWM[0] = iConstrain(1500,800,2200);
//	ele->PWM[1] = iConstrain(1500,800,2200);
//	//ele->PWM[2]=CtrlIO->pwm_temp;
//	ele->PWM[2] = iConstrain(1500,800,2200);
//	ele->PWM[3] = iConstrain(1500,800,2200);
//	ele->PWM[4] = iConstrain(1500,800,2200);
//	ele->PWM[5] = iConstrain(1500,800,2200);
//	//ele->PWM[5]=CtrlIO->pwm_temp;
//	ele->PWM[6] = iConstrain(1500,800,2200);
//	ele->PWM[7] = iConstrain(1500,800,2200);
	
	
	for(u8 i=0;i<PWM_OUT_NUM;i++)  //�޷�
		ele->PWM_OBS[i]=ele->PWM[i]=iConstrain(ele->PWM[i]-ele->PwmOff[i],Min_PWM_Out,Max_PWM_Out);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_1,ele->PWM[0]);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_2,ele->PWM[1]);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_3,ele->PWM[2]);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_4,ele->PWM[3]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_4,ele->PWM[4]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_3,ele->PWM[5]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_2,ele->PWM[6]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_1,ele->PWM[7]);
}




void LowPassFilt_mot(sMOT *ele,float filt_para)
{
	//if(ele->rotate_raw > 13000 ||ele->rotate_raw < 200) ele->rotate_raw = ele->rotate_speed;
	ele->rotate_speed += filt_para * (ele->rotate_raw - ele->rotate_speed);
	
}


float rotate_get(sMOT *ele1,sEXIT *ele2 )

{
	float D_Rev_Speed;
	float Pulse_count;
	float filt_para = 0.324f;//10hz
	static bool first_flag = false;  //���ν���
	
//	static float test_count = 0.0f;
//	int test_pulse_mid = 5714;
//	if(test_count == 0){
//		Pulse_count = test_pulse_mid;
//	}
//	else if(test_count > 0 && test_count < 1000){
//		Pulse_count -= 10;
//	}
//	else if(test_count >= 1000 && test_count < 2000){
//		Pulse_count += 10;
//	}
//	else if(test_count == 2000){
//		test_count = -1;
//	}
//	test_count++;
	
	Pulse_count=ele2->DatRaw[1];
	ele2->RxFlag[1] = false;
	
	if(Pulse_count>65000)
	{
	  Pulse_count=65000;
	}else if(Pulse_count<4000)
	{
		Pulse_count=4000;
	}
	ele1->rotate_raw=(60*1000000/Pulse_count);//����ʵ��ת��
  
	
	
	
	LowPassFilt_mot(ele1, filt_para);
	

	
	ele1->last_rotate_speed=ele1->rotate_speed;
	
   return ele1->rotate_speed;    //7Ϊ���ֱ�
 } 

// float setpoint_get(u16 Throttle_rc)
// {
//	 //�涨  thrֵ�ߵĻ����˵ͣ���֮Ϊ�ߣ�Ϩ�𿪹�ʹ��ʱ��ֵ��͡�
//	 float rota_speed_set;
//	       //
//	       if(Throttle_rc < 1800)
//          					  
// }