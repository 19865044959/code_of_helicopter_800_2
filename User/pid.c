#include "pid.h"

/***************************************************\
功能：
  pid控制算法
说明：
  1、串级PID
\***************************************************/

sPID pid[PID_NUM];
sPID *pidRolRate1=&pid[0],*pidPitRate1=&pid[1],*pidAcc_z=&pid[2];
sPID *pidRol1=&pid[3],*pidPit1=&pid[4],*pidYaw1=&pid[5];
sPID *pidVelX=&pid[6],*pidVelY=&pid[7],*pidVelZ=&pid[8];
sPID *pidX=&pid[9],*pidY=&pid[10],*pidZ=&pid[11];
sPID *pidRolRate1_FF=&pid[12],*pidPitRate1_FF=&pid[13];
sPID *pidTmp=&pid[14];
sPID *pidmotor = &pid[15];
void Pid_Init_Para(sPID *ele,sPID_CFG *elecfg)
{
	ele->Kp = elecfg->Kp;
	ele->Ki = elecfg->Ki;
	ele->Kd = elecfg->Kd;
	ele->Kb = elecfg->Kb;
	ele->eLimit=elecfg->eLimit;
	ele->iLimit = elecfg->iLimit;
	ele->dLimit=elecfg->dLimit;
	ele->integral=0.0f;
	ele->filter_para = elecfg->filter_para;
	ele->setpoint=elecfg->setpoint ;
}

void Low_pass_filter(sPID *ele,float error)
{
	float error_change = (error - ele->lastError) * ele->filter_para;
	ele->error = ele->lastError + error_change;
	ele->lastError = ele->error;
}

double Pid_Controller(sPID *ele)
{
	ele->error=fConstrain(ele->error,-ele->eLimit,ele->eLimit);
	ele->pout = ele->error * ele->Kp;
	ele->integral += ele->error*CONTROL_DT;
	ele->integral = fConstrain(ele->integral,-ele->iLimit,ele->iLimit);
	ele->iout = ele->integral * ele->Ki;
	ele->dout = fConstrain((ele->lastError-ele->error)/(CONTROL_DT),-ele->dLimit,ele->dLimit);
	ele->dout = ele->dout*ele->Kd;
	ele->output = ele->pout + ele->iout + ele->dout;
	return ele->output;
}

float Pid_Position(sPID* ele, float SetXYZ, float FeedBackXYZ)
{
	ele->setpoint = SetXYZ + ele->signal*0.03f;
	ele->feedback = FeedBackXYZ; 
	ele->error = ele->setpoint - ele->feedback;
	ele->pout = ele->error * ele->Kp;
	ele->integral += ele->error;
	ele->integral = fConstrain(ele->integral,-ele->iLimit,ele->iLimit); 
	ele->iout = ele->integral * ele->Ki;
	//不完全微分
	ele->dout = (ele->Kb*(ele->lastfeedback - ele->feedback) + (1.0f - ele->Kb)*ele->dout) * ele->Kd;
	ele->lastfeedback = ele->feedback;
	
	ele->output = ele->pout + ele->iout + ele->dout;
	return ele->output;
}

float Pid_Speed(sPID* ele, float SetSpeed, float FeedBackSpeed)
{
	ele->setpoint = SetSpeed + ele->signal;
	ele->feedback = FeedBackSpeed; 
	ele->error = ele->setpoint - ele->feedback;
	ele->pout = ele->error * ele->Kp;
	ele->integral += ele->error;
	ele->integral = fConstrain(ele->integral,-ele->iLimit,ele->iLimit); 
	ele->iout = ele->integral * ele->Ki;
	//不完全微分
	ele->dout = (ele->Kb*(ele->lastfeedback - ele->feedback) + (1.0f - ele->Kb)*ele->dout) * ele->Kd;
	ele->lastfeedback = ele->feedback;
	
	ele->output = ele->pout + ele->iout + ele->dout;
	return ele->output;
}

//PI，纯P控制，D相当于内环的P
float Pid_Angle(sPID* ele, float SetAngle, float FeedBackAngle)
{
	ele->setpoint = SetAngle + ele->signal;
	ele->feedback = FeedBackAngle; 
	ele->error = LoopConstrain(ele->setpoint - ele->feedback,-180.0f,180.0f);
	ele->pout = ele->error * ele->Kp;
	ele->integral += ele->error;
	ele->integral = fConstrain(ele->integral,-ele->iLimit,ele->iLimit); 
	ele->iout = ele->integral * ele->Ki;
	//不完全微分
	ele->dout = (ele->Kb*(ele->lastfeedback - ele->feedback) + (1.0f - ele->Kb)*ele->dout) * ele->Kd;
	ele->lastfeedback = ele->feedback;
	
	ele->output = ele->pout + ele->iout + ele->dout;
	return ele->output;
}

//PID+微分先行+不完全微分
float Pid_RateAngle(sPID* ele, float SetRateAngle, float FeedBackRateAngle)
{
	ele->setpoint = SetRateAngle + ele->signal;
	ele->feedback = FeedBackRateAngle;
	ele->error = ele->setpoint - ele->feedback;

	ele->pout = ele->error * ele->Kp;
	ele->integral += ele->error;		
	ele->integral = fConstrain(ele->integral,-ele->iLimit,ele->iLimit); 
	ele->iout = ele->integral * ele->Ki; 
	
	//不完全微分
	ele->dout = (ele->Kb*(ele->lastfeedback - ele->feedback) + (1.0f - ele->Kb)*ele->dout) * ele->Kd;
	ele->lastfeedback = ele->feedback;

	ele->output = ele->pout + ele->iout + ele->dout;
	return ele->output;
}

float Pid_PositionSpeed(sPID* ele, float SetPosition, float FeedBackPosition, float FeedBackSpeed)
{
	ele->setpoint = SetPosition + ele->signal;
	ele->feedback = FeedBackPosition;
	ele->error = ele->setpoint - ele->feedback;

	ele->pout = ele->error * ele->Kp;
	ele->integral += ele->error;		
	ele->integral = fConstrain(ele->integral,-ele->iLimit,ele->iLimit); 
	ele->iout = ele->integral * ele->Ki; 
	
	ele->dout = -FeedBackSpeed * ele->Kd;

	ele->output = ele->pout + ele->iout + ele->dout;
	return ele->output;
}

float Pid_motor(sPID* ele, float SetSpeed, float FeedBackSpeed)
{ 
	//2020.11.30
//	ele->Kp = pidAcc_z->Kp;
//	ele->Ki = pidAcc_z->Ki;
	
	//2021.1.13
//	ele->Kp = pidYaw1->Kp;
//	ele->Kd = pidAcc_z->Kp;
	ele->Kb = 0.5f; //为什么在para那里改没用呢？
	
	ele->feedback = FeedBackSpeed; 
	ele->error = SetSpeed - ele->feedback;
	ele->error = fConstrain(ele->error,-ele->eLimit,ele->eLimit); // -5000 ~ 5000
	ele->pout = ele->error * ele->Kp;

	ele->integral += ele->error * 0.01f;  
	ele->integral = fConstrain(ele->integral,-ele->iLimit,ele->iLimit); 
	ele->iout = ele->integral * ele->Ki;
	
	//不完全微分 2021.1.11
	/*
		与纯PID相比，不完全微分考虑了d项带来的对于噪声的敏感，用了一个一阶低通滤波器串联在了d通道上
		与传统PID相比，消除了对噪声的敏感，也拥有传统PID的D的超前校正
		Kb是微分先行系数，范围是(0,1)
		带有D项的PID怎么调：先调PI，加入D，首先把Kd调小些，增大Kp，使得系统有震荡的趋势，再增大Kd，使得系统稳定
		重复，直到增加D没办法使系统稳定，减少Kp，即可
	*/
	
	ele->dout = ele->Kd * (1 - ele->Kb) * (ele->error - ele->lastError) + ele->Kb * ele->dout;
	ele->lastError = ele->error;
	
	ele->output = ele->pout + ele->iout + ele->dout;//
	ele->output =  fConstrain(ele->output,-170,170);    //限幅正负300
	return ele->output;
}
void Pid_Reset(sPID* ele)
{
	ele->integral = 0;
	ele->lastError = 0;
}


void Integral_Init(void)
{
	Pid_Reset(pidRolRate1);
	Pid_Reset(pidPitRate1);
	Pid_Reset(pidRolRate1_FF);
	Pid_Reset(pidPitRate1_FF);
	Pid_Reset(pidVelX);
	Pid_Reset(pidVelY);
	Pid_Reset(pidAcc_z);
}

 float Pid_tmp(sPID *ele)
 {   
  ele->error=ele->setpoint - ele->feedback;   
	if(ele->iout <800.0f&&ele->iout>-800.0f)
	{
		if(ele->error> 5.0f||ele->error<- 5.0f) ele->iout=0;    //积分分离
		else  {
	      		  ele->integral += ele->error;            
		        	ele->iout=ele->Ki*ele->integral;
	         	}
	}
	else if  
		(((ele->iout>800||ele->iout==800)&&ele->error<0)||((ele->iout<-800||ele->iout==-800)&&ele->error>0))
    	{   ele->integral += ele->error;            
		    	ele->iout=ele->Ki*ele->integral;
			}
			
	ele->pout=ele->Kp*ele->error;
	//ele->dout=(ele->Kd*(ele->error-ele->lastError)+ele->filter_para*ele->lastdout)/(0.001+ele->filter_para);  //低通滤波微分公式 
  ele->dout=ele->Kd*(ele->error-ele->lastError);
	ele->output=ele->pout+ele->iout;//+ele->dout;
	//ele->output=ele->pout+ele->iout;
	if(ele->output < 0) ele->output=0;
	ele->lastError=ele->error;
	if(ele->output >999)ele->output=999;
	//ele->lastdout=ele->dout;  //此处为上一微分输出值
   return ele->output;
	
}
		 

