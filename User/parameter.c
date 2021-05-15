#include "parameter.h"

/***************************************************\
功能：
  参数表
说明：
  1、修改几乎所有模块的配置
\***************************************************/

sPARA para[1];

void Para_Init(sPARA *ele)
{
	ele->imucfg[0].hi2c = &hi2c1;
	ele->imucfg[0].name = (char *)"MPU6050";
	ele->imucfg[0].FiltNum = 1;
	ele->imucfg[0].Dir = (char *)"+Y-X+Z";
	ele->imucfg[0].OneG = 9.88f;
	ele->imucfg[0].AccOff[0] = -65;
	ele->imucfg[0].AccOff[1] = 296;
	ele->imucfg[0].AccOff[2] = 716;
	ele->imucfg[0].GyrOffO[0]=64.96f;
	ele->imucfg[0].GyrOffO[1]=35.18f;
	ele->imucfg[0].GyrOffO[2]= 9.18f;
	ele->imucfg[0].GyrOffT[0]=0.0f;
	ele->imucfg[0].GyrOffT[1]=0.0f;
	ele->imucfg[0].GyrOffT[2]=0.0f;
	Imu_Init_Para(imu,ele->imucfg);
	     
	ele->adiscfg[0].hspi = &hspi2;
	ele->adiscfg[0].name = (char *)"ADIS";
	ele->adiscfg[0].FiltNum = 1;
	ele->adiscfg[0].Dir = "-X+Y-Z";   
	ele->adiscfg[0].AccOff[0] = 0;
	ele->adiscfg[0].AccOff[1] = 0;
	ele->adiscfg[0].AccOff[2] = 0;
	ele->adiscfg[0].GyrOff[0] = 0;
	ele->adiscfg[0].GyrOff[1] = 0;
	ele->adiscfg[0].GyrOff[2] = 0;
	ele->adiscfg[0].OneG = 9.80f;
	Adis_Init_Para(adis,ele->adiscfg);
	
	ele->gpscfg[0].huart = &huart1;
	ele->gpscfg[0].name = (char *)"GPS";
	ele->gpscfg[0].CheckNum = 40;       //200ms
	ele->gpscfg[0].Dir = (char *)"-X+Y-Z";  //左手坐标系
	ele->gpscfg[0].MagOff[0] =     1929.716626499573f; //2020.11.01校正完毕，注意：要不就直接传MagRaw，然后就可以求出bias后减去bias即可；要不直接传MagRel，然后在Rel处减bias也行
	ele->gpscfg[0].MagOff[1]  =     -2619.665995391379f;
	ele->gpscfg[0].MagOff[2]  =      -586.3868853260857f;

	Gps_Init_Para(gps,ele->gpscfg);
	
	ele->ms5611cfg[0].hspi = &hspi1;
	ele->ms5611cfg[0].name = (char *)"MS5611[0]";
	ele->ms5611cfg[0].FiltNum = 3;       
	ele->ms5611cfg[0].Delay = 0;     
	Ms5611_Init_Para(&ms5611[0],&ele->ms5611cfg[0]);
	
	ele->ms5611cfg[1].hspi = &hspi4;
	ele->ms5611cfg[1].name = (char *)"MS5611[1]";
	ele->ms5611cfg[1].FiltNum = 3;   
	ele->ms5611cfg[1].Delay = 10;      
	Ms5611_Init_Para(&ms5611[1],&ele->ms5611cfg[1]);
	
	ele->ahrscfg[0].name = (char *)"AHRS[0]";
	ele->ahrscfg[0].AngOff[0] = 0.0f;   //往左飘增大该系数
	ele->ahrscfg[0].AngOff[1] = 0.0f;   //往前飘增大该系数
	ele->ahrscfg[0].AngOff[2] = 0.0f;
	ele->ahrscfg[0].halfDt = 0.0025f;
	ele->ahrscfg[0].mode = FLY_HEL;     //直升机模式
	Ahrs_Init_Para(&ahrs[0],&ele->ahrscfg[0]);
	
	ele->ahrscfg[1].name = (char *)"AHRS[1]";
	ele->ahrscfg[1].AngOff[0] = 0.0f;   //往左飘增大该系数
	ele->ahrscfg[1].AngOff[1] = 0.0f;   //往前飘增大该系数
	ele->ahrscfg[1].AngOff[2] = 0.0f;
	ele->ahrscfg[1].halfDt = 0.0025f;
	ele->ahrscfg[1].mode = FLY_HEL;     //直升机模式
	Ahrs_Init_Para(&ahrs[1],&ele->ahrscfg[1]);
	
	ele->motcfg[0].htimA = &htim4;  //左4 中1 右5
	ele->motcfg[0].htimB = &htim3;
	ele->motcfg[0].name = (char *)"MOTOR";
	ele->motcfg[0].PwmOff[0] = 0;
	ele->motcfg[0].PwmOff[1] = -1;
	ele->motcfg[0].PwmOff[2] = 5;
	ele->motcfg[0].PwmOff[3] = 10;
	ele->motcfg[0].PwmOff[4] = 0;
	ele->motcfg[0].PwmOff[5] = 0;
	ele->motcfg[0].PwmOff[6] = 0;
	ele->motcfg[0].PwmOff[7] = 0;
	
	Mot_Init_Para(mot,ele->motcfg);
	
	ele->cmdcfg[0].huart = &huart7;
	ele->cmdcfg[0].name = (char *)"CMD";
	ele->cmdcfg[0].CheckNum = 200;           //1s
	Cmd_Init_Para(cmd,ele->cmdcfg);
	
	ele->rccfg[0].huart = &huart4;
	ele->rccfg[0].name = (char *)"RC";
	ele->rccfg[0].CheckNum = 6;           //30ms
	ele->rccfg[0].ModeNum = 60;           //1.5s，连续多少个遥控器数据包
	ele->rccfg[0].LOW_THR = 1903;
	ele->rccfg[0].MID_THR = 1608;
	ele->rccfg[0].HIG_THR = 1360;


	Rc_Init_Para(rc,ele->rccfg);

	ele->exitcfg[0].name = (char *)"EXIT";
	ele->exitcfg[0].CheckNum[0] = 10;         //50ms
	ele->exitcfg[0].CheckNum[1] = 4;         //20ms
	ele->exitcfg[0].CheckNum[2] = 4;         //20ms
	ele->exitcfg[0].FiltNum[0] = 5;
	ele->exitcfg[0].FiltNum[1] = 5;
	ele->exitcfg[0].FiltNum[2] = 5;
	Exit_Init_Para(exitx,ele->exitcfg);
	
	ele->trancfg[0].huart = &huart2;
	ele->trancfg[0].name = (char *)"TRAN";
	Tran_Init_Para(tran,ele->trancfg);
	
	ele->bellcfg[0].hadc = &hadc1;
	ele->bellcfg[0].htim = &htim1;
	ele->bellcfg[0].name = (char *)"BELL";
	ele->bellcfg[0].chn = TIM_CHANNEL_1;
	ele->bellcfg[0].FiltNum = 20;
	Bell_Init_Para(bell,ele->bellcfg);
	
	//roll_rate pid
	ele->pidcfg[0].Kp = 0.03f;
	ele->pidcfg[0].Ki = 0.10f;
	ele->pidcfg[0].Kd = 0.0f;
	ele->pidcfg[0].Kb = 0.0f;
	ele->pidcfg[0].eLimit = 150.0f * D2R;
	ele->pidcfg[0].iLimit = 90.0f * D2R;
	ele->pidcfg[0].dLimit = PI;
	ele->pidcfg[0].filter_para = 0.557f;
	Pid_Init_Para(pidRolRate1,&ele->pidcfg[0]);
	
	//pitch_rate_pid
	ele->pidcfg[1].Kp = 0.03f;
	ele->pidcfg[1].Ki = 0.10f;
	ele->pidcfg[1].Kd = 0.0f;
	ele->pidcfg[1].Kb = 0.0f;
	ele->pidcfg[1].eLimit = 150.0f * D2R;
	ele->pidcfg[1].iLimit = 90.0f * D2R;
	ele->pidcfg[1].dLimit = PI;
	ele->pidcfg[1].filter_para = 0.557f;
	Pid_Init_Para(pidPitRate1,&ele->pidcfg[1]);
	
	//yaw_rate_pid
	ele->pidcfg[2].Kp = 0.03f;
	ele->pidcfg[2].Ki = 0.15f;
	ele->pidcfg[2].Kd = 0.0f;
	ele->pidcfg[2].Kb = 0.0f;
	ele->pidcfg[2].eLimit = 4*PI;
	ele->pidcfg[2].iLimit = PI;
	ele->pidcfg[2].dLimit = PI;
	ele->pidcfg[2].filter_para = 0.557f;
	Pid_Init_Para(pidAcc_z,&ele->pidcfg[2]);
	
	//roll_angle_pid
	ele->pidcfg[3].Kp = 4.00f;
	ele->pidcfg[3].Ki = 0.00f;
	ele->pidcfg[3].Kd = 0.0f;
	ele->pidcfg[3].Kb = 0.0f;
	ele->pidcfg[3].eLimit= 30.0f * D2R;
	ele->pidcfg[3].iLimit = 0.5f*PI;
	ele->pidcfg[3].filter_para = 0.557f;
	Pid_Init_Para(pidRol1,&ele->pidcfg[3]);
	
	//pitch_angle pid
	ele->pidcfg[4].Kp = 4.00f;
	ele->pidcfg[4].Ki = 0.00f;
	ele->pidcfg[4].Kd = 0.0f;
	ele->pidcfg[4].Kb = 0.0f;
	ele->pidcfg[4].eLimit = 30.0f * D2R;
	ele->pidcfg[4].iLimit = 0.5f*PI;
	ele->pidcfg[4].filter_para = 0.557f;
	Pid_Init_Para(pidPit1,&ele->pidcfg[4]);
	
	//yaw_angle pid
	ele->pidcfg[5].Kp = 4.5f;
	ele->pidcfg[5].Ki = 0.0f;
	ele->pidcfg[5].Kd = 0.0f;
	ele->pidcfg[5].Kb = 0.0f;
	ele->pidcfg[5].eLimit = PI;
	ele->pidcfg[5].iLimit = 0.5f*PI;
	ele->pidcfg[5].filter_para = 0.557f;
	Pid_Init_Para(pidYaw1,&ele->pidcfg[5]);
	
	ele->pidcfg[6].Kp = 0.15f;
	ele->pidcfg[6].Ki = 0.02f;
	ele->pidcfg[6].Kd = 0.0f;
	ele->pidcfg[6].Kb = 0.0f;
	ele->pidcfg[6].eLimit = 4*PI;
	ele->pidcfg[6].iLimit = PI;
	ele->pidcfg[6].dLimit = PI;
	Pid_Init_Para(pidVelX,&ele->pidcfg[6]);
	
	ele->pidcfg[7].Kp = 0.25f;
	ele->pidcfg[7].Ki = 0.04f;
	ele->pidcfg[7].Kd = 0.0f;
	ele->pidcfg[7].Kb = 0.0f;
	ele->pidcfg[7].eLimit = 4*PI;
	ele->pidcfg[7].iLimit = PI;
	ele->pidcfg[7].dLimit = PI;
	Pid_Init_Para(pidVelY,&ele->pidcfg[7]);
	
	ele->pidcfg[8].Kp = 0.25f;
	ele->pidcfg[8].Ki = 0.04f;
	ele->pidcfg[8].Kd = 0.0f;
	ele->pidcfg[8].Kb = 0.0f;
	ele->pidcfg[8].eLimit = 4*PI;
	ele->pidcfg[8].iLimit = PI;
	ele->pidcfg[8].dLimit = PI;
	Pid_Init_Para(pidVelZ,&ele->pidcfg[8]);
	
	ele->pidcfg[9].Kp = 0.0f;
	ele->pidcfg[9].Ki = 0.0f;
	ele->pidcfg[9].Kd = 0.0f;
	ele->pidcfg[9].Kb = 0.0f;
	ele->pidcfg[9].iLimit = 20000.0f;
	Pid_Init_Para(pidX,&ele->pidcfg[9]);
	
	ele->pidcfg[10].Kp = 0.0f;
	ele->pidcfg[10].Ki = 0.0f;
	ele->pidcfg[10].Kd = 0.0f;
	ele->pidcfg[10].Kb = 0.0f;
	ele->pidcfg[10].iLimit = 20000.0f;
	Pid_Init_Para(pidY,&ele->pidcfg[10]);
	
	ele->pidcfg[11].Kp = 4.0f;
	ele->pidcfg[11].Ki = 0.0f;
	ele->pidcfg[11].Kd = 0.0f;
	ele->pidcfg[11].Kb = 0.0f;
	ele->pidcfg[11].iLimit = 20000.0f;
	Pid_Init_Para(pidZ,&ele->pidcfg[11]);
	
	ele->pidcfg[12].Kp = 0.12f;
	ele->pidcfg[12].Ki = 0.0f;
	ele->pidcfg[12].Kd = 0.0f;
	ele->pidcfg[12].Kb = 0.0f;
	ele->pidcfg[12].iLimit = 20000.0f;
	Pid_Init_Para(pidRolRate1_FF,&ele->pidcfg[12]);
	
	ele->pidcfg[13].Kp = 0.12f;
	ele->pidcfg[13].Ki = 0.0f;
	ele->pidcfg[13].Kd = 0.0f;
	ele->pidcfg[13].Kb = 0.0f;
	ele->pidcfg[13].iLimit = 20000.0f;
	Pid_Init_Para(pidPitRate1_FF,&ele->pidcfg[13]);
	
	ele->pidcfg[14].Kp = 200.0f;
	ele->pidcfg[14].Ki = 0.07f;
	ele->pidcfg[14].Kd = 20.02f;
	ele->pidcfg[14].filter_para = 0.557f;
	ele->pidcfg[14].setpoint=50.0f;
	Pid_Init_Para(pidTmp,&ele->pidcfg[14]);
	
	ele->pidcfg[15].Kp = 0.2f;
	ele->pidcfg[15].Ki = 0.60f;
	ele->pidcfg[15].Kd = 0.2f;
	ele->pidcfg[15].Kb = 0.5f; //不完全微分PID，微分先行系数，2020.01.13加入
	ele->pidcfg[15].filter_para = 0.557f;
	ele->pidcfg[15].setpoint=50.0f;
	Pid_Init_Para(pidmotor,&ele->pidcfg[15]);
	//-----------------未用到---------------------------------------------------//
	ele->pidcfg[16].Kp = 0.2f;
	ele->pidcfg[16].Ki = 0.60f;
	ele->pidcfg[16].Kd = 0.2f;
	ele->pidcfg[15].Kb = 0.5f; //不完全微分PID，微分先行系数，2020.01.13加入
	ele->pidcfg[16].eLimit = 5000.0f;
	ele->pidcfg[16].iLimit = 9000.0f;
	ele->pidcfg[16].dLimit = 10000.0f;
	ele->pidcfg[16].filter_para = 0.557f;
	
	Pid_Init_Para(pidmotor,&ele->pidcfg[16]);
	
	ele->sdcfg->hsd = &hsd1;
	ele->sdcfg->FileNum = 99;
	ele->sdcfg->name = "SD_CARD";
	Sd_Init_Para(sd,ele->sdcfg);
}

/******************************END OF FILE************************************/

