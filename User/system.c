#include "system.h"

/***************************************************\
功能：
  系统 多线程布置
说明：
  1、初始化需要一定时间，若有看门狗，需要在初始化之后再开启
  2、Sys_Get线程进行输入设备数据获取与计算，如传感器、控制指令
  3、Sys_Spr线程进行备份输入设备数据获取与计算
  4、Sys_Ctr线程进行输出设备输出计算与控制，如执行器、回传信息
  5、Sys_Chk线程进行系统状态检测与决策，LED或蜂鸣器等显示状态
  6、Sys_1ms线程进行1ms运行，执行快速动作或状态机切换等
\***************************************************/

sTIM Sys_Time[5];
sTIM Deta_Time[5];
u32 Time[5];
bool sync=false;

/******************驱动程序****************/
void TIM7_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET)
  {
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
	  HAL_IncTick();
  }
}

/******************功能函数****************/
void Sys_Init(void)
{
	HAL_Delay(500);
	Dprintf("\r\n**********Welcome to Helicopter world**********\r\n");
	//---------传感器初始化----------//
	Para_Init(para);
	Bell_Init(bell);
	Imu_Init(imu);
	Adis_Init(adis);
  Gps_Init(gps);//未安装GPS屏蔽掉
	Ms5611_Init(&ms5611[0]);
//Ms5611_Init(&ms5611[1]);//MS5611板子未安装，所以屏蔽掉
	Cmd_Init(cmd);
	Rc_Init(rc);
	Ahrs_Init(&ahrs[0]);
	Ahrs_Init(&ahrs[1]);
  Exit_Init(exitx);//未检测转速，屏蔽掉
	Tran_Init(tran);
	Mot_Init(mot);
	HAL_TIM_Base_Start_IT(&htim5);
	Bell_Start(bell);
	//---------传感器初始化结束--------//
	Heli_Init(&Heli);//直升机初始化
	Dprintf("\r\n**********INITIAL SUCCESS**********\r\n");
}

//5ms周期，传感器获取，在freeRTOS.c修改
void Sys_Get(void)
{  static int i=0;
	i++;
	Tim_Calc(&Deta_Time[0]);
	Tim_Calc(&Sys_Time[0]);
	Adis_Calc(adis);//ADIS传感器数据采集(加速度，角速度)
	//Dprintf("%d,%.2f,%.2f,\r\n",i,adis->AccFil_3nd[2],adis->AccRel[2]);
	//Dprintf("%.2f,\r\n",adis->AccRel[2]);
	Imu_Calc(imu);//MPU6050传感器数据采集(加速度，角速度)
	Ms5611_Calc(&ms5611[0]);//MS5611气压计数据采集
//Ms5611_Calc(&ms5611[1]);//MS5611板子未安装，所以屏蔽掉
  Gps_Calc(gps);//GPS数据采集以及磁力计信息
	Cmd_Calc(cmd);
	Bell_Calc(bell);//蜂鸣器数据采集
	Tran_Calc(tran);//3730下传控制函数
	Ahrs_Calc(&ahrs[0]);//ADIS解算姿态角(互补滤波器)
	Ahrs_Calc_EKF(&ahrs[1]);//ADIS解算姿态角(FKF)
	Exit_Calc(exitx);//激光雷达高度
	Tim_Calc(&Sys_Time[0]);
	Time[0] = Sys_Time[0].OUT;
}

//20ms周期１，备用传感器获取，在freeRTOS.c修改
void Sys_Spr(void)
{
	Tim_Calc(&Deta_Time[1]);
	Tim_Calc(&Sys_Time[1]);
	Rc_Calc(rc);//遥控器数据采集
	PCM_Fail_Safe(rc);
	Tim_Calc(&Sys_Time[1]);
	Time[1] = Sys_Time[1].OUT;
}

//10ms周期，执行器控制，在freeRTOS.c修改
void Sys_Run(void)
{
	Tim_Calc(&Deta_Time[2]);
	Tim_Calc(&Sys_Time[2]);
	GPS_INS_EKF(gps);//9阶EKF 位置、速度、加速度偏置
	LLH2NED(gps);//LLH坐标系转NED坐标系
	update_flight_mode();//控制模式
	Sd_Calc(sd);//SD卡
	Mot_Ctr(mot);//输出舵机控制
	Ms5611_Ctr(ms5611);//气压计温度控制
  //Fprintf("%d$",CtrlIO->control_mode,"%d\r\n",Time[2]);
	Tim_Calc(&Sys_Time[2]);
	Time[2] = Sys_Time[2].OUT;	
}

//1ms周期，特殊时序控制，在freeRTOS.c修改
void Sys_1ms(void)
{
	Tim_Calc(&Deta_Time[3]);
	Tim_Calc(&Sys_Time[3]);
	Bell_Loop_1ms(bell);
	//Tran_Loop_1ms(tran);
	Tim_Calc(&Sys_Time[3]);
	Time[3] = Sys_Time[3].OUT;
}

//50ms周期，系统检测，在freeRTOS.c修改
void Sys_Chk(void)
{
  Tim_Calc(&Deta_Time[4]);
	Tim_Calc(&Sys_Time[4]);
	output2ground();//20hz下传
	Led_Tog(1);
	if(adis->Err)Led_Set(2);
	else Led_Clr(2);
	if(imu->Err)Led_Set(3);
	else Led_Clr(3);
	if(gps->Err)Led_Set(4);
	else Led_Clr(4);
	u8 ErrFlag=0;
	if(adis->Err)ErrFlag+=0x01;
	if(imu->Err)ErrFlag+=0x02;
	if(ms5611[0].Err)ErrFlag+=0x04;
//	if(ms5611[1].Err)ErrFlag+=0x08;
//	if(gps->MagErr)ErrFlag+=0x10;
//	if(exitx->Err)ErrFlag+=0x40;
//	if(bell->Err)ErrFlag+=80;
	if(ErrFlag)
	{
		Bell_Error(bell);
		DMAprintf("%2X\r\n",ErrFlag);
	}
	adis->Err=ERR_NONE;
	imu->Err=ERR_NONE;
	ms5611[0].Err=ERR_NONE;
	ms5611[1].Err=ERR_NONE;
	rc->Err=ERR_NONE;
	gps->MagErr=ERR_NONE;
	exitx->Err[0]=ERR_NONE;
	exitx->Err[1]=ERR_NONE;
	exitx->Err[2]=ERR_NONE;
	bell->Err=ERR_NONE;
	Tim_Calc(&Sys_Time[4]);
	Time[4] = Sys_Time[4].OUT;
}
