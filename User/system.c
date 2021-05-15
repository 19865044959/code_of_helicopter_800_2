#include "system.h"

/***************************************************\
���ܣ�
  ϵͳ ���̲߳���
˵����
  1����ʼ����Ҫһ��ʱ�䣬���п��Ź�����Ҫ�ڳ�ʼ��֮���ٿ���
  2��Sys_Get�߳̽��������豸���ݻ�ȡ����㣬�紫����������ָ��
  3��Sys_Spr�߳̽��б��������豸���ݻ�ȡ�����
  4��Sys_Ctr�߳̽�������豸�����������ƣ���ִ�������ش���Ϣ
  5��Sys_Chk�߳̽���ϵͳ״̬�������ߣ�LED�����������ʾ״̬
  6��Sys_1ms�߳̽���1ms���У�ִ�п��ٶ�����״̬���л���
\***************************************************/

sTIM Sys_Time[5];
sTIM Deta_Time[5];
u32 Time[5];
bool sync=false;

/******************��������****************/
void TIM7_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET)
  {
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
	  HAL_IncTick();
  }
}

/******************���ܺ���****************/
void Sys_Init(void)
{
	HAL_Delay(500);
	Dprintf("\r\n**********Welcome to Helicopter world**********\r\n");
	//---------��������ʼ��----------//
	Para_Init(para);
	Bell_Init(bell);
	Imu_Init(imu);
	Adis_Init(adis);
  Gps_Init(gps);//δ��װGPS���ε�
	Ms5611_Init(&ms5611[0]);
//Ms5611_Init(&ms5611[1]);//MS5611����δ��װ���������ε�
	Cmd_Init(cmd);
	Rc_Init(rc);
	Ahrs_Init(&ahrs[0]);
	Ahrs_Init(&ahrs[1]);
  Exit_Init(exitx);//δ���ת�٣����ε�
	Tran_Init(tran);
	Mot_Init(mot);
	HAL_TIM_Base_Start_IT(&htim5);
	Bell_Start(bell);
	//---------��������ʼ������--------//
	Heli_Init(&Heli);//ֱ������ʼ��
	Dprintf("\r\n**********INITIAL SUCCESS**********\r\n");
}

//5ms���ڣ���������ȡ����freeRTOS.c�޸�
void Sys_Get(void)
{  static int i=0;
	i++;
	Tim_Calc(&Deta_Time[0]);
	Tim_Calc(&Sys_Time[0]);
	Adis_Calc(adis);//ADIS���������ݲɼ�(���ٶȣ����ٶ�)
	//Dprintf("%d,%.2f,%.2f,\r\n",i,adis->AccFil_3nd[2],adis->AccRel[2]);
	//Dprintf("%.2f,\r\n",adis->AccRel[2]);
	Imu_Calc(imu);//MPU6050���������ݲɼ�(���ٶȣ����ٶ�)
	Ms5611_Calc(&ms5611[0]);//MS5611��ѹ�����ݲɼ�
//Ms5611_Calc(&ms5611[1]);//MS5611����δ��װ���������ε�
  Gps_Calc(gps);//GPS���ݲɼ��Լ���������Ϣ
	Cmd_Calc(cmd);
	Bell_Calc(bell);//���������ݲɼ�
	Tran_Calc(tran);//3730�´����ƺ���
	Ahrs_Calc(&ahrs[0]);//ADIS������̬��(�����˲���)
	Ahrs_Calc_EKF(&ahrs[1]);//ADIS������̬��(FKF)
	Exit_Calc(exitx);//�����״�߶�
	Tim_Calc(&Sys_Time[0]);
	Time[0] = Sys_Time[0].OUT;
}

//20ms���ڣ������ô�������ȡ����freeRTOS.c�޸�
void Sys_Spr(void)
{
	Tim_Calc(&Deta_Time[1]);
	Tim_Calc(&Sys_Time[1]);
	Rc_Calc(rc);//ң�������ݲɼ�
	PCM_Fail_Safe(rc);
	Tim_Calc(&Sys_Time[1]);
	Time[1] = Sys_Time[1].OUT;
}

//10ms���ڣ�ִ�������ƣ���freeRTOS.c�޸�
void Sys_Run(void)
{
	Tim_Calc(&Deta_Time[2]);
	Tim_Calc(&Sys_Time[2]);
	GPS_INS_EKF(gps);//9��EKF λ�á��ٶȡ����ٶ�ƫ��
	LLH2NED(gps);//LLH����ϵתNED����ϵ
	update_flight_mode();//����ģʽ
	Sd_Calc(sd);//SD��
	Mot_Ctr(mot);//����������
	Ms5611_Ctr(ms5611);//��ѹ���¶ȿ���
  //Fprintf("%d$",CtrlIO->control_mode,"%d\r\n",Time[2]);
	Tim_Calc(&Sys_Time[2]);
	Time[2] = Sys_Time[2].OUT;	
}

//1ms���ڣ�����ʱ����ƣ���freeRTOS.c�޸�
void Sys_1ms(void)
{
	Tim_Calc(&Deta_Time[3]);
	Tim_Calc(&Sys_Time[3]);
	Bell_Loop_1ms(bell);
	//Tran_Loop_1ms(tran);
	Tim_Calc(&Sys_Time[3]);
	Time[3] = Sys_Time[3].OUT;
}

//50ms���ڣ�ϵͳ��⣬��freeRTOS.c�޸�
void Sys_Chk(void)
{
  Tim_Calc(&Deta_Time[4]);
	Tim_Calc(&Sys_Time[4]);
	output2ground();//20hz�´�
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
