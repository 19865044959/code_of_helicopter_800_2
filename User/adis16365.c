#include "adis16365.h"

/***************************************************\
功能：
  adis 惯性导航系统SPI读取
说明：
  1、修改端口,配置成16bit模式，high 2edge（模式3）
  2、每次读取完数据需要将Update复位
\***************************************************/

sADIS adis[ADIS_NUM];

/******************驱动程序****************/
//spi只发送和接收数据，至于数据怎么处理，相应哪一部分是什么意义，是由器件决定的。
//adis对于接收到的数据到的数据进行的操作：[15][14-8][7-0],[15]R/W 0/1 [14-8]地址 [7-0]写入的数据
//adis要发送的数据则为上一次接收到的地址对应的寄存器值。

#define ADIS_CS_CLR	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET)
#define ADIS_CS_SET	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET)

//发送数据，数据[15][14-8][7-0],[15]R/W 0/1 [14-8]地址 [7-0]写入的数据
//若要往地址为ADD的寄存器写入数据（8bit）DAT，则Data=ADD+ADIS_WRITE_CMD+DAT
void Adis_Spi_Tran(SPI_HandleTypeDef *hspi,u16 Data)
{
	ADIS_CS_CLR;
	HAL_SPI_Transmit(hspi,(uint8_t *)&Data,1,10);
	ADIS_CS_SET;
}

//发送与接收数据，发送数据[15][14-8][7-0],[15]R/W 0/1 [14-8]地址 [7-0]写入的数据，接收数据为上一次发送地址的寄存器值。
u16 Adis_Spi_TranRecv(SPI_HandleTypeDef *hspi,u16 Data)   //返回上一个地址的数据，data高8位为地址值，
{
	u16 ReadData;
	ADIS_CS_CLR;
	HAL_SPI_TransmitReceive(hspi,(uint8_t *)&Data,(uint8_t *)&ReadData,1,10);  //全双工
	ADIS_CS_SET;
	return ReadData;
}

/******************功能函数****************/
void Adis_Init_Para(sADIS *ele,sADIS_CFG *elecfg)
{
	ele->Update = false;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;

	ele->hspi = elecfg->hspi;
	ele->name = elecfg->name;
	ele->Filt.CNT = 0;
	ele->Filt.CCR = elecfg->FiltNum;
	if(Dir_Trans(ele->Dir,elecfg->Dir)==false)ele->Err = ERR_SOFT;
	ele->OneG = 9.788f;

	ele->AccOff[0] = elecfg->AccOff[0];
	ele->AccOff[1] = elecfg->AccOff[1];
	ele->AccOff[2] = elecfg->AccOff[2];
	ele->GyrOff[0] = elecfg->GyrOff[0];
	ele->GyrOff[1] = elecfg->GyrOff[1];
	ele->GyrOff[2] = elecfg->GyrOff[2];
}

bool Adis_Init(sADIS *ele)
{
	Dprintf("\r\n%s Init...\r\n",ele->name);
  //配置
	if(ele->Sta != STA_INI)
	{
		Dprintf("--Par [NO]\r\n");
        return false;
	}
	if(ele->Err==ERR_SOFT)
	{
			Dprintf("--Dir [NO]\r\n");
			return false;
	}
	Dprintf("--Dir [OK]\r\n");
	//检测
	ele->Sta = STA_CHK;
	u16 ID=0,STA=0;
  Adis_Spi_Tran(ele->hspi,ADIS_PROD_ID);
	HAL_Delay(1);
	ID=Adis_Spi_TranRecv(ele->hspi,ADIS_DIAG_STAT);
	HAL_Delay(1);
	STA=Adis_Spi_TranRecv(ele->hspi,ADIS_DIAG_STAT);
	if(ID!=16360 && ID!=16365)  
	{
		Dprintf("--Chk [NO]\r\n");
        ele->Err=ERR_LOST;
        return false;
    }
  Dprintf("--Chk [OK]\r\n");
	if((STA&0xFC7F)!=0x0000)   //自检通过
	{
		Dprintf("--Sta [NO]\r\n");
		ele->Err=ERR_UPDT;
		return false;
	}
	Dprintf("--Sta [OK]\r\n");
	//初始化完毕
	ele->Sta = STA_RUN;
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

bool Adis_Read(sADIS *ele)
{
	u16 ImuAdd[8]={ADIS_SUPPLY_OUT,ADIS_XGYRO_OUT,ADIS_YGYRO_OUT,ADIS_ZGYRO_OUT,
									ADIS_XACCL_OUT,ADIS_YACCL_OUT,ADIS_ZACCL_OUT,ADIS_DIAG_STAT};
	u16 ImuDat[8];
	for(int i=0;i<8;i++)
	{
		ImuDat[i]=Adis_Spi_TranRecv(ele->hspi,ImuAdd[i]);
		User_Delay(10);
	}

	if((ImuDat[0]&0xFC7F)!=0x0000)
	{
		ele->Err = ERR_UPDT;
		return false;
	}
	if(ImuDat[1]&0x2000)ele->VolRaw[0]=(ImuDat[1]&0x0FFF)-0x1000;   //12bit
	else ele->VolRaw[0]=(ImuDat[1]&0x0FFF);
	for(int i=0;i<3;i++)
	{
		if(ImuDat[i+2]&0x2000)ele->GyrRaw[i]=(ImuDat[i+2]&0x3FFF)-0x4000;
		else ele->GyrRaw[i]=(ImuDat[i+2]&0x3FFF);
		if(ImuDat[i+5]&0x2000)ele->AccRaw[i]=(ImuDat[i+5]&0x3FFF)-0x4000;
		else ele->AccRaw[i]=(ImuDat[i+5]&0x3FFF);
	}
	return true;
}

static bool Adis_GyrCali(sADIS *ele)
{
	static sADIS *ready = NULL;
	if(ready==NULL)ready=ele;
	else if(ready!=ele)return false;
	
	static bool GyrIsCali = false;
	if(ele->GyrCal==true)
	{
		ele->GyrCal = false;
		ele->Sta = STA_CAL;
		GyrIsCali = true;
		Dprintf("\r\n%s Gyr Cali\r\n",ele->name);
	}
	if(GyrIsCali!=true)return true;
	#define GYR_SAMPLE_NUM 300  //3s     
	static float  GyrSum[3] = {0.0f, 0.0f, 0.0f};
	static u16 SmpCnt = 0;
	static u8 OneSmpCnt = 0;
	if(OneSmpCnt++<2)return false;  //2个周期采一次数据
	if(SmpCnt++<GYR_SAMPLE_NUM)    //采集由采集函数进行
	{
		OneSmpCnt=0;
		GyrSum[0] += (float)ele->GyrRaw[0];
		GyrSum[1] += (float)ele->GyrRaw[1];
		GyrSum[2] += (float)ele->GyrRaw[2];
		return false;
	}
	SmpCnt = 0;
	ele->GyrOff[0] = GyrSum[0]/(float)GYR_SAMPLE_NUM;
	ele->GyrOff[1] = GyrSum[1]/(float)GYR_SAMPLE_NUM;
	ele->GyrOff[2] = GyrSum[2]/(float)GYR_SAMPLE_NUM;
	GyrSum[0] = 0;
	GyrSum[1] = 0;
	GyrSum[2] = 0;
	
	Dprintf("\r\n%s Cali--Bias is:%d %d %d\r\n",ele->name,ele->GyrOff[0],ele->GyrOff[1],ele->GyrOff[2]);
	ele->Sta = STA_RUN;
	GyrIsCali = false;
	ready = NULL;
	return true;	
}

void LowPassFilt (sADIS *ele,float filt_para)
{
	ele->AccFil[0] += filt_para * (ele->AccRel[0] - ele->AccFil[0]);
	ele->AccFil[1] += filt_para * (ele->AccRel[1] - ele->AccFil[1]);
	ele->AccFil[2] += filt_para * (ele->AccRel[2] - ele->AccFil[2]);
}

void Butterworth_2nd(sADIS *ele)
{	
	//15hz左右
	//大约延时：0.1s左右
//	#define a0  0.02008336556421f
//	#define a1  0.04016673112842f
//	#define a2  0.02008336556421f
//	#define b1 -1.56101807580071f
//	#define b2  0.64135153805756f
	
	//10Hz
	//大约延时：0.15s左右
//	#define a0  0.067455273889071896f
//	#define a1  0.13491054777814379f
//	#define a2  0.067455273889071896f
//	#define b1 -1.1429805025399011f
//	#define b2  0.41280159809618877f
	
	//12Hz
	//大约延时：0.125s左右
	#define a0  0.091314900435831972f
	#define a1  0.18262980087166394f
	#define a2  0.091314900435831972f
	#define b1 -0.98240579310839538f
	#define b2  0.34766539485172343f
	
	


	static float A_last[3] = {0,0,0};
	static float A_last_last[3] = {0,0,0};
	static float B_last[3] = {0,0,0};
	static float B_last_last[3] = {0,0,0};
	
	for(int i=0;i<3;i++)
	{
		ele->GyrFil_2nd[i] = a0 * ele->GyrRel[i] + a1 * A_last[i] + a2 * A_last_last[i] - b1 * B_last[i] - b2 * B_last_last[i];
	}
	
	for(int i=0;i<3;i++)
	{
		A_last_last[i] = A_last[i];
		A_last[i] = ele->GyrRel[i];
		
		B_last_last[i] = B_last[i];
		B_last[i] = ele->GyrFil_2nd[i];
	}
}

void Butterworth_3nd(sADIS *ele)
{
////wc=20hz
//	#define a0_3nd 0.018098933007514f
//	#define a1_3nd 0.054296799022543f
//	#define a2_3nd 0.054296799022543f
//	#define a3_3nd 0.018098933007514f
//	#define b1_3nd -1.760041880343169f
//	#define b2_3nd 1.182893262037831f
//	#define b3_3nd -0.278059917634547f
////wc = 15hz
//	#define a0_3nd 	0.00859868608632f
//	#define a1_3nd  0.02579605825896f 
//	#define a2_3nd  0.02579605825895f 
//	#define a3_3nd  0.00859868608632f
//	#define b1_3nd -2.06513978339347f
//	#define b2_3nd  1.51999264125030f 
//	#define b3_3nd -0.38606336916625f 
//wc = 10hz
	#define a0_3nd 	0.0028981946337214f
	#define a1_3nd  0.0086945839011642f 
	#define a2_3nd  0.0086945839011642f 
	#define a3_3nd  0.0028981946337214f
	#define b1_3nd -2.3740947437093523f
	#define b2_3nd  1.9293556690912153f 
	#define b3_3nd -0.5320753683120919f 
	static float A_last_3nd[3] = {0,0,0};
	static float A_last_last_3nd[3] = {0,0,0};
	static float A_last_last_last_3nd[3] = {0,0,0};
	static float B_last_3nd[3] = {0,0,0};
	static float B_last_last_3nd[3] = {0,0,0};
	static float B_last_last_last_3nd[3] = {0,0,0};
	
	for(int i=0;i<3;i++)
	{
		ele->AccFil_3nd[i] = a0_3nd * ele->AccRel[i] + a1_3nd * A_last_3nd[i] + a2_3nd * A_last_last_3nd[i] + a3_3nd * A_last_last_last_3nd[i] - b1_3nd * B_last_3nd[i] - b2_3nd * B_last_last_3nd[i] - b3_3nd * B_last_last_last_3nd[i];
	}
	
	for(int i=0;i<3;i++)
	{
		A_last_last_last_3nd[i] = A_last_last_3nd[i];
		A_last_last_3nd[i] = A_last_3nd[i];
		A_last_3nd[i] = ele->AccRel[i];
		
		B_last_last_last_3nd[i] = B_last_last_3nd[i];
		B_last_last_3nd[i] = B_last_3nd[i];
		B_last_3nd[i] = ele->AccFil_3nd[i];
	}
}

bool Adis_Calc(sADIS *ele)
{
	if(ele->Sta != STA_CAL && ele->Sta != STA_RUN)return false;
	Tim_Calc(&ele->Tim);   //计时 
	if(Adis_Read(ele)==false)return false;
	Adis_GyrCali(ele);
	//方向变换
	ele->AccRot[0] = ele->Dir[0]*(ele->AccRaw[ele->Dir[1]]-ele->AccOff[ele->Dir[1]]);
	ele->AccRot[1] = ele->Dir[2]*(ele->AccRaw[ele->Dir[3]]-ele->AccOff[ele->Dir[3]]);
	ele->AccRot[2] = ele->Dir[4]*(ele->AccRaw[ele->Dir[5]]-ele->AccOff[ele->Dir[5]]);
	ele->GyrRot[0] = ele->Dir[0]*(ele->GyrRaw[ele->Dir[1]]-ele->GyrOff[ele->Dir[1]]);
	ele->GyrRot[1] = ele->Dir[2]*(ele->GyrRaw[ele->Dir[3]]-ele->GyrOff[ele->Dir[3]]);
	ele->GyrRot[2] = ele->Dir[4]*(ele->GyrRaw[ele->Dir[5]]-ele->GyrOff[ele->Dir[5]]);
	//实际值转换
	ele->AccRel[0]=ele->AccRot[0]*ADIS_ACC_FCT;
	ele->AccRel[1]=ele->AccRot[1]*ADIS_ACC_FCT;
	ele->AccRel[2]=ele->AccRot[2]*ADIS_ACC_FCT;
	ele->GyrRel[0]=ele->GyrRot[0]*ADIS_GYR_FCT;
	ele->GyrRel[1]=ele->GyrRot[1]*ADIS_GYR_FCT;
	ele->GyrRel[2]=ele->GyrRot[2]*ADIS_GYR_FCT;
	ele->VolRel[0]=ele->VolRaw[0]*ADIS_VOL_FCT;
	//滤波处理
	//SlideFilt(ele->AccFil,ele->AccRel,3,&ele->Filt,1);
	SlideFilt(ele->GyrFil,ele->GyrRel,3,&ele->Filt,1);
	float filt_para = 0.4458f;//10hz
	LowPassFilt(ele,filt_para);
	Butterworth_2nd(ele);
	Butterworth_3nd(ele);
	 ele->AccFil_3nd[0] = 1.007599f * (ele->AccFil_3nd[0] - 0.031598f);
	ele->AccFil_3nd[1] = 1.004482f * (ele->AccFil_3nd[1] + 0.010827f);
	ele->AccFil_3nd[2] = 1.000453f * (ele->AccFil_3nd[2] + 0.107315f);
	SlideFilt(ele->GyrFil,ele->GyrRel,3,&ele->Filt,1);
	SlideFilt(ele->VolRel,ele->VolRel,1,&ele->Filt,2);
	ele->Update=true;
	Tim_Calc(&ele->Tim);   //计时 
	ele->Time = ele->Tim.OUT;
	return true;
}

/******************************END OF FILE************************************/

