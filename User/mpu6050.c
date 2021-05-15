#include "mpu6050.h"

/***************************************************\
功能：
  mpu6050 重力计与陀螺仪I2C读取
说明：
  1、修改端口
  2、每次读取完数据需要将Update复位
  3、注意需要修改i2c库的timeout改成较小的值，否则硬件错误时会等待很长一段时间。
  I2C_TIMEOUT_BUSY
  4、IMU可考虑使用740LA1
\***************************************************/

sIMU imu[IMU_NUM];

/******************驱动程序****************/
bool Imu_I2c_Read(I2C_HandleTypeDef *hi2c,u8 mem_add,u8 *data,u16 len)
{
    if(HAL_I2C_Mem_Read(hi2c,MPU6050_ADDR,mem_add,I2C_MEMADD_SIZE_8BIT,data,len,I2C_TIMEOUT)!=HAL_OK)return false;
	return true;
}

bool Imu_I2c_Write(I2C_HandleTypeDef *hi2c,u8 mem_add,u8 data)
{
    if(HAL_I2C_Mem_Write(hi2c,MPU6050_ADDR,mem_add,I2C_MEMADD_SIZE_8BIT,&data,1,I2C_TIMEOUT)!=HAL_OK)return false;
	return true;
}

/******************功能函数****************/
void Imu_Init_Para(sIMU *ele,sIMU_CFG *elecfg)
{
	ele->Update = false;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	ele->AccCal = false;
	ele->GyrCal = false;
	ele->AccCalFlag = 0;
	
	ele->hi2c = elecfg->hi2c;
	ele->name = elecfg->name;
	ele->Filt.CNT = 0;
	ele->Filt.CCR = elecfg->FiltNum;
	ele->Time = 0;

	if(Dir_Trans(ele->Dir,elecfg->Dir)==false)ele->Err = ERR_SOFT;
	ele->OneG = elecfg->OneG;
	for(u8 i=0;i<3;i++)
	{
		ele->AccOff[i] = elecfg->AccOff[i];
		ele->GyrOffO[i] = elecfg->GyrOffO[i];
		ele->GyrOffT[i] = elecfg->GyrOffT[i];
	}
}

bool Imu_Init(sIMU *ele)
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
	bool I2cSta=true;
	I2cSta&=Imu_I2c_Write(ele->hi2c,MPU6050_PWR_MGMT_1,0x00);
	HAL_Delay(200);
	I2cSta&=Imu_I2c_Write(ele->hi2c,MPU6050_SMPLRT_DIV,0x07);
	I2cSta&=Imu_I2c_Write(ele->hi2c,MPU6050_PWR_MGMT_2,0x00);
	I2cSta&=Imu_I2c_Write(ele->hi2c,MPU6050_SMPLRT_DIV,0x00);
	I2cSta&=Imu_I2c_Write(ele->hi2c,MPU6050_CONFIG,BITS_DLPF_CFG_5HZ);
	I2cSta&=Imu_I2c_Write(ele->hi2c,MPU6050_GYRO_CONFIG,BITS_FS_2000DPS);
	I2cSta&=Imu_I2c_Write(ele->hi2c,MPU6050_ACCEL_CONFIG,BITS_FS_4G);
	if(I2cSta==false)
	{
		Dprintf("--Cfg [NO]\r\n");
		ele->Err=ERR_HARD;
		return false;
	}
	Dprintf("--Cfg [OK]\r\n");
	//检测
	u8 ID=0;
	I2cSta&=Imu_I2c_Read(ele->hi2c,MPU6050_WHOAMI,&ID,1);
	if(ID!=0x68)
	{
		Dprintf("--Chk [NO]\r\n");
		ele->Err=ERR_LOST;
		return false;
	}
	Dprintf("--Chk [OK]\r\n");
	//初始化完毕
	ele->GyrCal = false;
	ele->Sta = STA_RUN;
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

bool Imu_Read(sIMU *ele)
{
	u8 DatRaw[14];
	if(Imu_I2c_Read(ele->hi2c,MPU6050_ACCEL_XOUT_H,DatRaw,14)==false)
	{
		ele->Err=ERR_HARD;
		return false;
	}
	ele->TmpRaw[0] = (DatRaw[ 6]<<8) + DatRaw[ 7];
	ele->AccRaw[0] = (DatRaw[ 0]<<8) + DatRaw[ 1];
	ele->AccRaw[1] = (DatRaw[ 2]<<8) + DatRaw[ 3];
	ele->AccRaw[2] = (DatRaw[ 4]<<8) + DatRaw[ 5];
	ele->GyrRaw[0] = (DatRaw[ 8]<<8) + DatRaw[ 9];
	ele->GyrRaw[1] = (DatRaw[10]<<8) + DatRaw[11];
	ele->GyrRaw[2] = (DatRaw[12]<<8) + DatRaw[13];
	return true;
}

static bool Imu_GyrCali(sIMU *ele)
{
	static sIMU *ready = NULL;
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
	static float  GyrSum[2][3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	static float  ImuTmp[2][1] = {0.0f, 0.0f};
	static u16 SmpCnt = 0;
	static u8 OneSmpCnt = 0;
	if(OneSmpCnt++<2)return false;  //2个周期采一次数据
	if(SmpCnt++<GYR_SAMPLE_NUM)     //采集由采集函数进行
	{
		OneSmpCnt=0;
		GyrSum[1][0] += (float)ele->GyrRaw[0];
		GyrSum[1][1] += (float)ele->GyrRaw[1];
		GyrSum[1][2] += (float)ele->GyrRaw[2];
		ImuTmp[1][0] += (float)ele->TmpRaw[0]/340.0f+36.53f;
		return false;
	}
	SmpCnt = 0;
	GyrSum[1][0] /= (float)GYR_SAMPLE_NUM;
	GyrSum[1][1] /= (float)GYR_SAMPLE_NUM;
	GyrSum[1][2] /= (float)GYR_SAMPLE_NUM;
	ImuTmp[1][0] /= (float)GYR_SAMPLE_NUM;
	if(ImuTmp[0][0]!=0.0f && ImuTmp[1][0]-ImuTmp[0][0]>2.0f && ImuTmp[1][0]-ImuTmp[0][0]<-2.0f)//相差2度以上
	{
		ele->GyrOffT[0] = (GyrSum[1][0]-GyrSum[0][0])/(ImuTmp[1][0]-ImuTmp[0][0]);
		ele->GyrOffT[1] = (GyrSum[1][1]-GyrSum[0][1])/(ImuTmp[1][0]-ImuTmp[0][0]);
		ele->GyrOffT[2] = (GyrSum[1][2]-GyrSum[0][2])/(ImuTmp[1][0]-ImuTmp[0][0]);
		ele->GyrOffO[0] = GyrSum[1][0]-(ImuTmp[1][0]-25.0f)*ele->GyrOffT[0];
		ele->GyrOffO[1] = GyrSum[1][1]-(ImuTmp[1][0]-25.0f)*ele->GyrOffT[1];
		ele->GyrOffO[2] = GyrSum[1][2]-(ImuTmp[1][0]-25.0f)*ele->GyrOffT[2];
	}
	else
	{
		ele->GyrOffT[0] = 0.0f;
		ele->GyrOffT[1] = 0.0f;
		ele->GyrOffT[2] = 0.0f;
		ele->GyrOffO[0] = GyrSum[1][0];
		ele->GyrOffO[1] = GyrSum[1][1];
		ele->GyrOffO[2] = GyrSum[1][2];
	}
	GyrSum[0][0] = GyrSum[1][0];
	GyrSum[0][1] = GyrSum[1][1];
	GyrSum[0][2] = GyrSum[1][2];
	ImuTmp[0][0] = ImuTmp[1][0];
	GyrSum[1][0] = 0.0f;
	GyrSum[1][1] = 0.0f;
	GyrSum[1][2] = 0.0f;
	ImuTmp[1][0] = 0.0f;
	Dprintf("\r\n%s Cali--Temp is:%.2f,Zero Bias is:%.2f %.2f %.2f,Tmp Bias is:%.2f %.2f %.2f\r\n",ele->name,
		ImuTmp[0][0],ele->GyrOffO[0],ele->GyrOffO[1],ele->GyrOffO[2],ele->GyrOffT[0],ele->GyrOffT[1],ele->GyrOffT[2]);
	ele->Sta = STA_RUN;
	GyrIsCali = false;
	ready = NULL;
	return true;	
}

static bool Imu_AccCali(sIMU *ele)
{
	static sIMU *ready = NULL;
	if(ready==NULL)ready=ele;
	else if(ready!=ele)return false;
	
	static u8 Index = 0;
	static bool AccIsCali = false;
	if(ele->AccCal==true)
	{
		ele->AccCal = false;
		ele->Sta = STA_CAL;
		AccIsCali = true;
		Index = ele->AccCalFlag;
		Dprintf("\r\n%s Acc Cali:%d\r\n",ele->name,Index);
	}
	if(AccIsCali!=true)return true;
	#define ACC_SAMPLE_NUM 50  //0.5s     
	static float  AccSum[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	static u16 SmpCnt = 0;
	static u8 OneSmpCnt = 0;
	if(++OneSmpCnt<2)return false;  //2个周期采一次数据
	if(++SmpCnt<ACC_SAMPLE_NUM)   //采集由采集函数进行
	{
		OneSmpCnt=0;
		switch(Index)  //6面校准法
		{
			case 0:for(u8 i=0;i<6;i++)AccSum[i]=0.0f;AccIsCali=false;SmpCnt=0;break;
			case 1:AccSum[0] += -ele->Dir[4]*(ele->AccRaw[ele->Dir[5]]);break;  //+Z
			case 2:AccSum[1] +=  ele->Dir[0]*(ele->AccRaw[ele->Dir[1]]);break;  //-X
			case 3:AccSum[2] += -ele->Dir[2]*(ele->AccRaw[ele->Dir[3]]);break;  //+Y
			case 4:AccSum[3] +=  ele->Dir[2]*(ele->AccRaw[ele->Dir[3]]);break;  //-Y
			case 5:AccSum[4] += -ele->Dir[0]*(ele->AccRaw[ele->Dir[1]]);break;  //+X
			case 6:AccSum[5] +=  ele->Dir[4]*(ele->AccRaw[ele->Dir[5]]);break;  //-Z
		}
		return false;
	}
	SmpCnt = 0;
	AccSum[Index-1]/=(float)ACC_SAMPLE_NUM;
	Dprintf("%s Acc Cali:%d--G:%.2f\r\n",ele->name,Index,AccSum[Index-1]);  //正常值8250
	if(Index == 6)
	{
		ele->AccOff[0] = -ele->Dir[0]*(AccSum[4]-AccSum[1])/2.0f;
		ele->AccOff[1] = -ele->Dir[2]*(AccSum[2]-AccSum[3])/2.0f;
		ele->AccOff[2] = -ele->Dir[4]*(AccSum[0]-AccSum[5])/2.0f;
		ele->OneG = (AccSum[0]+AccSum[1]+AccSum[2]+AccSum[3]+AccSum[4]+AccSum[5])/6.0f*IMU_ACC_FCT;
		Dprintf("\r\n%s Cali--Bias is:%d %d %d,OneG is:%.2f\r\n",ele->name,
			ele->AccOff[0],ele->AccOff[1],ele->AccOff[2],ele->OneG);
	}
	ele->Sta = STA_RUN;
	AccIsCali = false;
	ready = NULL;
	return true;	
}

bool Imu_Calc(sIMU *ele)
{
	if(ele->Sta == STA_INI)return false;
	Tim_Calc(&ele->Tim);   //计时  
	if(Imu_Read(ele)==false)return false;
	Imu_GyrCali(ele);
	Imu_AccCali(ele);
	//获取温度和温漂值
	ele->TmpRel[0]=(float)ele->TmpRaw[0]/340.0f+36.53f;
	ele->GyrOff[0] = (ele->TmpRel[0]-25.0f)*ele->GyrOffT[0]+ele->GyrOffO[0];
	ele->GyrOff[1] = (ele->TmpRel[0]-25.0f)*ele->GyrOffT[1]+ele->GyrOffO[1];
	ele->GyrOff[2] = (ele->TmpRel[0]-25.0f)*ele->GyrOffT[2]+ele->GyrOffO[2];
	//方向变换
	ele->AccRot[0] = -ele->Dir[0]*(ele->AccRaw[ele->Dir[1]]-ele->AccOff[ele->Dir[1]]);
	ele->AccRot[1] = -ele->Dir[2]*(ele->AccRaw[ele->Dir[3]]-ele->AccOff[ele->Dir[3]]);
	ele->AccRot[2] = -ele->Dir[4]*(ele->AccRaw[ele->Dir[5]]-ele->AccOff[ele->Dir[5]]);
	ele->GyrRot[0] =  ele->Dir[0]*(ele->GyrRaw[ele->Dir[1]]-ele->GyrOff[ele->Dir[1]]);
	ele->GyrRot[1] =  ele->Dir[2]*(ele->GyrRaw[ele->Dir[3]]-ele->GyrOff[ele->Dir[3]]);
	ele->GyrRot[2] =  ele->Dir[4]*(ele->GyrRaw[ele->Dir[5]]-ele->GyrOff[ele->Dir[5]]);
	//实际值转换
	ele->AccRel[0] = ele->AccRot[0]*IMU_ACC_FCT;
	ele->AccRel[1] = ele->AccRot[1]*IMU_ACC_FCT;
	ele->AccRel[2] = ele->AccRot[2]*IMU_ACC_FCT;
	ele->GyrRel[0] = ele->GyrRot[0]*IMU_GYR_FCT;
	ele->GyrRel[1] = ele->GyrRot[1]*IMU_GYR_FCT;
	ele->GyrRel[2] = ele->GyrRot[2]*IMU_GYR_FCT;
	//滤波处理
	SlideFilt(ele->AccFil,ele->AccRel,3,&ele->Filt,1);
	SlideFilt(ele->GyrFil,ele->GyrRel,3,&ele->Filt,2);
	SlideFilt(ele->TmpFil,ele->TmpRel,1,&ele->Filt,2);
	ele->Update=true;
	Tim_Calc(&ele->Tim);
	ele->Time = ele->Tim.OUT;
	return true;
}
/******************************END OF FILE************************************/

