#include "hmc5883.h"
/***************************************************\
功能：
  hmc5883 磁力计I2C读取
说明：
  1、修改端口
  2、每次读取完数据需要将Update复位
  3、注意需要修改i2c库的timeout改成较小的值，否则硬件错误时会等待很长一段时间。
  I2C_TIMEOUT_BUSY
\***************************************************/

sHMC hmc[HMC_NUM];

/******************驱动程序****************/
bool Hmc_I2c_Read(I2C_HandleTypeDef *hi2c,u8 mem_add,u8 *data,u16 len)
{
	if(HAL_I2C_Mem_Read(hi2c,HMC5883_ADDR,mem_add,I2C_MEMADD_SIZE_8BIT,data,len,I2C_TIMEOUT)==HAL_OK)return true;
	else return false;
}

bool Hmc_I2c_Write(I2C_HandleTypeDef *hi2c,u8 mem_add,u8 data)
{
	if(HAL_I2C_Mem_Write(hi2c,HMC5883_ADDR,mem_add,I2C_MEMADD_SIZE_8BIT,&data,1,I2C_TIMEOUT)==HAL_OK)return true;
	else return false;
}

/******************HMC功能函数****************/
void Hmc_Init_Para(sHMC *ele,sHMC_CFG *elecfg)
{
	ele->Update = false;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	ele->HmcCal = false;
	
	ele->hi2c = elecfg->hi2c;
	ele->name = elecfg->name;
	ele->Filt.CNT = 0;
	ele->Filt.CCR = elecfg->FiltNum;
	ele->Time = 0;

	if(Dir_Trans(ele->Dir,elecfg->Dir)==false)ele->Err = ERR_SOFT;
	ele->HmcOff[0] = elecfg->HmcOff[0];
	ele->HmcOff[1] = elecfg->HmcOff[1];
	ele->HmcOff[2] = elecfg->HmcOff[2];
}

bool Hmc_Init(sHMC *ele)
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
	I2cSta&=Hmc_I2c_Write(ele->hi2c,HMC5883_CONFIG_REG_A,0xF8);
	I2cSta&=Hmc_I2c_Write(ele->hi2c,HMC5883_MODE_REG,0x00);
	if(I2cSta==false)
	{
			Dprintf("--Cfg [NO]\r\n");
			ele->Err=ERR_HARD;
			return false;
	}
	Dprintf("--Cfg [OK]\r\n");
	//检测
	ele->Sta = STA_CHK;
	u8 ID=0;
	I2cSta&=Hmc_I2c_Read(ele->hi2c,HMC5883_WHO_AM_I_A,&ID,1);
	if(ID!='H')
	{
			Dprintf("--Chk [NO]\r\n");
			ele->Err=ERR_LOST;
			return false;
	}
	Dprintf("--Chk [OK]\r\n");
	//初始化完毕
	ele->Sta = STA_RUN;
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

bool Hmc_Read(sHMC *ele)
{
	u8 DatRaw[6];
	if(Hmc_I2c_Read(ele->hi2c,HMC5883_DATA_X_MSB_REG,DatRaw,6)==false)
	{
		ele->Err=ERR_HARD;
		return false;
	}
	ele->HmcRaw[0] = (DatRaw[ 0]<<8) + DatRaw[ 1];
	ele->HmcRaw[1] = (DatRaw[ 2]<<8) + DatRaw[ 3];
	ele->HmcRaw[2] = (DatRaw[ 4]<<8) + DatRaw[ 5];
	return true;
}

static bool Hmc_Cali(sHMC *ele)
{
	static sHMC *ready = NULL;
	if(ready==NULL)ready=ele;
	else if(ready!=ele)return false;
	
	static bool HmcIsCali = false;
	if(ele->HmcCal==true)   
	{
		ele->HmcCal = false;
		ele->Sta = STA_CAL;
		HmcIsCali = true;
		Dprintf("\r\n%s Hmc Cali\r\n",ele->name);
	}
	if(HmcIsCali == false)return true;
	#define SAMPLE_NUM 600  //30s
	static float d[SAMPLE_NUM][3];
	static u16 SmpCnt = 0;
	static u8 OneSmpCnt = 0;
	if(OneSmpCnt++<10)return false;  //10个周期采一次数据
	if(SmpCnt<SAMPLE_NUM)
	{
		OneSmpCnt=0;
		d[SmpCnt][0] = (float)ele->HmcRaw[0];
		d[SmpCnt][1] = (float)ele->HmcRaw[1];
		d[SmpCnt][2] = (float)ele->HmcRaw[2];
		SmpCnt++;
		return false;
	}
	SmpCnt = 0;
	u16 Pop[2][3];
	float Origin[3],Radius;
	sphereFit(d, SAMPLE_NUM, 100, 0.0f, Pop, Origin, &Radius);
	ele->HmcOff[0]=Origin[0];
	ele->HmcOff[1]=Origin[1];
	ele->HmcOff[2]=Origin[2];
	Dprintf("\r\n%s Cali--Bias is:%4.2f %4.2f %4.2f,Radius is:%4.2f\r\n",ele->name,Origin[0],Origin[1],Origin[2],Radius);
	ele->Sta = STA_RUN;
	HmcIsCali = false;
	ready = NULL;
	return true;		
}

bool Hmc_Calc(sHMC *ele)
{
	if(ele->Sta != STA_CAL && ele->Sta != STA_RUN)return false;
	Tim_Calc(&ele->Tim);
	if(Hmc_Read(ele)==false)return false;
	Hmc_Cali(ele);
	if(ele->Sta!=STA_RUN)return false;
	//方向变换
	ele->HmcRot[0]=ele->Dir[0]*(ele->HmcRaw[ele->Dir[1]]-ele->HmcOff[ele->Dir[1]]);
	ele->HmcRot[1]=ele->Dir[2]*(ele->HmcRaw[ele->Dir[3]]-ele->HmcOff[ele->Dir[3]]);
	ele->HmcRot[2]=ele->Dir[4]*(ele->HmcRaw[ele->Dir[5]]-ele->HmcOff[ele->Dir[5]]);
	//实际值转换
	ele->HmcRel[0]=ele->HmcRot[0]*HMC_FCT;
	ele->HmcRel[1]=ele->HmcRot[1]*HMC_FCT;
	ele->HmcRel[2]=ele->HmcRot[2]*HMC_FCT;
	//滤波处理
  SlideFilt(ele->HmcFil,ele->HmcRel,3,&ele->Filt,1);
	ele->Update=true;
	Tim_Calc(&ele->Tim);
	ele->Time = ele->Tim.OUT;
	return true;
}

/******************************END OF FILE************************************/
