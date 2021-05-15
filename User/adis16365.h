#ifndef __ADIS16365_H
#define __ADIS16365_H

#include "userlib.h"

#define ADIS_FLASH_CNT 0x0000
#define ADIS_SUPPLY_OUT 0x0200
#define ADIS_XGYRO_OUT 0x0400
#define ADIS_YGYRO_OUT 0x0600
#define ADIS_ZGYRO_OUT 0x0800
#define ADIS_XACCL_OUT 0x0A00
#define ADIS_YACCL_OUT 0x0C00
#define ADIS_ZACCL_OUT 0x0E00
#define ADIS_XTEMP_OUT 0x1000
#define ADIS_YTEMP_OUT 0x1200
#define ADIS_ZTEMP_OUT 0x1400
#define ADIS_AUX_ADC   0x1600
#define ADIS_XGYRO_OFF 0x1A00
#define ADIS_YGYRO_OFF 0x1C00
#define ADIS_ZGYRO_OFF 0x1E00
#define ADIS_XACCL_OFF 0x2000
#define ADIS_YACCL_OFF 0x2200
#define ADIS_ZACCL_OFF 0x2400
#define ADIS_ALM_MAG1  0x2600
#define ADIS_ALM_MAG2  0x2800
#define ADIS_ALM_SMPL1 0x2A00
#define ADIS_ALM_SMPL2 0x2C00
#define ADIS_ALM_CTRL  0x2E00
#define ADIS_AUX_DAC   0x3000
#define ADIS_GPIO_CTRL 0x3200
#define ADIS_MSC_CTRL  0x3400
#define ADIS_SMPL_PRD  0x3600
#define ADIS_SENS_AVG  0x3800
#define ADIS_SLP_CNT   0x3A00
#define ADIS_DIAG_STAT 0x3C00
#define ADIS_GLOB_CMD  0x3E00
#define ADIS_LOT_ID1   0x5200
#define ADIS_LOT_ID2   0x5400
#define ADIS_PROD_ID   0x5600
#define ADIS_SERIAL_NUM 0x5800

#define ADIS_WRITE_CMD 0x8000

#define ADIS_ACC_FCT 0.032623f  //3.333mg=3.333*0.001*9.788    m/s
#define ADIS_GYR_FCT 0.0008726646f //rad/s                      °/s
//#define ADIS_GYR_FCT 0.05f      //0.05°/s                     °/s
#define ADIS_VOL_FCT 0.002418f    //2.418mV                      V

typedef struct
{
	SPI_HandleTypeDef *hspi;
	char *name;
	s16  FiltNum;
	char *Dir;   
	s16  AccOff[3];
	s16  GyrOff[3];
	float OneG;   
}sADIS_CFG;

//实际值：温度单位为摄氏度，陀螺仪为rad/s,加速度计为m/s，电压为V，高度为m。

typedef struct
{
	SPI_HandleTypeDef *hspi;  //端口  --
	char *name;       
	sTIM  Tim;        //计时器
	sCNT  Filt;       //均值滤波器  --
	bool  GyrCal;     //校准
	s8    Dir[6];     //方向  --
	s16   AccOff[3];
	s16   GyrOff[3];
	s16   AccRaw[3];  //原始值
	s16   GyrRaw[3];  //原始值
	s16   VolRaw[1];  //温度原始值
	s16   AccRot[3];  //旋转值
	s16   GyrRot[3];  //旋转值
	float AccRel[3];  //实际值
	float GyrRel[3];  //实际值
	float VolRel[1];  //温度实际值
	//用户访问数据
	bool  Update;     //更新  --
	eSTA  Sta;        //状态  --
	eERR  Err;        //错误信息  --
	u32   Time;       //计时
	float OneG;       //重力  --
	float AccFil[3];  //滤波值
	float GyrFil[3];  //滤波值
	float GyrFil_2nd[3];  //2阶角速度低通滤波值
	float VolFil[1];  //滤波值
	float AccFil_2nd[3];//2阶低通滤波器
	float AccFil_3nd[3];//3阶低通滤波器
}sADIS;

#define ADIS_NUM 1

extern sADIS adis[ADIS_NUM];

void Adis_Init_Para(sADIS *ele,sADIS_CFG *elecfg);
bool Adis_Init(sADIS *ele);
bool Adis_Read(sADIS *ele);
bool Adis_Calc(sADIS *ele);

#endif

