#ifndef __GPS_H
#define __GPS_H

#include "userlib.h"
#include "exit.h"
#define GPS_FREQ 10

#define C_WGS84_a 6378137.0        //地球长半轴
#define C_WGS84_b 6356752.314245   //地球短半轴   f=(a-b)/a
#define C_WGS84_f 1.0/298.257223563  //地球扁率
#define C_WGS84_e 0.081819790992   //第一偏心率

#define GPS_RX_LEN 200
#define GPS_TYPE_NUM 5     //GPS传输数据类型 $GPGGA $GPVTG $HMC $GPS $CPM

typedef enum
{
	GPGGA = 0,
	GPVTG = 1,
	HMC   = 2,
	GPS   = 3,
	CPM   = 4,
}GPS_TYPE;

typedef struct 
{
	UART_HandleTypeDef *huart;
	char *name;
	char *Dir;
	s16   CheckNum;
	float MagOff[3];
}sGPS_CFG;

typedef struct
{
	UART_HandleTypeDef *huart;
	HAL_LockTypeDef LockRx;
	char *name;
	u8   RxRawDat[GPS_RX_LEN];   //数据
	u16  RxRawIndex;
	bool RxRawHead;
	bool RxFlag;      //接收标志位
	u8   RxDat[GPS_RX_LEN];
	
	bool  GpsCal;        
	sTIM  Tim;        //计时器
	sCNT  Chk;        //检测
	float time;
	double lat,lng;
	u8    status;
	u8    star;
	float hdop;
	float alti;
	float wgs_alt;
	float vtg_dir;  
	double vtg_spd;
	double Re2t[3][3];
	double Zero_ECFF[3];
	double ECFF[3];
	double ECFF_Init[3];
	double LLH[3];
	double LLH_Init[3];
	double N;
	double N_Init;
	
	bool  MagCal;
	s8    Dir[6];     //方向  --
  s16   MagOff[3];  //漂移量  --
  s16   MagRaw[3];  //原始值
  s16   MagRot[3];  //旋转值
  float MagRel[3];  //实际值
	//用户访问数据
	bool  Update;     //更新  --
  eSTA  Sta;        //状态  --
  eERR  Err;        //错误信息  --
	u32   Time;
	double NED[3];
	double NED_Init[3];
	double NED_spd[3];
	eERR  MagErr;     //错误信息  --
	eSTA  MagSta;     //状态  --
	bool  MagUpdate;  //更新  --
	u8  GPSUpdate;
	bool ECEF_Init_Flag;//ECEF坐标系原点标志位
	float MagFil[3];
	bool GPS_INS_EKF_flag;//GPS_INS_EKF 初始化标志位
	bool GPS_INS_EKF_start_flag;
	double V_P[3];//速度预测值
	double P_p[3];//经纬度预测值
	double acc_bias_p[3];//加速度偏置预测值
	double Rm_p;
	double Rn_p;
	double V_EKF[3];//EKF 速度值
	double P_EKF[3];//EKF 位置值
	double acc_bias_EKF[3];//
	double P_GI[81];//协方差阵
	u32    gps_update_time;
}sGPS;

#define GPS_NUM 1
extern sGPS gps[GPS_NUM];
extern int gps_t;
void Gps_Init_Para(sGPS *ele,sGPS_CFG *elecfg);
bool Gps_Init(sGPS *ele);
bool Gps_Calc(sGPS *ele);

#endif
