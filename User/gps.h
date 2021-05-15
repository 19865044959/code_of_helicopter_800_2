#ifndef __GPS_H
#define __GPS_H

#include "userlib.h"
#include "exit.h"
#define GPS_FREQ 10

#define C_WGS84_a 6378137.0        //���򳤰���
#define C_WGS84_b 6356752.314245   //����̰���   f=(a-b)/a
#define C_WGS84_f 1.0/298.257223563  //�������
#define C_WGS84_e 0.081819790992   //��һƫ����

#define GPS_RX_LEN 200
#define GPS_TYPE_NUM 5     //GPS������������ $GPGGA $GPVTG $HMC $GPS $CPM

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
	u8   RxRawDat[GPS_RX_LEN];   //����
	u16  RxRawIndex;
	bool RxRawHead;
	bool RxFlag;      //���ձ�־λ
	u8   RxDat[GPS_RX_LEN];
	
	bool  GpsCal;        
	sTIM  Tim;        //��ʱ��
	sCNT  Chk;        //���
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
	s8    Dir[6];     //����  --
  s16   MagOff[3];  //Ư����  --
  s16   MagRaw[3];  //ԭʼֵ
  s16   MagRot[3];  //��תֵ
  float MagRel[3];  //ʵ��ֵ
	//�û���������
	bool  Update;     //����  --
  eSTA  Sta;        //״̬  --
  eERR  Err;        //������Ϣ  --
	u32   Time;
	double NED[3];
	double NED_Init[3];
	double NED_spd[3];
	eERR  MagErr;     //������Ϣ  --
	eSTA  MagSta;     //״̬  --
	bool  MagUpdate;  //����  --
	u8  GPSUpdate;
	bool ECEF_Init_Flag;//ECEF����ϵԭ���־λ
	float MagFil[3];
	bool GPS_INS_EKF_flag;//GPS_INS_EKF ��ʼ����־λ
	bool GPS_INS_EKF_start_flag;
	double V_P[3];//�ٶ�Ԥ��ֵ
	double P_p[3];//��γ��Ԥ��ֵ
	double acc_bias_p[3];//���ٶ�ƫ��Ԥ��ֵ
	double Rm_p;
	double Rn_p;
	double V_EKF[3];//EKF �ٶ�ֵ
	double P_EKF[3];//EKF λ��ֵ
	double acc_bias_EKF[3];//
	double P_GI[81];//Э������
	u32    gps_update_time;
}sGPS;

#define GPS_NUM 1
extern sGPS gps[GPS_NUM];
extern int gps_t;
void Gps_Init_Para(sGPS *ele,sGPS_CFG *elecfg);
bool Gps_Init(sGPS *ele);
bool Gps_Calc(sGPS *ele);

#endif
