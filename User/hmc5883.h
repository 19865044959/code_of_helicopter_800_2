#ifndef __HMC5883_H
#define __HMC5883_H

#include "userlib.h"

//�Ĵ�����ַ
#define HMC5883_CONFIG_REG_A    0x00
#define HMC5883_CONFIG_REG_B    0x01
#define HMC5883_MODE_REG        0x02
#define HMC5883_DATA_X_MSB_REG  0x03
#define HMC5883_DATA_X_LSB_REG  0x04
#define HMC5883_DATA_Y_MSB_REG  0x05
#define HMC5883_DATA_Y_LSB_REG  0x06
#define HMC5883_DATA_Z_MSB_REG  0x07
#define HMC5883_DATA_Z_LSB_REG  0x08
#define HMC5883_STATUS_REG      0x09
#define HMC5883_WHO_AM_I_A      0x0A
#define HMC5883_WHO_AM_I_B      0x0B
#define HMC5883_WHO_AM_I_C      0x0C

//0x18:1\75 0x38:2\75 0x58:4\75 0x78:8\75 0x10:1\15 0x30:2\15 0x50:4\15 0x70:8\15
#define SENSOR_CONFIG 0x70    // 8 Sample average, 15 Hz

#define NORMAL_MEASUREMENT_CONFIGURATION 0x00
#define POSITIVE_BIAS_CONFIGURATION      0x01

//0x00:0.88 0x20:1.3 0x40:1.9 0x60:2.5 0x80:4.0 0xA0:4.7 0xC0:5.6 0xE0:8.1
#define SENSOR_GAIN 0x20    // +/- 1.3  Ga (default)

#define OP_MODE_CONTINUOUS 0x00 // Continuous conversion
#define OP_MODE_SINGLE     0x01 // Single conversion
#define STATUS_RDY         0x01 // Data Ready

#define HMC5883_ADDR 0x3C
#define HMC_FCT 0.1f

typedef struct
{
    I2C_HandleTypeDef *hi2c;
	char *name;
    s16 FiltNum;
    char *Dir;       
    s16 HmcOff[3];
}sHMC_CFG;

//ʵ��ֵ���¶ȵ�λΪ���϶ȣ�������Ϊ��/s,���ٶȼ�Ϊm/s����ѹΪV���߶�Ϊm��
typedef struct
{
	I2C_HandleTypeDef *hi2c;  //�˿�  --
	char *name;       
    sTIM  Tim;        //��ʱ��
    sCNT  Filt;       //��ֵ�˲���  --
    s8    Dir[6];     //����  --
    s16   HmcOff[3];  //Ư����  --
    s16   HmcRaw[3];  //ԭʼֵ
    s16   HmcRot[3];  //��תֵ
    float HmcRel[3];  //ʵ��ֵ
	//�û���������
	bool  Update;     //����  --
    eSTA  Sta;        //״̬  --
    eERR  Err;        //������Ϣ  --
	u32   Time;       //��ʱ
	bool  HmcCal;     //У׼  --
    float HmcFil[3];  //�˲�ֵ
}sHMC;

#define HMC_NUM 1
extern sHMC hmc[HMC_NUM];

void Hmc_Init_Para(sHMC *ele,sHMC_CFG *elecfg);
bool Hmc_Init(sHMC *ele);
bool Hmc_Read(sHMC *ele);
bool Hmc_Calc(sHMC *ele);

#endif


