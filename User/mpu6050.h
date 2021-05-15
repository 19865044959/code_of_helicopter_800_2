#ifndef __MPU6050_H
#define __MPU6050_H

#include "userlib.h"

// Registers
#define MPU6050_SMPLRT_DIV          0x19U
#define MPU6050_CONFIG              0x1AU
#define MPU6050_GYRO_CONFIG         0x1BU
#define MPU6050_ACCEL_CONFIG        0x1CU
#define MPU6050_FIFO_EN             0x23U
#define MPU6050_INT_PIN_CFG         0x37U
#define MPU6050_INT_ENABLE          0x38U
#define MPU6050_INT_STATUS          0x3AU
#define MPU6050_ACCEL_XOUT_H        0x3BU
#define MPU6050_ACCEL_XOUT_L        0x3CU
#define MPU6050_ACCEL_YOUT_H        0x3DU
#define MPU6050_ACCEL_YOUT_L        0x3EU
#define MPU6050_ACCEL_ZOUT_H        0x3FU
#define MPU6050_ACCEL_ZOUT_L        0x40U
#define MPU6050_TEMP_OUT_H          0x41U
#define MPU6050_TEMP_OUT_L          0x42U
#define MPU6050_GYRO_XOUT_H         0x43U
#define MPU6050_GYRO_XOUT_L         0x44U
#define MPU6050_GYRO_YOUT_H         0x45U
#define MPU6050_GYRO_YOUT_L         0x46U
#define MPU6050_GYRO_ZOUT_H         0x47U
#define MPU6050_GYRO_ZOUT_L         0x48U
#define MPU6050_USER_CTRL           0x6AU
#define MPU6050_PWR_MGMT_1          0x6BU
#define MPU6050_PWR_MGMT_2          0x6CU
#define MPU6050_FIFO_COUNTH         0x72U
#define MPU6050_FIFO_COUNTL         0x73U
#define MPU6050_FIFO_R_W            0x74U
#define MPU6050_WHOAMI              0x75U

// Bits
#define BIT_SLEEP                   0x40U
#define BIT_H_RESET                 0x80U
#define BITS_CLKSEL                 0x07U
#define MPU_CLK_SEL_PLLGYROX        0x01U
#define MPU_CLK_SEL_PLLGYROZ        0x03U
#define MPU_EXT_SYNC_GYROX          0x02U
#define BITS_FS_250DPS              0x00U
#define BITS_FS_500DPS              0x08U
#define BITS_FS_1000DPS             0x10U
#define BITS_FS_2000DPS             0x18U
#define BITS_FS_2G                  0x00U
#define BITS_FS_4G                  0x08U
#define BITS_FS_8G                  0x10U
#define BITS_FS_16G                 0x18U
#define BITS_FS_MASK                0x18U
#define BITS_DLPF_CFG_256HZ         0x00U
#define BITS_DLPF_CFG_188HZ         0x01U
#define BITS_DLPF_CFG_98HZ          0x02U
#define BITS_DLPF_CFG_42HZ          0x03U
#define BITS_DLPF_CFG_20HZ          0x04U
#define BITS_DLPF_CFG_10HZ          0x05U
#define BITS_DLPF_CFG_5HZ           0x06U
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07U
#define BITS_DLPF_CFG_MASK          0x07U
#define BIT_INT_ANYRD_2CLEAR        0x10U
#define BIT_RAW_RDY_EN              0x01U
#define BIT_I2C_IF_DIS              0x10U
#define BIT_INT_STATUS_DATA         0x01U

#define MPU6050_ADDR  0xD0
#define IMU_ACC_FCT 0.00119708f  // (1/8192) * 9.8065  (8192 LSB = 1 G)
#define IMU_GYR_FCT 0.00106585f  // (1/65.5) * 4  (65.5 LSB = 1 RPS)
//#define IMU_GYR_FCT 0.06106870f  // (1/65.5) * 4  (65.5 LSB = 1 DPS)

typedef struct
{
    I2C_HandleTypeDef *hi2c;
	char *name;
    s16 FiltNum;
    char *Dir;       
    float OneG;
    s16 AccOff[3];
    float GyrOffO[3]; //25��
	float GyrOffT[3]; 
}sIMU_CFG;

//ʵ��ֵ���¶ȵ�λΪ���϶ȣ�������Ϊrad/s,���ٶȼ�Ϊm/s����ѹΪV���߶�Ϊm��

typedef struct
{
	I2C_HandleTypeDef *hi2c;  //�˿�  --
	char *name;       
    sTIM  Tim;        //��ʱ��
    sCNT  Filt;       //��ֵ�˲���  --
    s8    Dir[6];     //����  --
	u8    AccCalFlag;
    s16   AccOff[3];  //Ư����  --
	s16   GyrOff[3];  //Ư����  --
    float GyrOffO[3]; //��Ʈ  --
	float GyrOffT[3]; //��Ʈ  --
    s16   AccRaw[3];  //ԭʼֵ
    s16   GyrRaw[3];  //ԭʼֵ
    s16   TmpRaw[1];  //�¶�ԭʼֵ
    s16   AccRot[3];  //��תֵ
    s16   GyrRot[3];  //��תֵ
    float AccRel[3];  //ʵ��ֵ
    float GyrRel[3];  //ʵ��ֵ
    float TmpRel[1];     //�¶�ʵ��ֵ
	//�û���������
	bool  Update;     //����  --
    eSTA  Sta;        //״̬  --
    eERR  Err;        //������Ϣ  --
	u32   Time;       //��ʱ
	bool  GyrCal;     //У׼  --
	bool  AccCal;     //У׼  --
    float OneG;       //����  --
    float AccFil[3];  //�˲�ֵ
    float GyrFil[3];  //�˲�ֵ
    float TmpFil[1];  //�˲�ֵ
}sIMU;

#define IMU_NUM 1

extern sIMU imu[IMU_NUM];

void Imu_Init_Para(sIMU *ele,sIMU_CFG *elecfg);
bool Imu_Init(sIMU *ele);
bool Imu_Read(sIMU *ele);
bool Imu_Calc(sIMU *ele);

#endif

