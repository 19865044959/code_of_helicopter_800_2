#ifndef __COMMAND_H
#define __COMMAND_H

#include "userlib.h"

#define CMD_RX_LEN 200   //30
#define CMD_TX_LEN 100   //30

typedef struct
{
	UART_HandleTypeDef *huart;
	char *name;
	s16   CheckNum;
}sCMD_CFG;

typedef struct
{
	UART_HandleTypeDef *huart;
	HAL_LockTypeDef LockRx;
	char *name;
	u8   RxRawDat[CMD_RX_LEN];   //����
	u16  RxRawIndex;
	bool RxFlag;      //���ձ�־λ
	u8   RxDat[CMD_RX_LEN];
	bool TxFlag;
	u8   TxDat[CMD_TX_LEN];
	
	sTIM  Tim;        //��ʱ��
	sCNT  Chk;        //���
	//�û�����
	bool  Update;     //����  --
    eSTA  Sta;        //״̬  --
    eERR  Err;        //������Ϣ  --
	u32   Time;       //��ʱ
}sCMD;

#define CMD_NUM 1

extern sCMD cmd[CMD_NUM];

bool UART7_Transmit_DMA(u8 *pData, u16 Size);
void DMAprintf(const char * _format,...) __attribute__((format(printf,1,2)));

void Cmd_Init_Para(sCMD *ele,sCMD_CFG *elecfg);
bool Cmd_Init(sCMD *ele);
bool Cmd_Calc(sCMD *ele);

#endif
