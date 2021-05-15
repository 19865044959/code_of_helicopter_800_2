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
	u8   RxRawDat[CMD_RX_LEN];   //数据
	u16  RxRawIndex;
	bool RxFlag;      //接收标志位
	u8   RxDat[CMD_RX_LEN];
	bool TxFlag;
	u8   TxDat[CMD_TX_LEN];
	
	sTIM  Tim;        //计时器
	sCNT  Chk;        //检测
	//用户数据
	bool  Update;     //更新  --
    eSTA  Sta;        //状态  --
    eERR  Err;        //错误信息  --
	u32   Time;       //计时
}sCMD;

#define CMD_NUM 1

extern sCMD cmd[CMD_NUM];

bool UART7_Transmit_DMA(u8 *pData, u16 Size);
void DMAprintf(const char * _format,...) __attribute__((format(printf,1,2)));

void Cmd_Init_Para(sCMD *ele,sCMD_CFG *elecfg);
bool Cmd_Init(sCMD *ele);
bool Cmd_Calc(sCMD *ele);

#endif
