#include "command.h"
#include "sdcard.h"

/***************************************************\
���ܣ�
  �������� ���ڶ�ȡ
˵����
  1���޸Ķ˿�,��ע�͵�HAL��ô���ISR
  2��ÿ�ζ�ȡ��������Ҫ��Update��λ
  3��Э��$��ͷ \n��β
  4����ȡ����DMA+�����жϣ����Ͳ���DMA�ж�
\***************************************************/

sCMD cmd[CMD_NUM];                  

/******************��������****************/
void UART7_IRQHandler(void)        //�����ж�+DMA          	
{ 
	sCMD *ele=cmd;
	if((__HAL_UART_GET_FLAG(ele->huart,UART_FLAG_IDLE)==RESET))return;
	__HAL_UART_CLEAR_IDLEFLAG(ele->huart);
	__HAL_DMA_DISABLE(ele->huart->hdmarx); 
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmarx);
	ele->RxRawIndex=CMD_RX_LEN-__HAL_DMA_GET_COUNTER(ele->huart->hdmarx);
	//Э�������������Ҫ�޸�
	do{     
		if(ele->RxRawDat[0]!='#' || ele->RxRawDat[ele->RxRawIndex-1]!='\n')break;
		if(ele->LockRx == HAL_LOCKED)break;
		ele->LockRx = HAL_LOCKED;
		memcpy(ele->RxDat,ele->RxRawDat,ele->RxRawIndex);
		ele->RxDat[ele->RxRawIndex]=0;
		ele->LockRx = HAL_UNLOCKED;
		ele->RxFlag = true;   //�յ�����һ֡
	}while(0);
	__HAL_DMA_SET_COUNTER(ele->huart->hdmarx,CMD_RX_LEN);
	__HAL_DMA_ENABLE(ele->huart->hdmarx);
}

void DMA1_Stream1_IRQHandler(void)  //����DMA�ж�
{
	sCMD *ele=cmd;
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmatx);
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->TxFlag = false;
}

bool UART7_Transmit_DMA(u8 *pData, u16 Size)  //DMA����
{
	sCMD *ele=cmd;
	if(ele->TxFlag == true)return false;
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->huart->hdmatx->Instance->NDTR = Size;
    ele->huart->hdmatx->Instance->M0AR = (u32)pData;
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmatx);
	__HAL_DMA_ENABLE(ele->huart->hdmatx);
	ele->TxFlag = true;
	return true;
}

void DMAprintf(const char * _format,...)
{
	sCMD *ele=cmd;
	va_list ap;
	va_start(ap,_format);
	vsnprintf((char *)ele->TxDat,UART_CACHE_NUM,_format,ap);
	va_end(ap);
	UART7_Transmit_DMA(ele->TxDat,strlen((char *)ele->TxDat));
}

/******************���ܺ���****************/
void Cmd_Init_Para(sCMD *ele,sCMD_CFG *elecfg)
{
	ele->Update = false;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	
	ele->huart = elecfg->huart;
	ele->LockRx = HAL_UNLOCKED;
	ele->name = elecfg->name;
	ele->RxFlag = false;
	ele->RxRawIndex = 0;
	
	ele->Chk.CNT = 0;
	ele->Chk.CCR = elecfg->CheckNum;
	ele->Time = 0;
}

bool Cmd_Init(sCMD *ele)
{
	__HAL_DMA_DISABLE(ele->huart->hdmarx);
	ele->huart->hdmarx->Instance->PAR = (u32)&ele->huart->Instance->RDR;
	ele->huart->hdmarx->Instance->NDTR = CMD_RX_LEN;
	ele->huart->hdmarx->Instance->M0AR = (u32)ele->RxRawDat;
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmarx);
	__HAL_DMA_ENABLE(ele->huart->hdmarx);
	SET_BIT(ele->huart->Instance->CR3, USART_CR3_DMAR);

	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->huart->hdmatx->Instance->PAR = (u32)&ele->huart->Instance->TDR;
	ele->huart->hdmatx->Instance->NDTR = CMD_TX_LEN;
	ele->huart->hdmatx->Instance->M0AR = (u32)ele->TxDat;
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmatx);
	__HAL_DMA_ENABLE_IT(ele->huart->hdmatx,DMA_IT_TC);
	SET_BIT(ele->huart->Instance->CR3, USART_CR3_DMAT);

	__HAL_UART_CLEAR_IT(ele->huart,UART_CLEAR_FEF|UART_CLEAR_IDLEF|UART_CLEAR_NEF);
	__HAL_UART_ENABLE_IT(ele->huart,UART_IT_IDLE);	
	
	Dprintf("\r\n%s Init...\r\n",ele->name);
	if(ele->Sta != STA_INI)
	{
		Dprintf("--Par [NO]\r\n");
        return false;
	}
	ele->Sta = STA_RUN;
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

bool Cmd_Calc(sCMD *ele)
{
	Tim_Calc(&ele->Tim);   //��ʱ
	if(ele->RxFlag == false)
	{
		if(ele->Chk.CNT<ele->Chk.CCR)ele->Chk.CNT++;
		else ele->Err = ERR_UPDT;
		return false;
	}
	ele->RxFlag = false;
	ele->Chk.CNT = 0;
	
	if(ele->LockRx == HAL_LOCKED)return false;    //��ֹ�̷ָ߳����ַ��ָպý��յ����ַ������·ָ����
	ele->LockRx = HAL_LOCKED;                     //����һ��Ҫ����
	
	//Э�������������Ҫ�޸���Ӧ����
	do{
		char *RxPara[10];
		if(ele->RxDat[1]=='P')  //$P,x,y,t\r\n
		{
			if(StrSeg((char*)ele->RxDat,',',RxPara,3)!=3)break;
			ele->Update = true;	
		}
		if(ele->RxDat[1]=='S')  
		{
			sd->ReadFlag=atoi((char*)&ele->RxDat[2]);
		}
		if(ele->RxDat[1]=='R')  
		{
			if(strncmp((char*)ele->RxDat+1,"RST",3)==0)
				Sys_Rst();
		}
	}while(0);
	ele->LockRx = HAL_UNLOCKED;
	
	Tim_Calc(&ele->Tim);   //��ʱ
	ele->Time = ele->Tim.OUT;
	return ele->Update;
}

