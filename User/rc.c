#include "rc.h"

/***************************************************\
功能：
  遥控器 串口读取
说明：
  1、修改端口,并注释掉HAL库该串口ISR
  2、每次读取完数据需要将Update复位
  3、协议$开头 \n结尾
  4、遥控器为保证数据的可靠性，可启用接收中断
     若使用DMA需要配置XBEE
  5、1-4通道反向，5-8通道正向
	 
通道1——副翼     1088->1500->1912
通道2——升降     1088->1500->1912
通道3——油门     1088->1500->1912    1000->1420->1800
通道4——方向     1088->1500->1912
通道5——锁尾模式     946    ->   2054
通道6——总距     
通道7——控制模式
通道8——辅助模式

     CH2         CH3  
     ↑           ↑  
CH4← ◆ →    CH1← ◆ →  min
     ↓           ↓ min 
\***************************************************/

sRC rc[RC_NUM];
struct PCM_XZ pcm_xz={1,1,1,0,0};
/******************驱动程序****************/
void UART4_IRQHandler(void)        //空闲中断+DMA          	
{ 
	sRC *ele=rc;
	if((__HAL_UART_GET_FLAG(ele->huart,UART_FLAG_IDLE)==RESET))return;
	__HAL_UART_CLEAR_IDLEFLAG(ele->huart);
	__HAL_DMA_DISABLE(ele->huart->hdmarx); 
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmarx);
	ele->RxRawIndex=RC_RX_LEN-__HAL_DMA_GET_COUNTER(ele->huart->hdmarx);
	do{
		if(ele->RxRawDat[0]!='$' || ele->RxRawDat[ele->RxRawIndex-1]!='\n')break;
		ele->RxRawHead = true;
		if(ele->LockRx == HAL_LOCKED)break;
		ele->LockRx = HAL_LOCKED;
		if(ele->RxRawDat[1]=='$')memcpy(ele->RxDat,&ele->RxRawDat[1],PPM_NUM);
		else memcpy(ele->RxDat,ele->RxRawDat,PPM_NUM);
		ele->RxDat[ele->RxRawIndex]=0;
		ele->LockRx = HAL_UNLOCKED;
		ele->RxFlag = true;   //收到完整一帧
		ele->RxRawHead = false;
	}while(0);
	__HAL_DMA_SET_COUNTER(ele->huart->hdmarx,RC_RX_LEN);
	__HAL_DMA_ENABLE(ele->huart->hdmarx);
}

void DMA1_Stream4_IRQHandler(void)  //发送DMA中断
{
	sRC *ele=rc;
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmatx);
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->TxFlag = false;
}

bool UART4_Transmit_DMA(u8 *pData, u16 Size)  //DMA发送
{
	sRC *ele=rc;
	if(ele->TxFlag == true)return false;
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->huart->hdmatx->Instance->NDTR = Size;
    ele->huart->hdmatx->Instance->M0AR = (u32)pData;
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmatx);
	__HAL_DMA_ENABLE(ele->huart->hdmatx);
	ele->TxFlag = true;
	return true;
}

/******************功能函数****************/
void Rc_Init_Para(sRC *ele,sRC_CFG *elecfg)
{
	ele->Update = false;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	
	ele->huart = elecfg->huart;
	ele->LockRx = HAL_UNLOCKED;
	ele->name = elecfg->name;
	ele->RxFlag = false;
	ele->RxRawIndex = 0;
	ele->RxRawHead = false;
	
	ele->Chk.CNT = 0;
	ele->Chk.CCR = elecfg->CheckNum;
	ele->Mode = RC_NONE;
	ele->ModeChk.CNT = 0;
	ele->ModeChk.CCR = elecfg->ModeNum;
	ele->Time = 0;
	ele->LOW_THR = elecfg->LOW_THR;
	ele->MID_THR = elecfg->MID_THR;
	ele->HIG_THR = elecfg->HIG_THR;
	
	ele->Ang[0] = 0.0f;  
	ele->Ang[1] = 0.0f;  
	ele->dAng   = 0.0f;  
	ele->Thr    = 0.0f;  
	ele->Key[0] = 0;
	ele->Key[1] = 0;
	ele->Key[2] = 1;
	ele->Key[3] = 2;
	
	ele->Val[0] = 0;
	ele->Val[1] = 0;
	ele->Val[2] = 0;
	ele->Val[3] = 0;
	
	float fa,fb,fc;
	fa = ele->LOW_THR/1000.0f;
	fb = ele->MID_THR/1000.0f;
	fc = ele->HIG_THR/1000.0f;
	ele->a = -(4*(5*fc - 9*fa + 5*fa*fb - 5*fb*fc + 6))/(5*(2*fa - 3)*(3*fa - 3*fc - 2*fa*fc + 2*fc*fc));
	ele->b = (4*(5*fa*fa*fb - 5*fb*fc*fc - 9*fa*fa + 5*fc*fc + 9))/(5*(2*fa - 3)*(3*fa - 3*fc - 2*fa*fc + 2*fc*fc));
	ele->c = -(20*fb*fa*fa*fc - 54*fa*fa - 20*fb*fa*fc*fc + 81*fa + 30*fc*fc - 45*fc)/(5*(2*fa - 3)*(3*fa - 3*fc - 2*fa*fc + 2*fc*fc));
}

bool Rc_Init(sRC *ele)
{
	__HAL_DMA_DISABLE(ele->huart->hdmarx);
	ele->huart->hdmarx->Instance->PAR = (u32)&ele->huart->Instance->RDR;
	ele->huart->hdmarx->Instance->NDTR = RC_RX_LEN;
	ele->huart->hdmarx->Instance->M0AR = (u32)ele->RxRawDat;
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmarx);
	__HAL_DMA_ENABLE(ele->huart->hdmarx);
	SET_BIT(ele->huart->Instance->CR3, USART_CR3_DMAR);
	
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->huart->hdmatx->Instance->PAR = (u32)&ele->huart->Instance->TDR;
	ele->huart->hdmatx->Instance->NDTR = RC_TX_LEN;
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
	HAL_Delay(50);  //等待信号来临
	if(ele->RxFlag==false)
	{
		Dprintf("--Rcv [NO]\r\n");
		return false;
	}
	Dprintf("--Rcv [OK]\r\n");
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

bool Rc_Calc(sRC *ele)
{
	Tim_Calc(&ele->Tim);   //计时
	static u8 PackCnt = 0;
	static u8 PackNum = 0;
	if(PackCnt < 50)PackCnt++;
	else 
	{
		PackCnt = 0;
		ele->Pack = PackNum;
		PackNum = 0;
	}
	if(ele->RxFlag == false)
	{
		if(ele->Chk.CNT<ele->Chk.CCR)ele->Chk.CNT++;
		else ele->Err = ERR_UPDT;
		return false;      //未更新
	}
	ele->RxFlag = false;
	ele->Chk.CNT = 0;
	
	if(ele->LockRx == HAL_LOCKED)return false;
	ele->LockRx = HAL_LOCKED;
	do{
		if(ele->RxDat[0]=='$' && ele->RxDat[PPM_NUM-2]=='\r' && ele->RxDat[PPM_NUM-1]=='\n')
		{		
			PackNum ++;
			char CHN[4]="000";
			char *pStr;
			u16 tmp[PWM_RC_NUM];
			for(u8 i=0;i<PWM_RC_NUM;i++)
			{
				strncpy(CHN,ele->RxDat+i*3+1,3);
				tmp[i]=strtol(CHN,&pStr,16) + 420;   //十六进制转换 +420偏置
			}
			//遥控器行为定义 油门2
			static u8 preFlag = 0xFF;
			u8 Chn_Flag_L = 0x00,Chn_Flag_H = 0x00;
			if(tmp[0] < RC_FUN_MIN)Chn_Flag_L|=0x01;
			else if(tmp[0] > RC_FUN_MAX)Chn_Flag_H|=0x01;
			if(tmp[1] < RC_FUN_MIN)Chn_Flag_L|=0x02;
			else if(tmp[1] > RC_FUN_MAX)Chn_Flag_H|=0x02;
			if(tmp[2] < RC_FUN_MIN)Chn_Flag_L|=0x04;
			else if(tmp[2] > RC_FUN_MAX)Chn_Flag_H|=0x04;
			if(tmp[3] < RC_FUN_MIN)Chn_Flag_L|=0x08;
			else if(tmp[3] > RC_FUN_MAX)Chn_Flag_H|=0x08;
			if((Chn_Flag_L|Chn_Flag_H)==0x0F)  //行为持续一定时间视为有效
			{
				if(preFlag == Chn_Flag_H)
				{
					if(++ele->ModeChk.CNT>ele->ModeChk.CCR)
						ele->Mode = (eRC_MODE)Chn_Flag_H;
				}
				else
				{
					preFlag = Chn_Flag_H;
					ele->ModeChk.CNT = 0;
				}					
			}
			else preFlag = 0xFF;

			for(u8 i=0;i<PWM_RC_NUM;i++) ele->PPM[i] = tmp[i];
			ele->Val[0] = RC_PPM_MID-ele->PPM[3];
			ele->Val[1] = RC_PPM_MID-ele->PPM[1];
			ele->Val[2] = -(RC_PPM_MID-ele->PPM[0]);
			ele->Val[3] = ele->PPM[2];
			//角度设定
			ele->Ang[0] = ele->Val[0]*0.1f;   //-50°到50°
			ele->Ang[1] = ele->Val[1]*0.1f;   //-50°到50°
			ele->dAng   = ele->Val[2]*0.5f;   //-250°/s到250°/s
			ele->Thr    = ele->a*SQR(ele->PPM[2])/1000.0f + ele->b*ele->PPM[2] + ele->c*1000.0f;//油门换算
			//位置设定
			ele->Uvw[0] = ele->Val[0]*0.0008f; //40m/s
			ele->Uvw[1] = ele->Val[1]*0.0008f; //40m/s
			ele->Uvw[2] = ele->Val[3]*0.0008f; //40m/s
			
			ele->Key[0] = ele->PPM[4]<RC_FUN_MIN?0:(ele->PPM[4]>RC_FUN_MAX?2:1);
			ele->Key[1] = ele->PPM[5]<RC_FUN_MIN?0:(ele->PPM[5]>RC_FUN_MAX?2:1);
			ele->Key[2] = ele->PPM[6]<RC_FUN_MIN?1:(ele->PPM[6]>RC_FUN_MAX?3:2);//控制模式:1、手动 2、姿态 3、定点(可自行设置)
			ele->Key[3] = ele->PPM[7]<RC_FUN_MIN?0:1;    //0 变速模式  1恒速模式
			ele->Update = true;
		}
	}while(0);
	ele->LockRx = HAL_UNLOCKED;
	
	Tim_Calc(&ele->Tim);   //计时
	ele->Time = ele->Tim.OUT;
	return ele->Update;
}

void PCM_Fail_Safe(sRC *ele)
{
	pcm_xz.Right_Rate = (pcm_xz.Right_Rate*29+(ele->Update==1))/30.0f;
	pcm_xz.Right_Rate = fConstrain(pcm_xz.Right_Rate,0,1);
	
	pcm_xz.restart_count = pcm_xz.Right_Rate <= 0.2f? pcm_xz.restart_count + 1 : 0;
	pcm_xz.RTL_Init_Flag = pcm_xz.restart_count >=100? 1 : 0;
			
//    if((pcm_xz.RTL_Init_Flag) && (!pcm_xz.RTL_Start_Flag) && (trajectory.heli_step == 1) && (target_queue.is_queue_empty()) && (trajectory.hover_time <=-1) && (position.current_xyz[2] <= -0.2))
//    {
//        trajectory.trajectory_mod = RTL;
//        trajectory.RTL_Orignal_update_flag = 0;
//        pcm_xz.RTL_Start_Flag = 1;
//        control_mode = POSITION_HOLD;
//    }
	if(ele->Update)  ele->Update=false;
	if(pcm_xz.Right_Rate>0.55f) pcm_xz.PCM_Safe = true;
	else pcm_xz.PCM_Safe = false;
}
