#include "gps.h"

/***************************************************\
功能：
  GPS 串口读取
说明：
  1、修改端口,并注释掉HAL库该串口ISR
  2、每次读取完数据需要将Update复位
  3、协议$开头 \n结尾
  4、接收采用DMA和空闲中断
\***************************************************/
sGPS gps[GPS_NUM];
/******************驱动程序****************/
void USART1_IRQHandler(void)        //空闲中断+DMA        	
{ 
	sGPS *ele=gps;
	if((__HAL_UART_GET_FLAG(ele->huart,UART_FLAG_IDLE)==RESET))return;
	__HAL_UART_CLEAR_IDLEFLAG(ele->huart);
	__HAL_DMA_DISABLE(ele->huart->hdmarx); 
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmarx);
	ele->RxRawIndex=GPS_RX_LEN-__HAL_DMA_GET_COUNTER(ele->huart->hdmarx);
	do{
		if(ele->RxRawDat[0]!='$' || ele->RxRawDat[ele->RxRawIndex-1]!='\n')break;
		if(ele->LockRx == HAL_LOCKED)break;
		ele->LockRx = HAL_LOCKED;
		memcpy(ele->RxDat,ele->RxRawDat,ele->RxRawIndex);
		ele->RxDat[ele->RxRawIndex]=0;
		ele->LockRx = HAL_UNLOCKED;
		ele->RxFlag = true;   //收到完整一帧
	}while(0);
	__HAL_DMA_SET_COUNTER(ele->huart->hdmarx,GPS_RX_LEN);
	__HAL_DMA_ENABLE(ele->huart->hdmarx);
}

/******************功能函数****************/
void Gps_Init_Para(sGPS *ele,sGPS_CFG *elecfg)
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
	ele->Time = 0;
	
	ele->MagUpdate = false;
	ele->MagCal = false;
	ele->MagErr = ERR_NONE;
	ele->MagSta = STA_INI;
	if(Dir_Trans(ele->Dir,elecfg->Dir)==false)ele->Err = ERR_SOFT;
	for(u8 i=0;i<3;i++)
	{
		ele->MagOff[i] = elecfg->MagOff[i];
		ele->Re2t[i][0] = 0.0;ele->Re2t[i][1] = 0.0;ele->Re2t[i][2] = 0.0;
	}
	ele->ECEF_Init_Flag = false;
	ele->GPS_INS_EKF_flag = false;
	ele->GPS_INS_EKF_start_flag = false;
	
	for(u8 i = 0;i<3;i++){
		ele->NED[i] = 0.0f;
		ele->NED_spd[i] = 0.0f;
	}
	
}

bool Gps_Init(sGPS *ele)
{
	__HAL_DMA_DISABLE(ele->huart->hdmarx);
	ele->huart->hdmarx->Instance->PAR = (u32)&ele->huart->Instance->RDR;
	ele->huart->hdmarx->Instance->NDTR = GPS_RX_LEN;
	ele->huart->hdmarx->Instance->M0AR = (u32)ele->RxRawDat;
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmarx);
	__HAL_DMA_ENABLE(ele->huart->hdmarx);
	SET_BIT(ele->huart->Instance->CR3, USART_CR3_DMAR);
	
	__HAL_UART_CLEAR_IT(ele->huart,UART_CLEAR_FEF|UART_CLEAR_IDLEF|UART_CLEAR_NEF);
	__HAL_UART_ENABLE_IT(ele->huart,UART_IT_IDLE);	

	Dprintf("\r\n%s Init...\r\n",ele->name);
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
	ele->Sta = STA_RUN;
	ele->MagSta = STA_RUN;
	ele->GpsCal = true;  //gps必须校准原点
	HAL_Delay(250);  //等待信号来临
	if(ele->RxFlag==false)
	{
		Dprintf("--Rcv [NO]\r\n");
		return false;
	}
	Dprintf("--Rcv [OK]\r\n");
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

bool Gps_Cali(sGPS *ele)
{
	static bool GpsIsCali = false;
	if(ele->GpsCal==true)   
	{
		ele->GpsCal = false;
		ele->Sta = STA_CAL;
		GpsIsCali = true;
		Dprintf("\r\n%s Cali\r\n",ele->name);
	}
	if(GpsIsCali == false)return true;
	static u16 Cali=0;  //下次进来就不用等待了
	if(ele->star<=5)
	{
		if((Cali<100)){Cali++;return false;}
	}
	/****计算位置*****/
	ele->LLH_Init[0] = ele->lat*D2R;         //纬度
	ele->LLH_Init[1] = ele->lng*D2R;         //经度
	ele->LLH_Init[2] = exitx->DatRel[2];//高度，注意是该减还是加
	//LLH换算成ECFF（地球中心坐标系）
	ele->N_Init = C_WGS84_a/sqrt( 1.0 - SQR(C_WGS84_e)*SQR(sin(ele->LLH_Init[0])));
	ele->ECFF_Init[0]= (ele->N_Init + ele->LLH_Init[2]) * cos(ele->LLH_Init[0]) * cos(ele->LLH_Init[1]);
	ele->ECFF_Init[1]= (ele->N_Init + ele->LLH_Init[2]) * cos(ele->LLH_Init[0]) * sin(ele->LLH_Init[1]);
	ele->ECFF_Init[2]= (ele->N_Init * (1-SQR(C_WGS84_e)) +ele->LLH_Init[2]) * sin(ele->LLH_Init[0]);
	ele->Zero_ECFF[0] = ele->ECFF_Init[0];
	ele->Zero_ECFF[1] = ele->ECFF_Init[1];
	ele->Zero_ECFF[2] = ele->ECFF_Init[2];
	//产生Re2t矩阵  ，ECFF转NED矩阵，因为没飞那么远，认为该阵不变
	double clat=cos(ele->lat*D2R),slat=sin(ele->lat*D2R),clng=cos(ele->lng*D2R),slng=sin(ele->lng*D2R);
	ele->Re2t[0][0] = -slat*clng;
	ele->Re2t[0][1] = -slat*slng;
	ele->Re2t[0][2] =  clat;
	ele->Re2t[1][0] = -slng;
	ele->Re2t[1][1] =  clng;
	ele->Re2t[1][2] =  0.0;
	ele->Re2t[2][0] = -clat*clng;
	ele->Re2t[2][1] = -clat*slng;
	ele->Re2t[2][2] = -slat;
	ele->Sta=STA_RUN;
	GpsIsCali = false;
	ele->ECEF_Init_Flag = true;
	return true;
}

static bool Gps_MagCali(sGPS *ele)  //八字立体校准
{
	static bool MagIsCali = false;
	if(ele->MagCal==true)   
	{
		ele->MagCal = false;
		ele->MagSta = STA_CAL;
		MagIsCali = true;
		Dprintf("\r\n%s Mag Cali\r\n",ele->name);
	}
	if(MagIsCali == false)return true;
	#define SAMPLE_NUM 400  //10s
	static float d[SAMPLE_NUM][3];
	static u16 SmpCnt = 0;
	if(SmpCnt<SAMPLE_NUM)
	{
		d[SmpCnt][0] = (float)ele->MagRaw[0];
		d[SmpCnt][1] = (float)ele->MagRaw[1];
		d[SmpCnt][2] = (float)ele->MagRaw[2];
		SmpCnt++;
		return false;
	}
	SmpCnt = 0;
	u16 Pop[2][3];
	float Origin[3],Radius;
	sphereFit(d, SAMPLE_NUM, 100, 0.0f, Pop, Origin, &Radius);
	Dprintf("\r\n%s Mag Cali--Bias is:%4.2f %4.2f %4.2f,Radius is:%4.2f\r\n",ele->name,Origin[0],Origin[1],Origin[2],Radius);
	if(Radius>300 && Radius<500)
	{
		ele->MagOff[0]=Origin[0];
		ele->MagOff[1]=Origin[1];
		ele->MagOff[2]=Origin[2];
	}
	ele->MagSta = STA_RUN;
	MagIsCali = false;
	return true;
}

bool Gps_Calc(sGPS *ele)
{
	if(ele->Sta == STA_INI)return false;
	Tim_Calc(&ele->Tim);   //计时
	if(ele->RxFlag == false)
	{
		if(ele->Chk.CNT<ele->Chk.CCR)ele->Chk.CNT++;
		else 
		{
			ele->Err = ERR_UPDT;
			ele->MagErr = ERR_UPDT;
		}
		return false;
	}
	ele->RxFlag = false;
	ele->Chk.CNT = 0;
	
	if(ele->LockRx == HAL_LOCKED)return false;
	ele->LockRx = HAL_LOCKED;
	bool GpsFlag = false,MagFlag = false;
	do{
		char *RxType[4];
		RxType[0]=(char*)ele->RxDat-1;  //第一个存放RxDat的前一个以便和下面的格式相同
		u8 num = StrSeg((char*)ele->RxDat,'\n',&RxType[1],3);
		if(num==0)break;   //分割不同类型，这里只有$GPGGA,$GPVTG,$HMC,$GPS 4种
		for(u8 i=0;i<num;i++)
		{
			char *RxPara[17];  
			char Dat[GPS_RX_LEN];
			if(*(RxType[i]+4)=='G')
			{
				//$GPGGA,102134.00,1306.9847,N,11402.2986,W,2,5,1.0,50.3,M,-16.27,M,,*61\r\n  
				memcpy(Dat,RxType[i]+1,RxType[i+1]-RxType[i]);
				if(StrSeg(Dat,',',RxPara,14)!=14)break;   
				ele->time=atof(RxPara[0]);         //第二位为时间，格式为hhmmss.ss
				int time_int=(int)ele->time;
				ele->time = (int)(time_int/10000*3600+time_int%10000/100*60+time_int%100)+(ele->time-time_int);   //时分秒转化为秒
				ele->lat = atof(RxPara[1]);        //第三位纬度，格式为ddmm.mmmm
				int lat_int=(int)ele->lat;
				ele->lat = (int)(lat_int/100)+atof(RxPara[1]+2)/60.0f;
				if(*(RxPara[2])=='S') ele->lat=-ele->lat;    //第四位纬度，N或S(北纬或南纬)
				ele->lng = atof(RxPara[3]);         //第五位经度，格式为dddmm.mmmm
				int lng_int=(int)ele->lng;
				ele->lng = (int)(lng_int/100)+atof(RxPara[3]+3)/60.0f;
				if(*(RxPara[4])=='W') ele->lng=-ele->lng;    //第六位经度，E或W(东经或西经)
				ele->status=*(RxPara[5]);           //第七位GPS状态
				ele->star=atoi(RxPara[6]);          //第八位星数
				ele->hdop=atof(RxPara[7]);          //第九位水平精度
				ele->alti=atof(RxPara[8]);          //第十位椭球高
				ele->wgs_alt=atoi(RxPara[10]);      //第十二位水准平面高
				GpsFlag = true;
			}
			if(*(RxType[i]+4)=='V')
			{
				//$GPVTG,<1>,T,<2>,M,<3>,N,<4>,K,<5>*hh\r\n
				memcpy(Dat,RxType[i]+1,RxType[i+1]-RxType[i]);
				if(StrSeg(Dat,',',RxPara,9)!=9)break;   
				ele->vtg_dir = atof(RxPara[0]);     //第二位为真北方向
				ele->vtg_spd = atof(RxPara[6]);     //第八位为地面速率，单位：公里/小时
				GpsFlag = true;
			}
			if(*(RxType[i]+4)=='C')
			{
				//$HMC,mx,my,mz\r\n
				memcpy(Dat,RxType[i]+1,RxType[i+1]-RxType[i]);
				if(StrSeg(Dat,',',RxPara,3)!=3)break;
				ele->MagRaw[0] = atoi(RxPara[0]);
				ele->MagRaw[1] = atoi(RxPara[1]);
				ele->MagRaw[2] = atoi(RxPara[2]);
				MagFlag = true;
			}
			if(*(RxType[i]+4)=='S')
			{
				//$GPS,%1d%1d,%d,%d,%d,%f,%f,%f,%f,%d\r\n\r\n
				memcpy(Dat,RxType[i]+1,RxType[i+1]-RxType[i]);
				if(StrSeg(Dat,',',RxPara,17)!=17)break;
				if(*(RxPara[0])=='1'){
					MagFlag=true;
					ele->MagRaw[0] = atoi(RxPara[1]);
					ele->MagRaw[1] = atoi(RxPara[2]);
					ele->MagRaw[2] = atoi(RxPara[3]);
					ele->MagUpdate = true;
				}
				else
				{
					ele->MagUpdate = false;
				}
				static int i;
				static int record[100];
				if(*(RxPara[0]+1)=='1'){
					sTIM tim_tmp;
					GpsFlag=true;
					gps->lat = atof(RxPara[4]);
					gps->lng = atof(RxPara[5]);
					gps->NED_spd[0] = atof(RxPara[6]);
					gps->NED_spd[1] = atof(RxPara[7]);
					gps->NED_spd[2] = atof(RxPara[8]);
					gps->alti = atof(RxPara[9]);
					gps->star = atof(RxPara[10]);
				
					gps->status = atoi(RxPara[16]);
					gps->GPSUpdate = 1;
				
					Tim_Calc(&tim_tmp);
					gps->gps_update_time = tim_tmp.CNT;
					if(i<100)
					{
						record[i] =  gps->gps_update_time;
						i++;
					}
					
				}
			}
		}
	}while(0);
	ele->LockRx = HAL_UNLOCKED;

	if(MagFlag==true)
	{
		Gps_MagCali(ele);
		//方向变换
		ele->MagRot[0]=ele->Dir[0]*(ele->MagRaw[ele->Dir[1]]-ele->MagOff[ele->Dir[1]]);
		ele->MagRot[1]=ele->Dir[2]*(ele->MagRaw[ele->Dir[3]]-ele->MagOff[ele->Dir[3]]);
		ele->MagRot[2]=ele->Dir[4]*(ele->MagRaw[ele->Dir[5]]-ele->MagOff[ele->Dir[5]]);
		//实际值转换
		ele->MagFil[0]=ele->MagRel[0]=ele->MagRot[0];
		ele->MagFil[1]=ele->MagRel[1]=ele->MagRot[1];
		ele->MagFil[2]=ele->MagRel[2]=ele->MagRot[2];
		
		if(ele->MagSta == STA_RUN)ele->MagUpdate = true;
	//	float sqrNorm=SQR(ele->MagFil[0])+SQR(ele->MagFil[1])+SQR(ele->MagFil[2]);
	//	if(sqrNorm>360000||sqrNorm<40000)ele->MagUpdate = false;
	}
	
	if(GpsFlag == true)
	{
		//是否收到有效数据
		if(ele->star<6)
		{
			ele->Err = ERR_UPDT;
			return false;
		}
		//初始化ECEF坐标系
		Gps_Cali(ele);	
		if(ele->Sta==STA_RUN)ele->Update = true;
	}
	Tim_Calc(&ele->Tim);   //计时
	ele->Time = ele->Tim.OUT;
	return ele->Update;
}


