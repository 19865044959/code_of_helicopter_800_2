#include "transfer.h"

/***************************************************\
功能：
  串口传输管理
说明：
  1、使用匿名科创地面站 
  2、读取采用DMA+空闲中断，发送采用DMA中断
\***************************************************/

#define UPLOAD_NUM 139
char groundtoarm[UPLOAD_NUM];
char put2ground[400]="$DATA";
bool to_ground_transmit_flag=false;
bool receive_index = false;
bool target_receive_flag = false;
double temp;
extern int show_temp_rotate;

sTRAN tran[TRAN_NUM];

void USART2_IRQHandler(void)        //空闲中断           	
{ 
	sTRAN *ele=tran;
	if((__HAL_UART_GET_FLAG(ele->huart,UART_FLAG_IDLE)==RESET))return;
	__HAL_UART_CLEAR_IDLEFLAG(ele->huart);
	__HAL_DMA_DISABLE(ele->huart->hdmarx); 
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmarx);
	ele->RxRawIndex=TRAN_RX_LEN-__HAL_DMA_GET_COUNTER(ele->huart->hdmarx);
	do{  
		if(ele->RxRawDat[0]!='$' || ele->RxRawDat[ele->RxRawIndex-1]!='\n')break;
		if(ele->LockRx == HAL_LOCKED)break;
		ele->LockRx = HAL_LOCKED;
		memcpy(ele->RxDat,ele->RxRawDat,ele->RxRawIndex);
		ele->RxDat[ele->RxRawIndex]=0;
		ele->LockRx = HAL_UNLOCKED;
		receive_index = false;
		ele->RxFlag = true;   //收到完整一帧
	}while(0);
	__HAL_DMA_SET_COUNTER(ele->huart->hdmarx,TRAN_RX_LEN);
	__HAL_DMA_ENABLE(ele->huart->hdmarx);
}

void DMA1_Stream6_IRQHandler(void)  //发送DMA中断
{
	sTRAN *ele=tran;
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmatx);
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->TxFlag = false;
}

bool UART2_Transmit_DMA(u8 *pData, u16 Size)  //DMA发送
{
	sTRAN *ele=tran;
	if(ele->TxFlag == true)return false;
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->huart->hdmatx->Instance->NDTR = Size;
    ele->huart->hdmatx->Instance->M0AR = (u32)pData;
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmatx);
	__HAL_DMA_ENABLE(ele->huart->hdmatx);
	ele->TxFlag = true;
	return true;
}

void Uprintf(const char * _format,...)
{
	sCMD *ele=cmd;
	va_list ap;
	va_start(ap,_format);
	vsnprintf((char *)ele->TxDat,UART_CACHE_NUM,_format,ap);
	va_end(ap);
	UART2_Transmit_DMA(ele->TxDat,strlen((char *)ele->TxDat));
}

//float target[6][4];

void deal_groundtoarm_date(void)
{
	sTRAN *ele=tran;
	if (!strncmp((char*)ele->RxDat,"$TARG",5))
	{
		receive_index=true;
		char *RxType[6];
		char *RxPara[3];
		RxType[0]=(char*)&ele->RxDat[6];  //第一个存放RxDat的前一个以便和下面的格式相同
		if(StrSeg((char*)ele->RxDat,';',&RxType[1],5)!=5)return;
		u8 i;
		for(i=0;i<6;i++){
			if(StrSeg((char*)RxType[i],',',RxPara,3)!=3)return;
			target_command.target[i][0]=atof(RxType[i]);
			target_command.target[i][1]=atof(RxPara[0]);
			target_command.target[i][2]=atof(RxPara[1]);
			target_command.target[i][3]=atof(RxPara[2]);
			if(target_command.target[i][3] == -1) break;
		}
		target_command.count = i;
		target_receive_flag = true;
	}

	else if (!strncmp((char*)ele->RxDat,"$PARA",5))
	{
		receive_index=true;
		char *RxPara[22];
		if(StrSeg((char*)ele->RxDat,',',RxPara,22)!=22)return;
		Heli.auto_roll_control_mode=*(RxPara[0]+0) - '0';
		Heli.auto_pitch_control_mode=*(RxPara[0]+1) - '0';
		Heli.auto_yaw_control_mode=*(RxPara[0]+2) - '0';
		Heli.auto_coll_control_mode=*(RxPara[0]+3) - '0';
		Heli.amplify=atoi(RxPara[1]);
		char CHN[4]="000";
		char *pStr;
		strncpy(CHN,RxPara[2],3);
		Heli.pwm_roll_mid=strtol(CHN,&pStr,16);
		strncpy(CHN,RxPara[2]+3,3);
		Heli.pwm_pitch_mid=strtol(CHN,&pStr,16);
		strncpy(CHN,RxPara[2]+6,3);
		Heli.pwm_yaw_mid=strtol(CHN,&pStr,16);
		strncpy(CHN,RxPara[2]+9,3);
		Heli.pwm_coll_mid=strtol(CHN,&pStr,16);
		pidX->Kp=atof(RxPara[3]);
	  pidVelX->Ki=atof(RxPara[4]);
	  pidVelX->Kp=atof(RxPara[5]);
//    pidmotor->Ki=atof(RxPara[4]);
//		pidmotor->Kp=atof(RxPara[5]);
		pidY->Kp=atof(RxPara[6]);
		pidVelY->Ki=atof(RxPara[7]);
		pidVelY->Kp=atof(RxPara[8]);
		pidZ->Kp=atof(RxPara[9]);
		pidVelZ->Ki=atof(RxPara[10]);
		pidVelZ->Kp=atof(RxPara[11]);
		pidPit1->Kp=atof(RxPara[12]);
		pidPitRate1->Ki=atof(RxPara[13]);
		pidPitRate1->Kp=atof(RxPara[14]);
		pidRol1->Kp=atof(RxPara[15]);
		pidRolRate1->Ki=atof(RxPara[16]);
		pidRolRate1->Kp=atof(RxPara[17]);
		pidYaw1->Kp=atof(RxPara[18]);
		pidAcc_z->Ki=atof(RxPara[19]);
		pidAcc_z->Kp=atof(RxPara[20]);
		temp = atof(RxPara[21]);
		temp=(-1)*(temp-5);
	}
}


void output2ground(void)
{
	static float cnt = 0;
  char i;
  int len=5;
  //int pcm_up_flag;
  //static int err_count=0;
  //if(!to_ground_transmit_flag)return;
	//-------------------degree------------------------------------  
	len += sprintf(put2ground+len,",%.3f",ahrs[1].Ang[0]);//ahrs[1].Ang[0] gps[0].MagRaw[0]*D2R
	//len += sprintf(put2ground+len,"%.3f",roll_commend[0]);

	len += sprintf(put2ground+len,",%.3f",ahrs[1].Ang[1]);//ahrs[1].Ang[1] gps[0].MagRel[1]*D2R

	//len += sprintf(put2ground+len,"%.3f",pitch_commend[0]);
	len += sprintf(put2ground+len,",%.3f",ahrs[1].Ang[2]);//ahrs[1].Ang[2] gps[0].MagRel[2]*D2R
	
	//------------------------roll pitch yaw commend------------
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",Heli.roll_command);//roll_commend[0]
		
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",Heli.pitch_command);//pitch_commend[0]
		
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",Heli.yaw_command);//yaw_commend[0] Given_rate

//-------------------(raw_ahrs_theta)PQR----------------------------------
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.4f",pcm_xz.Right_Rate * D2R);//test_target_rate * D2R   Heli.vel_feedback_ned[0] * D2R
		
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.4f",mot->rotate_raw * D2R);//adis->GyrFil[2] * D2R
		
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.4f",mot->rotate_speed * D2R);//adis->GyrFil_2nd[2] * D2R   mot->rotate_speed * D2R
	
////-------------------accel---------------------------------     
//	put2ground[len++]=',';
//	len += sprintf(put2ground+len,"%.3f",pcm_xz.Right_Rate);//Heli.roll_rate_feedforward * R2D
//	
//	put2ground[len++]=',';
//	len += sprintf(put2ground+len,"%.3f",trajectory.cur_target_vel_ned[0]);//ahrs[0].AccB[1]  desired_p * R2D Heli.yaw_rate_feedforward * R2D 
//	
//	put2ground[len++]=',';
//	len += sprintf(put2ground+len,"%.3f",trajectory.cur_target_vel_ned[1]); //ahrs[0].AccB[2] desired_q * R2D

	
////-------------------accel---------------------------------     
/*	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",test_kp);//Heli.roll_rate_feedforward * R2D Heli.vel_command_ned[0] ahrs[1].pqr[0] * R2D
	
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",test_ki);//ahrs[0].AccB[1]  desired_p * R2D Heli.yaw_rate_feedforward * R2D  Heli.vel_command_ned[1]  ahrs[1].pqr[1] * R2D
	
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",test_target_rate * R2D); //ahrs[0].AccB[2] desired_q * R2D output_data_new.Vel[2]
*/

//-------------------accel---------------------------------     
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",Heli.vel_command_ned[0]);//Heli.vel_command_ned[0] trajectory.cur_target_pos_ned[0] gps[0].MagRel[0]
	
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",Heli.vel_command_ned[1]);//Heli.vel_command_ned[1] trajectory.cur_target_pos_ned[1] pidAcc_z->Kp
	
	put2ground[len++]=',';
//	len += sprintf(put2ground+len,"%.3f",Heli.vel_command_ned[2]); //ahrs[0].AccB[2] desired_q * R2D
	len += sprintf(put2ground+len,"%.3f",Heli.vel_command_ned[2]); //pidAcc_z->Ki
	
	//-------------------accel---------------------------------     
	/*put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",ahrs[1].pqr[1] * R2D);//Heli.roll_rate_feedforward * R2D Heli.vel_command_ned[0] 
	
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",ahrs[1].pqr[1] * R2D);//ahrs[0].AccB[1]  desired_p * R2D Heli.yaw_rate_feedforward * R2D  Heli.vel_command_ned[1] 
	
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",ahrs[1].pqr[2] * R2D); //ahrs[0].AccB[2] desired_q * R2D 
*/
//-------------------servo_widths  mode--------------------
	put2ground[len++]=',';
	//put2ground[len++]= rc->Key[2] + 48;
	put2ground[len++]= rc->Key[2] + 48;
	
	 
//-------------------manual roll pwm------------------------  
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%d",gps[0].MagRaw[0]);//Heli.manual_servo_width[0] show_temp_rotate
 
//--------------------manual pitch pwm----------------------      
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%d",gps[0].MagRaw[1]);//Heli.manual_servo_width[1] lim_servo_widths_temp[2] 

//-------------------manual throttle pwm--------------------   
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%d",gps[0].MagRaw[2]);//Heli.manual_servo_width[2] lim_servo_widths_temp[3]

//-------------------manual yaw pwm-------------------------    
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%d",gps[0].Err);//Heli.manual_servo_width[3] lim_servo_widths_temp[4] 


//-------------------manual coll pwm------------------------      
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%d",Heli.manual_servo_width[2]);//

//-------------------auto_roll--------------------------------------------------      
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%d",Heli.auto_servo_width[0]);

//-------------------auto_pitch------------------------------      
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%d",Heli.auto_servo_width[1]);

//-------------------auto_coll--------------------------------     
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%d",Heli.auto_servo_width[5]);//servo_widths_throttle 

//-------------------auto_yaw--------------------------------           
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%d",Heli.auto_servo_width[3]);

//-------------------latitude-------------------------------
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.7f",gps[0].lat);//P_EKF[0]*R2D output_data_new.Pos[0]*R2D

//-------------------longitude------------------------------   
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.7f",gps[0].lng);//P_EKF[1]*R2D output_data_new.Pos[1]*R2D

//-------------------quality--------------------------------- GPS工作模式   
	put2ground[len++]=',';
	put2ground[len++]= gps[0].GPSUpdate+48;
	 
//-------------------num_sats--------------------------------     
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%d",gps[0].star);

//------------------------altitude---------------------------      
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.4f",ms5611[0].AltFil); //gps[0].P_EKF[2] bmp_height  

//------------------------current_ned----------------------  //当前位置
	
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.7f",gps[0].NED[0]);//gps[0].NED[0] Heli.roll_rate_feedback
	
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.7f",gps[0].NED[1]); //gps[0].NED[1] Heli.pitch_rate_feedback

	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.5f",gps[0].NED[2]); //gps[0].NED[2]
//------------------------ned_error------------------------------ //偏离位置
	for( i =0 ; i!=3; ++i )
	{
		put2ground[len++]=',';
		len += sprintf(put2ground+len,"%.2f",Heli.pos_error_ned[i]); //Heli.pos_error_ned[i]
	}
//	put2ground[len++]=',';
//	len += sprintf(put2ground+len,"%.5f",pidVelX->Kp); //gps[0].NED[2]
//	
//	put2ground[len++]=',';
//	len += sprintf(put2ground+len,"%.5f",gps[0].NED[2]); //gps[0].NED[2]
//	
//	put2ground[len++]=',';
//	len += sprintf(put2ground+len,"%.5f",gps[0].NED[2]); //gps[0].NED[2]

	//------------------------vel_ned-----------------------------
	for( i =0 ; i!=3; ++i )
	{
		put2ground[len++]=',';
		len += sprintf(put2ground+len,"%.2f",output_data_new.Vel[i]); //相对速度 output_data_new.Vel[i]
	}
	//------------------------course /track-------------------------  
		//put2ground[len++]=',';
		//len += sprintf(put2ground+len,"%.1f",gps_vtg_course);//
	
//------------------------top_speed[0]------------------------------------ 
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.2f",trajectory.top_speed_body[0]);//水平总速度
	//---------------hover_xyz[3]-------------------------    
	for( i =0 ; i!=3; ++i )
	{
		put2ground[len++]=',';
		len += sprintf(put2ground+len,"%.2f",trajectory.hover_ned[i]);//trajectory.hover_xyz 基准悬停点
	}
	//-------------------- current_target[3]------------------------
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",trajectory.cur_target_pos_ned[0]); //目标位置 Heli.roll_rate_command trajectory.cur_target_pos_ned[0] 
	
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",trajectory.cur_target_pos_ned[1]); 
	
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.3f",trajectory.cur_target_pos_ned[2]);
	//-------------heli_step--------------//
	put2ground[len++]=',';
	put2ground[len++]=trajectory.heli_step + 48; //飞行阶段

	//--------------------receive_index-----------------//2    
	put2ground[len++]=',';
	put2ground[len++]=receive_index+48;
	if(receive_index==true)
		receive_index=false;

//---------------pcm_update_flag------------------------//3  
	put2ground[len++]=',';
	put2ground[len++]= pcm_xz.PCM_Safe + 48;

	//---------compass_update_flag--------//4
	put2ground[len++]=',';
	put2ground[len++]=gps[0].MagUpdate + 48;
	//compass_update_flag = false;
 
	//---------imu_update_flag----------//5
	put2ground[len++]=',';
	put2ground[len++]=adis[0].Update+48;   
	//imu_update_flag = 0;
 
	//---------gps_update_flag----------//6
	put2ground[len++]=',';
	put2ground[len++]=gps[0].Update+48;           //gps_update_flag;
	
//---------------round_count---------//7
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%d",3);  //round_count 
	
//-----------how_long---------------//8 
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.2f",trajectory.how_long_body[0]); //trajectory.how_long[0] how_long[0]  改的

	//------------------------hover_time--------------------------------//9 
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.2f",trajectory.hover_time); //trajectory.hover_time hover_time 

//-----------top_speed[0]---------------// 
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.2f",trajectory.top_speed_body[0]);
	
//---------------------com_velocity[0]----------------------------//10
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.2f",trajectory.cur_target_vel_body[0]);//com_position[0]  trajectory.cur_target_vel_body[0]

	//-----------com_position[0]----------//11
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%.2f",3.0f); //com_velocity[0]   
	
	put2ground[len++]=',';
	put2ground[len++]=1+48;//gps
	//bmp_update_flag = 0;
	//abnormal_limit_flag = 0;
		
	put2ground[len++]=',';
	len += sprintf(put2ground+len,"%d",len);

	put2ground[len++]='\r';
	put2ground[len++]='\n';
	to_ground_transmit_flag=false;
  
	UART2_Transmit_DMA((u8*)put2ground,len);
}

void Tran_Init_Para(sTRAN *ele,sTRAN_CFG *elecfg)
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
	
	ele->f.send_check = false;
	ele->f.send_motopwm = false;
	ele->f.send_offset = false;
	ele->f.send_pid1 = false;
	ele->f.send_pid2 = false;
	ele->f.send_pid3 = false;
	ele->f.send_pid4 = false;
	ele->f.send_pid5 = false;
	ele->f.send_pid6 = false;
	ele->f.send_rcdata = false;
	ele->f.send_offset = false;
	ele->f.send_motopwm = false;
	ele->f.send_power = false;
	ele->f.send_check = false;
	ele->f.send_gps = false;
	ele->f.send_senser = false;
	ele->f.send_senser2 = false;
	ele->f.send_user = false;
	
	ele->TxDat[0]=0xAA;
	ele->TxDat[1]=0xAA;
}

bool Tran_Init(sTRAN *ele)
{
	__HAL_DMA_DISABLE(ele->huart->hdmarx);
	ele->huart->hdmarx->Instance->PAR = (u32)&ele->huart->Instance->RDR;
	ele->huart->hdmarx->Instance->NDTR = TRAN_RX_LEN;
	ele->huart->hdmarx->Instance->M0AR = (u32)ele->RxRawDat;
	DMA_CLEAR_FLAG_ALL(ele->huart->hdmarx);
	__HAL_DMA_ENABLE(ele->huart->hdmarx);
	SET_BIT(ele->huart->Instance->CR3, USART_CR3_DMAR);
	
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->huart->hdmatx->Instance->PAR = (u32)&ele->huart->Instance->TDR;
	ele->huart->hdmatx->Instance->NDTR = TRAN_TX_LEN;
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

//void Tran_Loop_1ms(sTRAN *ele)
//{
//	static u8 cnt = 0;
//	static u8 trans_cnt 	= 20;
//	cnt++;
//	if((cnt % trans_cnt) == 0)
//		to_ground_transmit_flag = true;
//	if(cnt>250)
//		cnt = 1;
//}

bool Tran_Calc(sTRAN *ele)
{
	Tim_Calc(&ele->Tim);   //计时
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
		deal_groundtoarm_date();
		ele->Update = true;
		
	}while(0);
	ele->LockRx = HAL_UNLOCKED;
	
	Tim_Calc(&ele->Tim);   //计时
	ele->Time = ele->Tim.OUT;
	return ele->Update;
}

/***************END OF FILE************/
