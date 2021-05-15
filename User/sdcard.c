#include "sdcard.h"

/***************************************************\
功能：
  sd卡 SDIO读取
说明：
  1、在fatfs.c中添加SD_Init();
  2、修改sd_diskio.c，SD_Read和SD_Write加入DMA与uint64
  3、使用f_sync及时刷新数据。
\***************************************************/

sSD sd[SD_NUM];

/******************驱动程序****************/
void SDMMC1_IRQHandler(void)
{
	sSD *ele = sd;
  HAL_SD_IRQHandler(ele->hsd);
}

void DMA2_Stream3_IRQHandler(void)
{
	sSD *ele = sd;
  HAL_DMA_IRQHandler(ele->hsd->hdmarx);
}

void DMA2_Stream6_IRQHandler(void)
{
	sSD *ele = sd;
  HAL_DMA_IRQHandler(ele->hsd->hdmatx);
}

void Fprintf(const char * _format,...)
{
	sSD *ele=sd;
	if(ele->Sta!=STA_RUN)return;
	static u8 count=0;
	u32 len;
	va_list ap;
	va_start(ap,_format);
	vsnprintf(ele->TxtWrite,300,_format,ap);
	len=strlen(ele->TxtWrite);
	ele->Res=f_write(&ele->MyFile, ele->TxtWrite, len, (UINT*)&ele->ByteWrite);
	if(count<100)count++;
	else
	{
		count = 0;
		f_sync(&ele->MyFile);  //强制刷新
	}
}

/******************功能函数****************/
void Sd_Init_Para(sSD *ele,sSD_CFG *elecfg)
{
	ele->hsd = elecfg->hsd;
	ele->FileNum = elecfg->FileNum<99?elecfg->FileNum:99;
	ele->name = elecfg->name;
	ele->Info = 0;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	ele->ReadFlag = 0;
	memset(ele->TxtRead,0,sizeof(ele->TxtRead));
}

bool Sd_Init(sSD *ele)
{
	Dprintf("\r\n%s Init...\r\n",ele->name);
	if(ele->Sta != STA_INI)
	{
		Dprintf("--Par [NO]\r\n");
        return false;
	}
	extern char SD_Path[4];
	if((ele->Res=f_mount(&ele->FatFs, (TCHAR const*)SD_Path, 0)) != FR_OK)
	{
		Dprintf("%s Mount [NO]\r\n",ele->name);
		ele->Err = ERR_HARD;
		return false;
	}
	Dprintf("%s Mount [OK]\r\n",ele->name);
	if((ele->Res=f_open(&ele->MyFile, "info.txt",  FA_OPEN_ALWAYS | FA_READ | FA_WRITE)) != FR_OK)
	{
		f_close(&ele->MyFile);
		if((ele->Res=f_mkfs((TCHAR const*)SD_Path, 0, 0)) != FR_OK)
		{
			Dprintf("%s Exist [NO]\r\n",ele->name);
			ele->Err = ERR_LOST;
			return false;
		}
		if((ele->Res=f_open(&ele->MyFile, "info.txt",  FA_OPEN_ALWAYS | FA_READ | FA_WRITE)) != FR_OK)
		{
			Dprintf("info.txt Open [NO]\r\n");
			ele->Err = ERR_HARD;
			return false;
		}
	}
	Dprintf("info.txt Open [OK]\r\n");
	if((ele->Res=f_read(&ele->MyFile, ele->TxtRead, 2, (UINT*)&ele->ByteRead))!=FR_OK)
	{
		Dprintf("info.txt Read [NO]\r\n");
		ele->Err = ERR_HARD;
		return false;
	}
	if(ele->ByteRead!=0)ele->Info=atoi(ele->TxtRead);  //读不到数据
	ele->Info++;   //当前文件名
	if(ele->Info<=0||ele->Info>ele->FileNum)ele->Info=1;
	Dprintf("info.txt Read [OK]--%s.TXT\r\n",ele->TxtRead);
	sprintf(ele->TxtWrite,"%d",ele->Info);
	f_lseek(&ele->MyFile, 0);    //文件移到头部修改数据
	if((ele->Res=f_write(&ele->MyFile, ele->TxtWrite, 2, (UINT*)&ele->ByteWrite))!=FR_OK)
	{
		Dprintf("info.txt Write [NO]\r\n");
		ele->Err = ERR_HARD;
		return false;
	}
	if(ele->ByteWrite!=0)
		Dprintf("info.txt Write [OK]--%d.TXT\r\n",ele->Info);
	f_close(&ele->MyFile);
	
	char fileName[8];
	sprintf(fileName,"%d.txt",ele->Info);
	if((ele->Res=f_open(&ele->MyFile, (const TCHAR*)fileName,  FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK)
	{
		Dprintf("%s Open [NO]\r\n",fileName);
		ele->Err = ERR_HARD;
		return false;
	}
	Dprintf("%s Open [OK]\r\n",fileName);
	
	char TxtDoc[20];
	sprintf(TxtDoc,"%d.txt--DATA\r\n",ele->Info);
	if((ele->Res=f_write(&ele->MyFile, TxtDoc, strlen(TxtDoc), (UINT*)&ele->ByteWrite))!=FR_OK)
	{
		Dprintf("%s Write [NO]\r\n",fileName);
		ele->Err = ERR_HARD;
		return false;
	}
	if(ele->ByteWrite!=0)
		Dprintf("%s Write [OK]\r\n",fileName);
	f_sync(&ele->MyFile);  //强制刷新
	ele->Sta = STA_RUN;
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

bool Sd_Calc(sSD *ele)
{
	if(ele->Sta!=STA_CAL && ele->Sta!=STA_RUN)return false;
	Tim_Calc(&ele->Tim);   //计时 
	static bool SdIsRead = false;
	if(ele->ReadFlag!=0)
	{
		ele->Sta = STA_CAL;
		if(SdIsRead==false)
		{
			ele->Res=f_close(&ele->MyFile);
			char fileName[8];
			sprintf(fileName,"%d.txt",ele->ReadFlag);
			if((ele->Res=f_open(&ele->MyFile, (const TCHAR*)fileName,  FA_OPEN_ALWAYS | FA_READ | FA_WRITE)) != FR_OK)
			{
				Dprintf("\r\n%s Open Fail!\r\n",fileName);
				ele->Err = ERR_HARD;
				ele->ReadFlag = 0;
			}
			else SdIsRead = true;
		}
		else
		{
			if((ele->Res=f_read(&ele->MyFile, ele->TxtRead, 10, (UINT*)&ele->ByteRead))!=FR_OK)
			{
				Dprintf("\r\n%s Read Fail!\r\n",ele->name);
				ele->ReadFlag = 0;
				ele->Err = ERR_HARD;
				SdIsRead = false;
			}
			if(ele->ByteRead==0)
			{
				ele->ReadFlag = 0;
				Dprintf("\r\n%s Read End!\r\n",ele->name);
				f_close(&ele->MyFile);
				SdIsRead = false;
			}
			else
				UART7_Transmit_DMA((u8*)ele->TxtRead,ele->ByteRead);
		}
	}
	Tim_Calc(&ele->Tim);   //计时 
	ele->Time = ele->Tim.OUT;
	return true;
}

