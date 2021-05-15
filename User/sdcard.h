#ifndef __SDCARD_H
#define __SDCARD_H

#include "ff.h"	
#include "userlib.h"
#include "command.h"

#define FILE_BYTE_READ_LEN 50
#define FILE_BYTE_WRITE_LEN 300

typedef struct
{
	u8 FileNum;
	char *name;
	SD_HandleTypeDef *hsd;
}sSD_CFG;

typedef struct
{
	SD_HandleTypeDef *hsd;
	u8    FileNum;
	char  *name;
	eSTA  Sta;       
	eERR  Err;
	sTIM  Tim;
	u32   Time;       //¼ÆÊ±
  u8    ReadFlag;	
	FATFS FatFs;
	FIL   MyFile;
	FRESULT Res;
	u8    Info; 
	UINT  ByteWrite;
	UINT  ByteRead;
	char  TxtRead[FILE_BYTE_READ_LEN];
	char  TxtWrite[FILE_BYTE_WRITE_LEN];
}sSD;

#define SD_NUM 1

extern sSD sd[SD_NUM];

void Sd_Init_Para(sSD *ele,sSD_CFG *elecfg);
bool Sd_Init(sSD *ele);
bool Sd_Calc(sSD *ele);
void Fprintf(const char * _format,...);

#endif
