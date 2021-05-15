#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "userlib.h"
#include "mpu6050.h"
#include "adis16365.h"
#include "gps.h"
#include "ms5611.h"
#include "ahrs.h"
#include "command.h"
#include "motor.h"
#include "rc.h"
#include "exit.h"
#include "transfer.h"
#include "bell.h"
#include "sdcard.h"
#include "gps_ins_EKF.h"
typedef struct
{
	sIMU_CFG imucfg[IMU_NUM];
	sADIS_CFG adiscfg[ADIS_NUM];
	sGPS_CFG gpscfg[GPS_NUM];
	sMS5611_CFG ms5611cfg[MS5611_NUM];
	sAHRS_CFG ahrscfg[AHRS_NUM];
	sMOT_CFG motcfg[MOT_NUM];
	sCMD_CFG cmdcfg[CMD_NUM];
	sRC_CFG rccfg[RC_NUM];
	sEXIT_CFG exitcfg[EXIT_NUM];
	sTRAN_CFG trancfg[TRAN_NUM];
	sBELL_CFG bellcfg[BELL_NUM];
	sPID_CFG pidcfg[PID_NUM];
	sSD_CFG sdcfg[SD_NUM];
}sPARA;

extern sPARA para[1];

void Para_Init(sPARA *para);

#endif

