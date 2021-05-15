#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "userlib.h"
#include "bell.h"
#include "sdcard.h"
#include "parameter.h"
#include "flight_mode.h"
#include "helicopter.h"

void Sys_Init(void);
void Sys_Get(void);
void Sys_Run(void);
void Sys_Spr(void);
void Sys_1ms(void);
void Sys_Chk(void);

#endif
