/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "system.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osTimerId myTimer01Handle;
osSemaphoreId myBinarySem01Handle;
osSemaphoreId myBinarySem02Handle;
osSemaphoreId myBinarySem03Handle;
osSemaphoreId myBinarySem04Handle;
osSemaphoreId myBinarySem05Handle;

/* USER CODE BEGIN Variables */
bool SYNC_FLAG = false;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);
void Callback01(void const * argument);

extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  Sys_Init();  
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* definition and creation of myBinarySem02 */
  osSemaphoreDef(myBinarySem02);
  myBinarySem02Handle = osSemaphoreCreate(osSemaphore(myBinarySem02), 1);

  /* definition and creation of myBinarySem03 */
  osSemaphoreDef(myBinarySem03);
  myBinarySem03Handle = osSemaphoreCreate(osSemaphore(myBinarySem03), 1);

  /* definition and creation of myBinarySem04 */
  osSemaphoreDef(myBinarySem04);
  myBinarySem04Handle = osSemaphoreCreate(osSemaphore(myBinarySem04), 1);

  /* definition and creation of myBinarySem05 */
  osSemaphoreDef(myBinarySem05);
  myBinarySem05Handle = osSemaphoreCreate(osSemaphore(myBinarySem05), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(myTimer01Handle, 1);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 512);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityNormal, 0, 512);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, StartTask04, osPriorityLow, 0, 512);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, StartTask05, osPriorityIdle, 0, 512);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN StartDefaultTask */
	SYNC_FLAG = true;
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(myBinarySem01Handle,osWaitForever);
	  Sys_1ms();
  }
  /* USER CODE END StartDefaultTask */
}

/* StartTask02 function */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(myBinarySem02Handle,osWaitForever);
	  Sys_Get();
  }
  /* USER CODE END StartTask02 */
}

/* StartTask03 function */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(myBinarySem03Handle,osWaitForever);
	  Sys_Run();
  }
  /* USER CODE END StartTask03 */
}

/* StartTask04 function */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(myBinarySem04Handle,osWaitForever);
	  Sys_Spr();
		SPEED=rotate_get(mot,exitx);
		Dprintf("%d\r\n",SPEED);
  }
  /* USER CODE END StartTask04 */
}

/* StartTask05 function */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(myBinarySem05Handle,osWaitForever);
	  Sys_Chk();

  }
  /* USER CODE END StartTask05 */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */
  static u32 Timer = 0;
  static u32 TaskTime[5]={1,5,10,20,50};
  Timer++;
  if(Timer%TaskTime[0]==TaskTime[0]-1)osSemaphoreRelease(myBinarySem01Handle);
	if(SYNC_FLAG==false)return;
  if(Timer%TaskTime[1]==TaskTime[1]-1)osSemaphoreRelease(myBinarySem02Handle);
  if(Timer%TaskTime[2]==TaskTime[2]-1)osSemaphoreRelease(myBinarySem03Handle);
  if(Timer%TaskTime[3]==TaskTime[3]-1)osSemaphoreRelease(myBinarySem04Handle);
  if(Timer%TaskTime[4]==TaskTime[4]-1)osSemaphoreRelease(myBinarySem05Handle);
  /* USER CODE END Callback01 */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
