#ifndef _TARGET_H
#define _TARGET_H
#include "userlib.h"

#define MAX_QUEUE_SIZE 20
#define MAX_COMMAND_SIZE 6

typedef struct{
	float target[MAX_COMMAND_SIZE][4];
	int	   count;
}Target_Command;

typedef struct{	
	float queue[MAX_QUEUE_SIZE][4];
	int  	 index;
}Target_Queue;
 
extern Target_Command target_command;
extern Target_Queue   target_queue;
void Target_Command_Reset(void);
void Target_Queue_Reset(void);
bool Target_Queue_Push_Target(void);
bool Target_Queue_Pop_Target(float * target_p,float* hover_time);
bool Target_Queue_Is_Queue_Empty(void);
#endif
