#include "target.h"
Target_Command target_command;
Target_Queue   target_queue;

void Target_Command_Reset(void)
{
  for(int i =0 ; i!= MAX_COMMAND_SIZE; ++i)
  {
     for(int j =0 ; j!=3 ; ++j)
       target_command.target[i][j] = 0.0;
     target_command.target[i][3] = -1;
  }
  target_command.count = 0 ;
}

void Target_Queue_Reset(void)
{
  target_queue.index = 0;  
  for(int i =0 ; i!=MAX_QUEUE_SIZE; ++i)
  {  
    for(int j =0 ; j!=3 ; ++j)
        target_queue.queue[i][j] =0;   
    target_queue.queue[i][3] = -1 ;
  }
}

bool Target_Queue_Push_Target(void)
{
	bool brake_flag = true;
	for( int i =0 ; target_command.count >0 && target_queue.index != MAX_QUEUE_SIZE; ++i)
	{  
		for(int j=0; j!=4;++j)
			target_queue.queue[target_queue.index][j]=target_command.target[i][j];        
		target_command.count--;   
		target_queue.index ++ ;
		if(-2 == target_queue.queue[i][3])
		{  
			Target_Queue_Reset();
			brake_flag = false;
			break;
		}  
	}
	Target_Command_Reset();   
	return brake_flag; 
}

bool Target_Queue_Pop_Target(float * target_p,float* hover_time)
{
  if(target_queue.index <= 0)
    return false ;  
  for(int i =0 ; i!=3 ; ++i)
    *(target_p+i)=  target_queue.queue[0][i] ;//+ trajectory.hover_xyz[i];
  //trajectory.hover_time =  target_queue.queue[0][3]*30;  
  *hover_time = target_queue.queue[0][3] * 80; //在每次到达目标点后，悬停计时开始，停15 * 80 = 12s
  for(int i =0 ; i !=MAX_QUEUE_SIZE-1 ; ++i )
    for(int j =0 ; j!= 4 ; ++j)
      target_queue.queue[i][j] =  target_queue.queue[i+1][j];
  
  target_queue.queue[MAX_QUEUE_SIZE-1][3] = -1;
  target_queue.index--;
  return true;
} 

bool Target_Queue_Is_Queue_Empty(void){
  return target_queue.index<=0? true:false;
}

/*bool target_monitor(void)
{
   if(target[0][0] >= 500 || target[0][0] <=-500)
     return false ;
   
   if(target[0][1] >= 500 || target[0][1] <=-500)
     return false ;
   
   if(target[0][2] > 100 || target[0][2] <-80)
     return false ;
   
   if(target[0][3] >= 150 || target[0][3] <=0)
     return false ;

   return true;
  
}
*/

