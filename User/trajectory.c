#include "trajectory.h"
sTrajectory trajectory;

float circle_radius_test = 10.0f;
float brake_distance = 0.0f;
float circle_roll_angle = 0.0f;
int num = 0;
bool Mixture_to_Pointtopoint_flag = false;

//-------------2015/6/3-------------------
unsigned char auto_pilot11 = 0;
//----------------------------------------
void Trajectoyr_Current_Target_Update(sTrajectory *ele,float *next_target_ned)
{
	for(int i=0 ; i!=3 ; ++i)
		ele->cur_target_pos_ned[i] = ele->next_target_ned[i];
}

//----------------------------------------

void Trajectory_Reset(sTrajectory *ele)
{
	ele->trajectory_mod = Point_to_point;//Circle  Point_to_point
	ele->heli_step = 1;
	ele->RTL_Orignal_flag = 0; //RTL 
	ele->RTL_Orignal_update_flag = 1;//RTL 
	ele->RTL_First_switch = 1;//RTL 
	ele->end_heading = ahrs[1].Ang[2];
	ele->hover_time = FLY_TIME;
	ele->Isturn = 0;
	ele->first_receive_target = 0;
	ele->turn_accel = 0;
	ele->turn_accel_flag = false;
	ele->first_entry_brake = 1;
	ele->first_entry_circle = 1;
	ele->circle_velocity = 4.0f;//Ô²»¡¹ìÏßËÙ¶È
	//ele->num = 1;
	for(int i =0 ; i!= 3 ;  ++i)
	{
		ele->hover_ned[i] = gps[0].NED[i];
		ele->next_target_ned[i] = ele->hover_ned[i]; 
		ele->cur_target_pos_ned[i] = ele->hover_ned[i];
		ele->cur_target_vel_ned[i] = 0.0f;
		ele->com_pqr[i]      = 0.0f;
		ele->com_theta[i]    = 0.0f;
		ele->top_speed_body[i] = 0.0f;
		ele->how_long_body[i] = 0.0f;
	}

}

//void ahrs_adjust(float *theta,float *angle_offset)
//{
//	for (int i = 0;i < 3;i++)
//	{	
//		angle_offset[i] = angle_offset[i]*C_DEG2RAD;
//	}
//	for (int j = 0;j < 3;j++)
//	{
//		theta[j] -= angle_offset[j];
//	}
//}

void Target_Plus_Hover_Point(sTrajectory *ele)
{
	for(int i=0;i<3;i++)
	{
		ele->next_target_ned[i] = ele->next_target_ned[i] + ele->hover_ned[i];
	}
}

bool Trajectory_Target_Update(sTrajectory *ele,int trajectory_mod)
{  
	if (ele->trajectory_mod == Point_to_point)
	{
		if ((ele->hover_time <=-1 && ele->heli_step == 1 && (!Target_Queue_Is_Queue_Empty())) || ele->first_receive_target == 1)
		{
			if(!Target_Queue_Pop_Target(ele->next_target_ned,&ele->hover_time))
				return false;
			Target_Plus_Hover_Point(ele);
			ele->first_receive_target++;
			return true;
		}
		if (Target_Queue_Is_Queue_Empty() && ele->hover_time <= -2000)
		{ 
			ele->next_target_ned[0] = ele->hover_ned[0] ;
			ele->next_target_ned[1] = ele->hover_ned[1] ;
			ele->next_target_ned[2] = ele->hover_ned[2] ;
			ele->hover_time = DEFAULT_HOVER_TIEM ;
			return true;
		}
	}

	if (ele->trajectory_mod == Circle)
	{
		if ((!Target_Queue_Is_Queue_Empty()) && ele->first_receive_target == 1)
		{
			if(!Target_Queue_Pop_Target(ele->next_target_ned,&ele->hover_time))
			return false;
			Target_Plus_Hover_Point(ele);
			ele->first_receive_target++;
			return true;
		}
		if ((!Target_Queue_Is_Queue_Empty()) && ele->circle_finish_flag == true && ele->hover_time <=-1 )
		{	
			if(!Target_Queue_Pop_Target(ele->next_target_ned,&ele->hover_time))
			return false;
			Target_Plus_Hover_Point(ele);
			ele->circle_finish_flag = false;
			return true;
		}		
/*		
if (target_queue_t->is_queue_empty() && ele->hover_time <= -2000)
//,1 
{ 
	ele->next_target_ned[0] = ele->hover_ned[0] ;
	ele->next_target_ned[1] = ele->hover_ned[1] ;
	ele->next_target_ned[2] = ele->hover_ned[2] ;
	ele->hover_time = DEFAULT_HOVER_TIEM ;
	return true;
}
*/
	}

	if (ele->trajectory_mod == Mixture)
	{
		if (ele->circle_finish_flag == true)
		{	
			for (int i=0; i!=5; i++)
			{
				ele->next_target_ned[i] = ele->next_next_target[i];
			}
			if(!Target_Queue_Pop_Target(ele->next_next_target,&ele->hover_time))
			{
				ele->heli_step = 8;
				return false;
			}
			Target_Plus_Hover_Point(ele);
			return true;
		}
		if ((!Target_Queue_Is_Queue_Empty()) && ele->first_receive_target == 1)
		{
			if(!Target_Queue_Pop_Target(ele->next_target_ned,&ele->hover_time))
				return false;
			Target_Plus_Hover_Point(ele);
			if(!Target_Queue_Pop_Target(ele->next_next_target,&ele->hover_time))
				return false;
			Target_Plus_Hover_Point(ele);
			ele->first_receive_target++;
			return true;
		}
	}
	return false;
}

//  ,--ele->end_heading--ele->Isturn 
// end_heading_point ;
//n_target ;current_heading ;current_xyz 
void Trajectory_Turn_Heading(sTrajectory *ele,float * end_heading_point)
{
	float temp = atan2f(ele->next_target_ned[0]-ele->cur_target_pos_ned[0], 
								 ele->next_target_ned[1]-ele->cur_target_pos_ned[1]);
	// atan2X,end_headingNED
	if ( temp > -PI /2.0f )
		*end_heading_point = PI /2.0f - temp ;
	else 
		*end_heading_point = - 3.0f * PI /2.0f - temp ;

	if ( *end_heading_point - Heli.yaw_command > PI )
	{   
		ele->Isturn = 1;     //
		return;
	}
	if ( *end_heading_point - Heli.yaw_command < -PI )
	{
		ele->Isturn = 2;     //
		return;
	}
	if (*end_heading_point - Heli.yaw_command > 0.00f && *end_heading_point - Heli.yaw_command < PI)
	{
		ele->Isturn = 3;     //
		return;
	}
	if (*end_heading_point - Heli.yaw_command < 0.00f && *end_heading_point - Heli.yaw_command > -PI)
	{
	ele->Isturn = 4;    //
	return;
	}
}


void Trajectory_Distance_Calculate(sTrajectory *ele,int trajectory_mod)
{
	if (ele->trajectory_mod == Point_to_point || ele->trajectory_mod == Circle || ele->trajectory_mod == RTL)
	{
		ele->how_long_body[0] =sqrt((ele->cur_target_pos_ned[0]-ele->next_target_ned[0])*(ele->cur_target_pos_ned[0]-ele->next_target_ned[0]) 
					+ (ele->cur_target_pos_ned[1]-ele->next_target_ned[1])*(ele->cur_target_pos_ned[1]-ele->next_target_ned[1]));      
		ele->how_long_body[2] = ele->cur_target_pos_ned[2]-ele->next_target_ned[2];
		
		ele->vertical_speed_direction = ele->how_long_body[2] >= 0?-1:1;//-1  1 
		ele->distance_line = ele->how_long_body[0];
		ele->distance_vertical = fabs(ele->how_long_body[2]);
		return;
	}
	if (ele->trajectory_mod == Mixture)
	{
	/*v1(x1, y1)v2(x2, y2)v1×v2=x1y2-y1x2
	v1v2,,
	0()*/
		float cross_product = (ele->next_target_ned[0]-ele->cur_target_pos_ned[0])*(ele->next_next_target[1]-ele->cur_target_pos_ned[1])
			- (ele->next_target_ned[1]-ele->cur_target_pos_ned[1])*(ele->next_next_target[0]-ele->cur_target_pos_ned[0]);
		if (cross_product == 0)
		{
			for (int i=0; i!=5; i++)
			{
				ele->next_target_ned[i] = ele->next_next_target[i];
			}		
			if(!Target_Queue_Pop_Target(ele->next_next_target,&ele->hover_time))
			{
				ele->heli_step = 8;
				return;
			}
			Target_Plus_Hover_Point(ele);
		}
		else 
		{
			if (cross_product < 0)
			{
				ele->circle_turn = 2;	//,
			}
			else
			{
				ele->circle_turn = 1;	//,
			}
		}

		ele->how_long_body[0] = sqrt((ele->cur_target_pos_ned[0]-ele->next_target_ned[0])*(ele->cur_target_pos_ned[0]-ele->next_target_ned[0]) 
					+ (ele->cur_target_pos_ned[1]-ele->next_target_ned[1])*(ele->cur_target_pos_ned[1]-ele->next_target_ned[1]));
		ele->how_long_body[2] = ele->hover_ned[2]-ele->next_target_ned[2];

		float b = sqrt((ele->next_target_ned[0]-ele->next_next_target[0])*(ele->next_target_ned[0]-ele->next_next_target[0]) 
						+ (ele->next_target_ned[1]-ele->next_next_target[1])*(ele->next_target_ned[1]-ele->next_next_target[1]));
		float c = sqrt((ele->cur_target_pos_ned[0]-ele->next_next_target[0])*(ele->cur_target_pos_ned[0]-ele->next_next_target[0]) 
						+ (ele->cur_target_pos_ned[1]-ele->next_next_target[1])*(ele->cur_target_pos_ned[1]-ele->next_next_target[1]));

		ele->sector_angle = acos((pow(ele->how_long_body[0],2)+pow(b,2)-pow(c,2)) / 2*fabs(ele->how_long_body[0]*b));
		ele->circle_radius = 2*ele->circle_velocity; 		 
		if (ele->sector_angle >= 0.785f && ele->sector_angle <= 2.356f)//60-120
		{
			float r;
			ele->sector_angle = PI - ele->sector_angle;
			r = ele->circle_radius*tan(ele->sector_angle/2);
			ele->how_long_body[0] = ele->how_long_body[0] - r;
			ele->next_target_ned[0] -= r*cos(ahrs[1].Ang[2]);
			ele->next_target_ned[1] -= r*sin(ahrs[1].Ang[2]);
			return;
		}
		else
		{
			ele->trajectory_mod = Point_to_point;
			Mixture_to_Pointtopoint_flag = true;
			return;
		}	 
	}
}

void Trajectory_Speed_Calculate(sTrajectory *ele,int max_com_speed, int trajectory_mod)
{   
	if (ele->trajectory_mod == Point_to_point || ele->trajectory_mod == RTL )
	{
		if (2.0f * (max_com_speed * max_com_speed)/ACCEL > ele->distance_line) //1
		{	
			ele->top_speed_body[0] = sqrt(ACCEL * ele->distance_line/2.0f );// 1/4
		}	
		else
		{     
			ele->top_speed_body[0] = max_com_speed;
		}

		if (2.0f * (MAX_COM_SPEED_VERTICAL * MAX_COM_SPEED_VERTICAL)/ACCEL_VERTICAL > ele->distance_vertical) //1
		{	
			ele->top_speed_body[2] = sqrt(ACCEL_VERTICAL * ele->distance_vertical/2.0f);// 1/4
		}	
		else
		{     
			ele->top_speed_body[2] = MAX_COM_SPEED_VERTICAL;
		}

		return;
	}
	if (ele->trajectory_mod == Circle)
	{
		if (0.5f*(max_com_speed*max_com_speed)/ACCEL > ele->how_long_body[0])
		{
			ele->top_speed_body[0] = 0;
		}
		else
		{
			ele->top_speed_body[0] = max_com_speed;
		}
		return;
	}	 
//	if (ele->trajectory_mod == Mixture)
//	{
//		if (ele->circle_finish_flag == false)
//		{	 
//			if (0.5f*(max_com_speed*max_com_speed)/ACCEL > ele->how_long_body[0])
//			{
//				ele->top_speed_body[0] = 0;
//			}
//			else
//			{
//				ele->top_speed_body[0] = max_com_speed;
//			}
//			return;
//		}
//		else
//		{
//			ele->top_speed_body[0] = max_com_speed;
//		}

//	}	 
}

void Trajectory_Schedule(sTrajectory *ele,int max_speed, int mode)  //next_targe
{
	Trajectory_Distance_Calculate(ele,mode);              //ele->how_long_body
	Trajectory_Speed_Calculate(ele,max_speed, mode);                //ele->top_speed_body 
}

void Trajectory_Hover_Step(sTrajectory *ele)
{
	for(int i =0 ; i!= 3 ;  ++i)
	{ 
		ele->com_pqr[i] = 0.0f;
		ele->com_theta[i] = 0.0f;   //!
	}
	ele->cur_target_vel_ned[0] = 0.0f;
	ele->cur_target_vel_ned[1] = 0.0f;
	ele->cur_target_vel_ned[2] = 0.0f;
	ele->cur_target_vel_body[0] = 0.0f;
	ele->top_speed_body[0] = 0.0f;
	ele->how_long_body[0] = 0.0f;
	ele->top_speed_body[1] = 0.0f;
	ele->how_long_body[1] = 0.0f;
	ele->top_speed_body[2] = 0.0f;
	ele->how_long_body[2] = 0.0f; 

	ele->hover_time--;
	if(ele->hover_time < -500)
		ele->hover_time = -200;
}

//-------------------------------syc--2012.4.13--------------------//
void Trajectory_Turning_Head_Step(sTrajectory *ele)
{  	 
	if ((fabs(ele->end_heading - Heli.yaw_command)) < 0.04f )  //0.04rad 2.3deg 
	{
		ele->Isturn = 0 ;
		Heli.yaw_command = ele->end_heading;
		ele->turn_accel_flag=true;
		return;
	}

	if ( ele->Isturn == 1 || ele->Isturn ==4 )
	{ //
		Heli.yaw_command -= 0.005f; //by syc
		if (Heli.yaw_command< - PI )
			Heli.yaw_command +=2* PI;
		return;
	}
	if ( ele->Isturn == 2 || ele->Isturn ==3 )
	{ //
		Heli.yaw_command += 0.005f;
		if (Heli.yaw_command > PI )
			Heli.yaw_command -=2* PI;     	
		//position_t->heading += 0.025 ;
		return;
	}
}

void Trajectory_Speed_Up_Step(sTrajectory *ele)
{
	if (ele->cur_target_vel_body[0] < ele->top_speed_body[0])
	{
		ele->cur_target_vel_body[0] +=  CONTROL_DT * ACCEL ;
	}
	else
	{
		ele->cur_target_vel_body[0] =  ele->top_speed_body[0] ;
	}

	ele->how_long_body[0] -= ele->cur_target_vel_body[0] * CONTROL_DT;
	ele->cur_target_vel_ned[0] = ele->cur_target_vel_body[0] * cos(Heli.yaw_command);
	ele->cur_target_vel_ned[1] = ele->cur_target_vel_body[0] * sin(Heli.yaw_command);
	
	ele->cur_target_pos_ned[0]+= ele->cur_target_vel_ned[0] * CONTROL_DT;
	ele->cur_target_pos_ned[1]+= ele->cur_target_vel_ned[1] * CONTROL_DT;

//	#ifdef circle_test
//	if(ele->cur_target_vel_body[0] >= 0.5)
//	{   
//		ele->heli_step = 9;
//	  return ;
//	}
//	#endif
}

void Trajectory_Speed_Down_Step(sTrajectory *ele)
{
	if ( ele->cur_target_vel_body[0] > 0.0f) 
	{
		ele->cur_target_vel_body[0] -=  CONTROL_DT *ACCEL ;
	}
	else
	{
		ele->cur_target_vel_body[0] = 0.0f;  
	}
	ele->how_long_body[0] -= ele->cur_target_vel_body[0] * CONTROL_DT;
	ele->cur_target_vel_ned[0] = ele->cur_target_vel_body[0] * cos(Heli.yaw_command);
	ele->cur_target_vel_ned[1] = ele->cur_target_vel_body[0] * sin(Heli.yaw_command);
	
	ele->cur_target_pos_ned[0]+= ele->cur_target_vel_ned[0] * CONTROL_DT;
	ele->cur_target_pos_ned[1]+= ele->cur_target_vel_ned[1] * CONTROL_DT;

}

void Trajectory_Speed_Up_Step_Vertical(sTrajectory *ele)
{
	if(ele->vertical_speed_direction == 1)//-1  1 
	{
		if (ele->cur_target_vel_body[2] < ele->top_speed_body[2])
		{
			ele->cur_target_vel_body[2] +=  CONTROL_DT * ACCEL_VERTICAL ;
		}
		else
		{
			ele->cur_target_vel_body[2] =  ele->top_speed_body[2] ;
		}

		ele->how_long_body[2] += ele->cur_target_vel_body[2]*CONTROL_DT;
		ele->cur_target_vel_ned[2] = ele->cur_target_vel_body[2];
		ele->cur_target_pos_ned[2] -= ele->cur_target_vel_ned[2];
	}
	else if(ele->vertical_speed_direction == -1)//-1  1 
	{
		if (ele->cur_target_vel_body[2] > -1.0f * ele->top_speed_body[2])
		{
			ele->cur_target_vel_body[2] -=  CONTROL_DT * ACCEL_VERTICAL ;
		}
		else
		{
			ele->cur_target_vel_body[2] =  -1.0f * ele->top_speed_body[2] ;
		}

		ele->how_long_body[2] += ele->cur_target_vel_body[2]*CONTROL_DT;
		ele->cur_target_vel_ned[2] = ele->cur_target_vel_body[2];
		ele->cur_target_pos_ned[2] -= ele->cur_target_vel_ned[2];
	}
}

void Trajectory_Speed_Down_Step_Vertical(sTrajectory *ele)
{
	if(ele->vertical_speed_direction == 1)//-1  1 
	{   
		if ( ele->cur_target_vel_body[2] > 0.0f) 
		{
			ele->cur_target_vel_body[2] -=  CONTROL_DT *ACCEL_VERTICAL ;
		}
		else
		{
			ele->cur_target_vel_body[2] = 0.0f;  
		}

		ele->how_long_body[2] += ele->cur_target_vel_body[2]*CONTROL_DT;
		ele->cur_target_vel_ned[2] = ele->cur_target_vel_body[2];
		ele->cur_target_pos_ned[2] -= ele->cur_target_vel_ned[2];
		}

	else if(ele->vertical_speed_direction == -1)//-1  1 
	{   
		if (ele->cur_target_vel_body[2] < 0.0f) 
		{
			ele->cur_target_vel_body[2] +=  CONTROL_DT *ACCEL_VERTICAL ;
		}
		else
		{
			ele->cur_target_vel_body[2] = 0.0f;  
		}

		ele->how_long_body[2] += ele->cur_target_vel_body[2]*CONTROL_DT;
		ele->cur_target_vel_ned[2] = ele->cur_target_vel_body[2];
		ele->cur_target_pos_ned[2] -= ele->cur_target_vel_ned[2];
	}
}


void Trajectory_Land_Step(sTrajectory *ele)
{
	ele->cur_target_vel_body[2] = 0.1f;
	ele->how_long_body[2] += ele->cur_target_vel_body[2]*CONTROL_DT;
	ele->cur_target_vel_ned[2] = ele->cur_target_vel_body[2];
	ele->cur_target_pos_ned[2] -= ele->cur_target_vel_ned[2];   
}


void Trajectory_Brake_Down_Step(sTrajectory *ele)
{
	ele->hover_time = DEFAULT_HOVER_TIEM;

	if((ele->first_entry_brake))
	{
		brake_distance = 0.5f * ele->cur_target_vel_body[0] * ele->cur_target_vel_body[0] / ACCEL;
		ele->first_entry_brake = 0;       
		for(int i =0 ; i!= 3 ;  ++i)
		{  
			ele->top_speed_body[i] 	= 0.0f;
			ele->how_long_body[i] 	= 0.0f;
		}
	}
	else
	{
		if (ele->cur_target_vel_body[0] > 0)
		{ 
			ele->cur_target_vel_body[0] -=  CONTROL_DT *ACCEL ;
			brake_distance = brake_distance - ele->cur_target_vel_body[0]*CONTROL_DT;
			ele->cur_target_vel_ned[0] = ele->cur_target_vel_body[0] * cos(Heli.yaw_command);
			ele->cur_target_vel_ned[1] = ele->cur_target_vel_body[0] * sin(Heli.yaw_command);
			ele->cur_target_pos_ned[0]+= ele->cur_target_vel_ned[0] * CONTROL_DT;
			ele->cur_target_pos_ned[1]+= ele->cur_target_vel_ned[1] * CONTROL_DT;
		}
		else
		{
			ele->heli_step = 1;
			ele->first_entry_brake = 1;   
		}
	}
}

bool Trajectory_Climbing_Step(sTrajectory *ele)
{
//	com_position[2] = ele->how_long_body[2] ; 

//	if(abs(ele->how_long_body[2]) > 3)// 2015-12-14 byliu
//	{                    
//		 ele->cur_target_vel_body[2] = ele->top_speed_body[2] ; 
//		 ele->how_long_body[2] += ele->cur_target_vel_body[2]*CONTROL_DT;//
//	return false;
//	}

//	if(abs(ele->how_long_body[2]) <= 3)
//	{   
//		 if( ele->how_long_body[2] <= -1 && ele->top_speed_body[2]>0.01 )// 2015-12-14 byliu
//				{
//					 ele->cur_target_vel_body[2] = 0.2;
//					 ele->how_long_body[2] +=ele->cur_target_vel_body[2]*CONTROL_DT;//
//					 return false;
//					
//				}
//		 else if(ele->how_long_body[2]>= 1 && ele->top_speed_body[2]<-0.01)// 2015-12-14 byliu
//				{   
//					 ele->cur_target_vel_body[2] = -0.2;
//					 ele->how_long_body[2] +=ele->cur_target_vel_body[2]*CONTROL_DT;//
//					 return false;               
//				} 
//		 else
//				 {
//							ele->how_long_body[2] = 0.0 ;
//							ele->top_speed_body[2] = 0.0 ;
//							ele->cur_target_vel_body[2] = 0.0;
//							com_position[2] = 0.0;
//							ele->hover_ned[2] = ele->next_target_ned[2];
//							return true;               
//				 }

//	}
	return false;
}

void Trajectory_Circle_Step(sTrajectory *ele,float _circle_velocity, float _radius,float _helix_velocity, float _angle)
{
	static float init_theta,init_position[3];

	if (ele->first_entry_circle == 1)
	{
		ele->cur_target_vel_body[2] = -_helix_velocity;
		ele->cushion_flag = 1;

		ele->cur_target_vel_body[0] = _circle_velocity;//limit(ele->cur_target_vel_body[0],-init_velocity_on_circle,init_velocity_on_circle ); //
		ele->com_pqr_ned[2] = ele->cur_target_vel_body[0]/_radius;       // 

		ele->circle_angle = 0.0;                    //,
		init_theta = Heli.yaw_command;           //     
		init_position[0] = ele->cur_target_pos_ned[0]; // current_xyz
		init_position[1] = ele->cur_target_pos_ned[1];
		init_position[2] = ele->cur_target_pos_ned[2];
	}
	// com_theta[0] = atan2((ele->cur_target_vel_body[0]*com_pqr[2]),G_0); //roll (trim_roll_angle * C_DEG2RAD) 
	//
	ele->circle_angle += ele->com_pqr_ned[2]*CONTROL_DT;	
	if (ele->circle_angle >= _angle)
	{
		for(int i =0 ; i!= 3 ;  ++i)
		{
			ele->com_pqr[i]      = 0.0; 
			ele->com_theta[i]    = 0.0;
			ele->top_speed_body[i] = 0.0;
			ele->how_long_body[i] = 0.0;
		}

		ele->cur_target_vel_body[2] = 0.0;
		ele->circle_finish_flag = true;
		ele->first_entry_circle = 1;
		ele->first_entry_brake = 1;
		ele->heli_step = 8;//,,
		return;
	}
	else
	{
		ele->com_pqr[2] = ele->com_pqr_ned[2] * cos(ahrs[1].Ang[0]) * cos(ahrs[1].Ang[1]);
	}

	if (ele->circle_turn == 1)//
	{
		Heli.yaw_command += ele->com_pqr[2]*CONTROL_DT;
		if (Heli.yaw_command > PI)
		{
		  Heli.yaw_command -= 2*PI;
		}
		ele->cur_target_pos_ned[0] = init_position[0]+_radius*(-sin(init_theta)+sin(Heli.yaw_command));
		ele->cur_target_pos_ned[1] = init_position[1]+_radius*(cos(init_theta)-cos(Heli.yaw_command));		
		ele->cur_target_vel_ned[0] = ele->cur_target_vel_body[0] * cos(Heli.yaw_command);
		ele->cur_target_vel_ned[1] = ele->cur_target_vel_body[0] * sin(Heli.yaw_command);
		
//		circle_roll_angle = atan2((ele->cur_target_vel_body[0]*ele->com_pqr[2]),G_zinit);//roll;
//		if((fabs(ele->com_theta[0]) < fabs(circle_roll_angle)) & (ele->cushion_flag ==1))
//		{
//			ele->com_theta[0] = ele->com_theta[0] + circle_roll_angle *0.02f;
//		}
//		else
//		{
//			ele->com_theta[0] = circle_roll_angle;
//			ele->cushion_flag = 0;
//		}
	}
	if (ele->circle_turn == 2)//
	{
		Heli.yaw_command -= ele->com_pqr[2]*CONTROL_DT;
		if (Heli.yaw_command < -PI)
		{
			Heli.yaw_command += 2*PI;
		}
		ele->cur_target_pos_ned[0] = init_position[0]+_radius*(sin(init_theta)-sin(Heli.yaw_command));
		ele->cur_target_pos_ned[1] = init_position[1]+_radius*(-cos(init_theta)+cos(Heli.yaw_command));		
		ele->cur_target_vel_ned[0] = ele->cur_target_vel_body[0] * cos(Heli.yaw_command);
		ele->cur_target_vel_ned[1] = ele->cur_target_vel_body[0] * sin(Heli.yaw_command);
		
//		circle_roll_angle = -atan2((ele->cur_target_vel_body[0]*ele->com_pqr[2]),G_zinit);//roll
//		if((fabs(ele->com_theta[0]) < fabs(circle_roll_angle)) & (ele->cushion_flag ==1))
//		{
//			ele->com_theta[0] = ele->com_theta[0] + circle_roll_angle *0.04;
//		}
//		else
//		{
//			ele->com_theta[0] = circle_roll_angle;
//			ele->cushion_flag = 0;
//		}
	}		

	ele->cur_target_pos_ned[2] += ele->cur_target_vel_body[2]*CONTROL_DT ;
	ele->com_pqr[1] = ele->com_pqr[2]*cos(ele->com_theta[0]);//pitch
	ele->first_entry_circle ++;
	if(ele->first_entry_circle >= 10000)
		ele->first_entry_circle = 200;
}

void Trajectory_Step(sTrajectory *ele,int trajectory_mode)
{
	switch (trajectory_mode)
	{
		case Hover : Trajectory_Hover_Mode_Step(ele);//
		break;
		 
		case Point_to_point : Trajectory_Point_to_point_Mode_Step(ele);//
		break;
		 
		case Circle : Trajectory_Circle_Mode_Step(ele);//
		break;
		 
		case Mixture : Trajectory_Mixture_Mode_Step(ele);//
		break; 
		 
		case Test : Trajectory_Test_Mode_Step(ele);//
		break;
																		
		case RTL : Trajectory_RTL_Mode_Step(ele); //
		break;
		 
		default : Trajectory_Brake_Down_Step(ele);	//
		break;
	}
}

void Trajectory_Hover_Mode_Step(sTrajectory *ele)
{
	if (ele->heli_step == 1) 
		Trajectory_Hover_Step(ele);
	if (ele->hover_time <= 0)
		ele->hover_time = FLY_TIME;
}

void Trajectory_Point_to_point_Mode_Step(sTrajectory *ele)
{
	if(3 == auto_pilot11)
	{
		ele->next_target_ned[0] = ele->hover_ned[0];
		ele->next_target_ned[1] = ele->hover_ned[1];
		ele->next_target_ned[2] = TAKE_OFF_HEIGHT;   //,TAKE_OFF_HEIGHT
		ele->hover_time = DEFAULT_HOVER_TIEM;
		Trajectory_Schedule(ele,MAX_COM_SPEED, Point_to_point);
		ele->heli_step = 5;
		auto_pilot11 = 0;  //,
	}
	//------------------------2015/6/15-------------------------------------        
	else if((2 == auto_pilot11) && (ele->hover_time <=-1)) //,
	{
		ele->next_target_ned[0] = ele->cur_target_pos_ned[0];
		ele->next_target_ned[1] = ele->cur_target_pos_ned[1];
		ele->next_target_ned[2] = -1.0f;   //,1
		ele->hover_time = 400;//4
		Trajectory_Schedule(ele,MAX_COM_SPEED, Point_to_point);
		if(fabs(ele->how_long_body[2]) < 0.4f)
		{
			ele->heli_step = 1;
			Trajectoyr_Current_Target_Update(ele,ele->next_target_ned);
		}
		else
		{
			ele->heli_step = 5;
		}
		auto_pilot11 = 4;  //,    4
	}
	else if((4 == auto_pilot11) && (ele->hover_time <=-1)) //,14
	{
		ele->next_target_ned[0] = ele->cur_target_pos_ned[0];
		ele->next_target_ned[1] = ele->cur_target_pos_ned[0];
		ele->next_target_ned[2] = 0.0f;  
		ele->hover_time = 200;
		ele->how_long_body[2] = -1;
		ele->heli_step = 7;	
		auto_pilot11 = 6;  //,
	}
	//--------------------------------------------------------------------- 
	else if (Trajectory_Target_Update(ele,Point_to_point))		// ele->next_target_ned
	{	
		Trajectory_Schedule(ele,MAX_COM_SPEED, Point_to_point);	// ele->next_target_ned
		if (ele->how_long_body[0] > BOUNDARY_DISTANCE)
		{
			Trajectory_Turn_Heading(ele,&ele->end_heading);
			ele->heli_step = 2;
		}
		if (ele->how_long_body[0] <= BOUNDARY_DISTANCE)
		{
			Trajectoyr_Current_Target_Update(ele,ele->next_target_ned);
			ele->end_heading = ahrs[1].Ang[2]; 
			ele->heli_step = 1;
		} 
	}	
	if (ele->heli_step == 1)
	{	
		Trajectory_Hover_Step(ele);
	}
	if (ele->heli_step == 2)
	{
		Trajectory_Turning_Head_Step(ele);   //		
		if(ele->turn_accel_flag==true)
		{
			ele->turn_accel++;
		}
		if(ele->turn_accel==300)//3 
		{
			ele->turn_accel=0;
			ele->turn_accel_flag=false;
			ele->heli_step = 3;  
		}
	}
	if (ele->heli_step == 3)
	{                
		if (fabs(ele->how_long_body[0]) <= 0.5f * ele->top_speed_body[0] * ele->top_speed_body[0] / ACCEL) //1/4    
			ele->heli_step = 4;
		else
			Trajectory_Speed_Up_Step(ele);

	}
	if (ele->heli_step == 4)
	{
		if ((ele->how_long_body[0]) <= 0.05f || ele->cur_target_vel_body[0] == 0.0f)
		{
			ele->how_long_body[0] = 0.0f ;
			ele->heli_step = 1;
			Trajectoyr_Current_Target_Update(ele,ele->next_target_ned);    
		}
		else
		{
			Trajectory_Speed_Down_Step(ele);      //
		}
	}

	if (ele->heli_step == 5) //
	{
		/************************2015-12-18**********************************/                             
		if (fabs(ele->how_long_body[2]) <= 0.25f * ele->distance_vertical) //1/4    
			ele->heli_step = 6;
		else
			Trajectory_Speed_Up_Step_Vertical(ele);        //
	}

	if (ele->heli_step == 6) //
	{
		/************************2015-12-18**********************************/                             
		if (fabs(ele->how_long_body[2]) <= 0.05f || ele->cur_target_vel_body[2] == 0.0f) //1/4    
		{
			ele->heli_step = 1;
			Trajectoyr_Current_Target_Update(ele,ele->next_target_ned);
		}
		else
		{
		  Trajectory_Speed_Down_Step_Vertical(ele);        //
		}
	}

	if (ele->heli_step == 7)//
	{
		if(fabs(ele->how_long_body[2]) <= 0.01f)
		{
			ele->heli_step = 1;
			auto_pilot11 = 7;//
			Trajectoyr_Current_Target_Update(ele,ele->next_target_ned);
		}
		else
			Trajectory_Land_Step(ele);
	}

	if (ele->heli_step == 8)
	{	
		Trajectory_Brake_Down_Step(ele);
	}
}

void Trajectory_Circle_Mode_Step(sTrajectory *ele)
{
	if (Trajectory_Target_Update(ele,Circle))
	{
		Trajectory_Schedule(ele,ele->circle_velocity, Circle);
		if (ele->how_long_body[0] > 0.5f*ele->circle_velocity*ele->circle_velocity/ACCEL)
		{
			Trajectory_Turn_Heading(ele,&ele->end_heading);
		  ele->heli_step = 2;
		}
		else
		{
			Trajectoyr_Current_Target_Update(ele,ele->next_target_ned);
			ele->end_heading = ahrs[1].Ang[2]; 
			ele->heli_step = 1;
		}

	}
	if (ele->heli_step == 1)
	{
		Trajectory_Hover_Step(ele);
	}
	if (ele->heli_step == 2)
	{
		Trajectory_Turning_Head_Step(ele); 
		if(ele->turn_accel_flag==true)
		{
		  ele->turn_accel++;
		}
		if(ele->turn_accel==300)//3 
		{
			ele->turn_accel=0;
			ele->turn_accel_flag=false;
			ele->heli_step = 3;     
		}
	}
	if (ele->heli_step == 3)
	{
	  Trajectory_Speed_Up_Step(ele);   
		if (abs(ele->how_long_body[0] <= 0.2f))
		{
			ele->heli_step = 9;
			ele->circle_finish_flag = false;
			Trajectoyr_Current_Target_Update(ele,ele->next_target_ned);  
		}
	}
	if (ele->heli_step == 9 && ele->circle_finish_flag == false)
	{
		ele->circle_turn = 1;
		circle_radius_test = 10.0f;
		Trajectory_Circle_Step(ele,ele->circle_velocity, circle_radius_test, 0.0f, 360.0f * D2R);//by wybb
		return;
	}
	if (ele->heli_step == 8)
	{	
		Trajectory_Brake_Down_Step(ele);
	}
}

void Trajectory_Mixture_Mode_Step(sTrajectory *ele)
{
//if (target_update(Mixture))
//{	
//schedule(init_velocity_on_circle, Mixture);
//if (Mixture_to_Pointtopoint_flag == true)
//{
//Mixture_to_Pointtopoint_flag = false;
//if (ele->how_long_body[0] > BOUNDARY_DISTANCE)
//{
//		turn_heading(&ele->end_heading);
//		ele->heli_step = 2;
//	}
//if (ele->how_long_body[0] <= BOUNDARY_DISTANCE)
//	{
//		position_t->current_target_update(ele->next_target_ned);
//		position_t->heading = ahrs_theta[2];
//		ele->end_heading = ahrs_theta[2]; 
//		ele->heli_step = 1;  
//	}
//return;
//}
//if (circle_finish_flag == true)
//{
//circle_finish_flag = false;
//position_t->current_target_update(ele->next_target_ned);
//position_t->heading = ahrs_theta[2];
//ele->heli_step = 3;
//}
//if (ele->how_long_body[0] > 0.5*init_velocity_on_circle*init_velocity_on_circle/ACCEL
//&& circle_finish_flag == false)
//{
//		turn_heading(&ele->end_heading);
//		ele->heli_step = 2;
//}
//else
//{
//		position_t->current_target_update(ele->hover_ned);
//		position_t->heading = ahrs_theta[2];
//		ele->end_heading = ahrs_theta[2]; 
//		ele->heli_step = 1;  
//}
//}
//if (ele->heli_step == 1)
//{
//hover_step();
//}
//if (ele->heli_step == 2)
//{
//turning_head_step();   //

//if(ele->turn_accel_flag==true)
//{
//	ele->turn_accel++;
//}
//if(ele->turn_accel==90)//3 
//{
//	ele->turn_accel=0;
//	ele->turn_accel_flag=false;
//	ele->heli_step = 3;
//	position_t->current_target_update(ele->next_target_ned);      
//}
//}
//if (ele->heli_step == 3)
//{
//				speed_up_step();        //
//if (abs(input_xyz[0]) <= 3 ||input_xyz[0]*last_input_xyz0<0)
//{
//				 ele->how_long_body[0] = 0.0;
//ele->heli_step = 9;
//}
//}
//if (ele->heli_step == 9 && circle_finish_flag == false)
//{
//circle_step(ele->circle_velocity, circle_radius_test, helix_velocity, ele->sector_angle); //by wybb	
//return;
//}
//if (ele->heli_step == 8)
//{	
//brake_down_step();
//}
}

void Trajectory_Test_Mode_Step(sTrajectory *ele)
{
//	if (ele->heli_step == 1)
//	{
//		hover_step();	
//		if (num == 100)
//		{
//			com_theta[0]  = 0.052; //roll            3°
//			com_theta[1]  = -0.052;//pitch          -3°
//			yaw_com_theta = 0.174; //yaw            10°  
//			G_zinit=G_zinit+0.5;   //     
//		}
//		if (num == 160)
//		{
//			com_theta[0]  = -0.052; //roll            -3°
//			com_theta[1]  = 0.052;//pitch              3°
//			yaw_com_theta = -0.174; //yaw            -10°  
//			G_zinit=G_zinit - 1;   //     
//		}
//		if (num == 260)
//		{
//			com_theta[0]  = 0.052; //roll            3°
//			com_theta[1]  = -0.052;//pitch          -3°
//			yaw_com_theta = 0.174; //yaw            10°  
//			G_zinit=G_zinit + 1;   //     
//		}
//		if (num == 360)
//		{
//			com_theta[0]  = -0.052; //roll            -3°
//			com_theta[1]  =  0.052;//pitch              3°
//			yaw_com_theta = -0.174; //yaw            -10°  
//			G_zinit=G_zinit - 1;   //     
//		}
//		if (num == 460)
//		{
//			//
//			com_theta[0]  = 0; //roll            
//			com_theta[1]  = 0;//pitch             
//			yaw_com_theta = 0; //yaw              
//			G_zinit=G_zinit +0.5; //    
//		}
//		}
//}
}

void Trajectory_RTL_Mode_Step(sTrajectory *ele)
{
	if((2 == auto_pilot11) && (ele->hover_time <=-1)) //,
	{
		ele->next_target_ned[0] = ele->cur_target_pos_ned[0];
		ele->next_target_ned[1] = ele->cur_target_pos_ned[1];
		ele->next_target_ned[2] = -1.0;   //,1
		ele->hover_time = 200;//4
		Trajectory_Schedule(ele,MAX_COM_SPEED, Point_to_point);
		if(fabs(ele->how_long_body[2]) < 0.4f)
		{
			ele->heli_step = 1;
			Trajectoyr_Current_Target_Update(ele,ele->next_target_ned);
		}
		else
		{
			ele->heli_step = 5;
		}
		auto_pilot11 = 4;  //,    4
	}
	else if((4 == auto_pilot11) && (ele->hover_time <=-1)) //,14
	{
		ele->next_target_ned[0] = ele->cur_target_pos_ned[0];
		ele->next_target_ned[1] = ele->cur_target_pos_ned[1];
		ele->next_target_ned[2] = 0.0f;  
		ele->hover_time = 100;
		ele->how_long_body[2] = -1;
		ele->heli_step = 7;
		auto_pilot11 = 6;  //,
	}
	else if(ele->RTL_Orignal_update_flag == 0)
	{   //
		ele->next_target_ned[0] = ele->hover_ned[0];
		ele->next_target_ned[1] = ele->hover_ned[1];
		ele->next_target_ned[2] = ele->hover_ned[2];
		ele->hover_time = 200;//4
		Trajectory_Schedule(ele,MAX_COM_SPEED, Point_to_point);	// ele->next_target_ned
		ele->RTL_Orignal_update_flag = 1;
		if (ele->how_long_body[0] > BOUNDARY_DISTANCE)
		{
			Trajectory_Turn_Heading(ele,&ele->end_heading);
			ele->heli_step = 2;
		}
		else if (ele->how_long_body[0] <= BOUNDARY_DISTANCE)
		{
			Trajectoyr_Current_Target_Update(ele,ele->next_target_ned);
			ele->end_heading = ahrs[1].Ang[2]; 
			ele->heli_step = 1;
			ele->RTL_Orignal_flag = 1;
		} 
	}    
	if (ele->heli_step == 1)
	{	
		Trajectory_Hover_Step(ele);
		if((ele->hover_time < -1) && (ele->RTL_Orignal_flag == 1))
		{
			auto_pilot11 = 2;
			ele->RTL_Orignal_flag = 0;
		}
	}

	if (ele->heli_step == 2)
	{
		Trajectory_Turning_Head_Step(ele);  //    
		if(ele->turn_accel_flag==true)
		{
			ele->turn_accel++;
		}
		if(ele->turn_accel==90)//3 
		{
			ele->turn_accel=0;
			ele->turn_accel_flag=false;
			ele->heli_step = 3;    
		}
	}

	if (ele->heli_step == 3)
	{                          
		if (fabs(ele->how_long_body[0]) <= 0.5f * ele->top_speed_body[0] * ele->top_speed_body[0])    
			ele->heli_step = 4;
		else
			Trajectory_Speed_Up_Step_Vertical(ele);          //	
	}

	if (ele->heli_step == 4)
	{ 
		if ((ele->how_long_body[0]) <= 0.05f || ele->cur_target_vel_body[0] == 0.0f)//compass
		{
			ele->how_long_body[0] = 0.0f;
			ele->RTL_Orignal_flag = 1;//
			ele->heli_step = 1;
			Trajectoyr_Current_Target_Update(ele,ele->next_target_ned);
		}
		else
		{
			Trajectory_Speed_Down_Step_Vertical(ele);        //
		}
	}

	if (ele->heli_step == 5) //
	{                           
		if (fabs(ele->how_long_body[2]) <= 0.25f * ele->distance_vertical)   
			ele->heli_step = 6;
		else
			Trajectory_Speed_Up_Step_Vertical(ele);        //
	}

	if (ele->heli_step == 6) //
	{                           
		if (fabs(ele->how_long_body[2]) <= 0.05f || ele->cur_target_vel_body[2] == 0.0f) //1/4    
		{
			ele->heli_step = 1;
			Trajectoyr_Current_Target_Update(ele,ele->next_target_ned);
		}
		else
			Trajectory_Speed_Down_Step_Vertical(ele);         //
	}

	if (ele->heli_step == 7)//
	{
		if(fabs(ele->how_long_body[2]) <= 0.01f)
		{
			ele->heli_step = 1;
			auto_pilot11 = 7;//
			Trajectoyr_Current_Target_Update(ele,ele->next_target_ned);
		}
		else
		{
			Trajectory_Land_Step(ele);
		}
	}
}
