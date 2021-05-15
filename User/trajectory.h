#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_
#include "target.h"
#include "ahrs.h"
#include "helicopter.h"

#define FLY_TIME            500    
#define ACCEL               1.0f
#define MAX_COM_SPEED       3.0f 
#define BOUNDARY_DISTANCE   2.0f
#define DEFAULT_HOVER_TIEM  500

#define ACCEL_VERTICAL               0.5f
#define MAX_COM_SPEED_VERTICAL       1.0f
#define BOUNDARY_DISTANCE_VERTICAL   1.0f

#define PILOT_HEIGHT        1.0f  
#define TAKE_OFF_HEIGHT     1.0f  
extern unsigned char auto_pilot11;
enum  Trajectory_mod
{
	Hover,                      //0
	Point_to_point,		    //1
	Circle,			    //2
	Mixture,		    //3
	Test,			    //4
  RTL,                        //5
	False
};

typedef struct 
{
	float        top_speed_body[3];
	float        how_long_body[3] ;

	Target_Queue* target_queue_t;

	float        next_target_ned[4];
	float        end_heading;
	float        hover_time;
	float        hover_ned[3];      //,
	float        cur_target_vel_body[3];
	float        cur_target_vel_ned[3];
	float        cur_target_pos_ned[3];
	float        com_pqr[3];
	float        com_pqr_ned[3];
	float        com_theta[3];
	int          heli_step;
	int          Isturn; 			
	float        yaw_com_theta;
	enum Trajectory_mod trajectory_mod;  
	float   		 sector_angle;    //
	float   		 circle_velocity; //
	float 		   helix_velocity;  //
	float				 circle_radius;	 //
	float        circle_angle;
	float 		   next_next_target[4];
	int          circle_turn;
	float        distance_line; //
	float        distance_vertical;//
	int 				 vertical_speed_direction;//
	int 				 RTL_Orignal_flag; //RTL 
	int 				 RTL_Orignal_update_flag;//RTL 
	int 				 RTL_First_switch;
	int 				 first_receive_target;
	int 				 turn_accel;
	bool         turn_accel_flag;
	int          first_entry_brake;
	int          first_entry_circle;
	int					 cushion_flag;
	bool         circle_finish_flag;
}sTrajectory;

extern sTrajectory trajectory;
void Trajectory_Reset(sTrajectory *ele);
void Trajectory_Step(sTrajectory *ele,int trajectory_mode);
void Trajectory_Point_to_Point_Mode_Step(sTrajectory *ele);
void Trajectory_Hover_Mode_Step(sTrajectory *ele);
void Trajectory_Point_to_point_Mode_Step(sTrajectory *ele);
void Trajectory_Circle_Mode_Step(sTrajectory *ele);
void Trajectory_Mixture_Mode_Step(sTrajectory *ele);
void Trajectory_Test_Mode_Step(sTrajectory *ele);
void Trajectory_RTL_Mode_Step(sTrajectory *ele);
void Trajectory_Brake_Down_Step(sTrajectory *ele);
void Trajectory_Schedule(sTrajectory *ele,int max_speed, int mode);
#endif
