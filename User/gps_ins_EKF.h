#ifndef _GPS_INS_EKF_H
#define _GPS_INS_EKF_H
#include "gps.h"
#include "ahrs.h"

#define IMU_BUFFER_SIZE 42 //26
#define GPS_BUFFER_SIZE 6
#define GPS_DELAYED_TIME 400000 //230000
typedef struct
{
	u32 update_time;
	double acc[3];
	double w_qpr[3];
	double T[9];
}IMU_RING_ELEMENT;

typedef struct
{
	u8  youngest;
	u8  oldest;
	bool is_filled;
  IMU_RING_ELEMENT imu_data[IMU_BUFFER_SIZE];
}STORE_IMU_BUFFER;

typedef struct
{
	u32 update_time;
	double Pos_obs[3];
	double Vel_obs[2];
}GPS_RING_ELEMENT;

typedef struct
{
	u8  youngest;
	u8  oldest;
	bool new_data_flag;
	u32 last_update_time;
	GPS_RING_ELEMENT gps_data[GPS_BUFFER_SIZE];
}STORE_GPS_BUFFER;

typedef struct
{
	double Pos[3];
	double Vel[3];
}OUTPUT_RING_ELEMENT;

typedef struct
{
	u8  youngest;
	u8  oldest;
	OUTPUT_RING_ELEMENT output_data[IMU_BUFFER_SIZE];
}STORE_OUTPUT_BUFFER;

typedef struct
{
	u32 update_time;
	double Height;
}BAR_RING_ELEMENT;


typedef struct
{
	u8  youngest;
	u8  oldest;
	bool new_data_flag;
	BAR_RING_ELEMENT bar_data[IMU_BUFFER_SIZE];
}STORE_BAR_BUFFER;
extern OUTPUT_RING_ELEMENT output_data_new;
extern GPS_RING_ELEMENT gps_data_new;
extern bool ekf_update_flag;
void GPS_INS_EKF(sGPS* ele);
void LLH2NED(sGPS* ele);
#endif
