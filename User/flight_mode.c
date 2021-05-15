#include "flight_mode.h"

/***************************************************\
功能：
  直升机飞行模式选择
说明：
  1、手控 2、姿态 3、定点
\***************************************************/
void update_flight_mode(void)
{
	switch(rc->Key[2])
	{
		case 1:
			Heli.control_mode = MODE_MANUAL;
			Heli_Manual_control(&Heli);
			break;
		case 2:
			Heli.control_mode = MODE_MANUAL;
			Heli_Manual_control(&Heli);
//			Heli.control_mode = MODE_ATTITUDE;
	//		Heli_Attitude_control(&Heli);
			break;
		case 3:
			Heli.control_mode = MODE_POSITION_HOLD;
			Heli_Poshold_control(&Heli);
		//Heli_Manual_control(&Heli);
			break;
		default:
			break;
	}
}

/*========================================END OF FILE========================================*/
