#include "flight_mode.h"

/***************************************************\
���ܣ�
  ֱ��������ģʽѡ��
˵����
  1���ֿ� 2����̬ 3������
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
