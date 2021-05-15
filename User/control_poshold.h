#ifndef _CONTROL_POSHOLD_H
#define _CONTROL_POSHOLD_H

#include "helicopter.h"
#include "ahrs.h"
#include "control_stabilize.h"
#include "gps.h"
#include "trajectory.h"
#include "transfer.h"
#include "target.h"
#define COLL_AMPLIFY 30
void Heli_Poshold_control(sHeli *ele);
#endif
