#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "mb_structs.h"
#define CFG_PATH "pid.cfg"

int mb_controller_odometry_init();
int mb_controller_odometry_load_config();
int mb_controller_odometry_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints);
int mb_controller_odometry_cleanup();

#endif

