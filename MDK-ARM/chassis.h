#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "pid.h"

#include "bsp_can.h"
/*
#define CHASSIS_WZ_SET_SCALE 0.1f	
#define MOTOR_DISTANCE_TO_CENTER 0.55273f
#define Wheel_Radius 0.076f
#define PI 3.1415926f

*/
extern PID_TypeDef    drive_motor_pid[4]; 

void chassis_init(void);
void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4]);
void chassis_control_loop(float wheel_speed[4]);
void chassis_task(const float	 vx_set, const float vy_set, const float wz_set, float wheel_speed[4]);

void chassis_state_update(void);
void compute_pid(void);
void send_can(void);
void chassis_sendcan(void);

#endif
