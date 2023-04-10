#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "pid.h"
#include "CAN_receive.h"

#define CHASSIS_WZ_SET_SCALE 0.1f	
#define MOTOR_DISTANCE_TO_CENTER 0.55273f
#define Wheel_Radius 0.076f
#define PI 3.1415926f

void chassis_init(void);
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const float wz_set, fp32 wheel_speed[4]);
static void chassis_control_loop(fp32 wheel_speed[4]);
void chassis_task(void);
void chassis_state(void);
static void compute_pid(void);
void chassis_pid(void);
static void send_can(void);
void chassis_sendcan(void);
static void change_speed(void);
void chassis_changeSpeed(const uint16_t vx_set, const uint16_t vy_set, const uint16_t wz_set);
#endif
