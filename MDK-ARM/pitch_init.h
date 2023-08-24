#ifndef __PITCH_INIT_H
#define __PITCH_INIT_H


#include "pid.h"

#include "pitch.h"


extern PID_TypeDef    drive_pitch_pid[1];
void pitch_init(void);

void pid_pitch_control(float target_position,int state_reload) ;










#endif