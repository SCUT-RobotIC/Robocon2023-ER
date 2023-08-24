/**
 * @brief 使用非arm——math库实现pid控制
 * @detail 使用了pitch.c 内部分函数 ， 后续考虑和pitch.c合并
 * @attention 此程序适用于位置式编码器的控制系统
 * @author Jiao
 
 */
 
 #include "pid.h"
 #include "pitch.h"
 #include "pitch_init.h"
 
 
 
 PID_TypeDef    drive_pitch_pid[1];
 
 
 void pitch_init(){
	 
	 pid_init(&drive_pitch_pid[0]);
	 
	 drive_pitch_pid[0].f_param_init(&drive_pitch_pid[0], PID_Speed, 400 ,    200, 2, 0, 100, 0,  10 ,0.2, 0);//1,0,0	
	 //                                                           maxout   interlim deadb
	drive_pitch_pid[0].servo = 1;
}

void pid_pitch_control(float target_position,int state_reload)  //200 - 150
{
		/// 设置各电机的的目标转速值（单位rpm）目前还未确定映射关系
		/*drive_motor_pid[0].target = wheel_speed[0]*30/PI;  //   
		drive_motor_pid[1].target = wheel_speed[1]*30/PI;  //
		drive_motor_pid[2].target = wheel_speed[2]*30/PI;  //
		drive_motor_pid[3].target = wheel_speed[3]*30/PI;  //
			*/
	
		float measurement; 
	
		if(state_reload == -1 || state_reload == 1){
		
		drive_pitch_pid[0].f_param_init(&drive_pitch_pid[0], PID_Speed, 400 ,    200, 2, 0, 100, 0,  10 ,0.2, 0);//1,0,0	
		
		}
		
		drive_pitch_pid[0].target = target_position;     
		
	
		measurement = Pitch_Get_Position(0); // so just zero , 
	
		drive_pitch_pid[0].f_cal_pid(&drive_pitch_pid[0],measurement);   // pitch pid cc
	
		//pitch start 
		PITCH_SetThrottle(0, drive_pitch_pid[0].output);
	
		//close it 
	
		//PITCH_SetThrottle(0, target_position);
		
		PITCH_Run(0);
		
}	