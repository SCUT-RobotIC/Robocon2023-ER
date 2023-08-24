#include "chassis.h"
#include "pid.h"
#include "bsp_can.h"
#include "stm32f4xx_hal.h"


PID_TypeDef    drive_motor_pid[4];
extern const motor_measure_t   *motor_data[4];


void chassis_init(){
	for(int di=0; di<4; di++)	
  {
      pid_init(&drive_motor_pid[di]);
      drive_motor_pid[di].f_param_init(&drive_motor_pid[di], PID_Speed, 16384, 5000, 10, 0, 8000, 0,  5 ,0.02, 0);//10,0.02,0	
		
		// 先只管P
  }
}

float vx=0.0f;
float vy=0.0f;
float wz=0.0f;
float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};

//速度-> rpm


void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4]){
/*	
    wheel_speed[0] = 19*(vx_set - (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;
    wheel_speed[1] = -19*(vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;
    wheel_speed[2] = -19*(vx_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;
    wheel_speed[3] = 19*(vy_set - (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;	
*/
		
	

	
	//        O
	//     2  +  3  
	//     1  |  4
  //        V 
	// plus :  4 3 的方向是反的
	
	
	
		wheel_speed[0] = -vx_set;
		wheel_speed[1] = -vx_set;
		wheel_speed[2] = vx_set;
		wheel_speed[3] = vx_set;
		
		
		
		
		
		wheel_speed[0] += vy_set ;
		wheel_speed[1] -= vy_set ;
		wheel_speed[2] += -vy_set ;
		wheel_speed[3] -= -vy_set ;
		
		wheel_speed[0] += wz_set ;
		wheel_speed[1] += wz_set ;
		wheel_speed[2] -= -wz_set ;
		wheel_speed[3] -= -wz_set ;
		
		
 		
	
	
	}

	
	/**
  * @brief          发送电流数据给电机
  * @param[in]      wheel_speed: 四个轮子速度
	* @note           底盘的轮子正电流为顺时针转动
	* @warning        一定要使用速度环PID控制电机，直接用电流控制会导致电机全速运转 
  */
void chassis_control_loop(float wheel_speed[4])  
{
		/// 设置各电机的的目标转速值（单位rpm）目前还未确定映射关系
		/*drive_motor_pid[0].target = wheel_speed[0]*30/PI;  //   
		drive_motor_pid[1].target = wheel_speed[1]*30/PI;  //
		drive_motor_pid[2].target = wheel_speed[2]*30/PI;  //
		drive_motor_pid[3].target = wheel_speed[3]*30/PI;  //
			*/
	
		drive_motor_pid[0].target = wheel_speed[0];     
		drive_motor_pid[1].target = wheel_speed[1]; // 
		drive_motor_pid[2].target = wheel_speed[2];  
		drive_motor_pid[3].target = wheel_speed[3];
	
		drive_motor_pid[0].f_cal_pid(&drive_motor_pid[0],motor_data[0]->speed_rpm);   //1号电机的pid电流计算值
		drive_motor_pid[1].f_cal_pid(&drive_motor_pid[1],motor_data[1]->speed_rpm);   //2号电机的pid电流计算值
		drive_motor_pid[2].f_cal_pid(&drive_motor_pid[2],motor_data[2]->speed_rpm);   //3号电机的pid电流计算值
		drive_motor_pid[3].f_cal_pid(&drive_motor_pid[3],motor_data[3]->speed_rpm);   //4号电机的pid电流计算值
		
		/// 将电流值发送给底盘电机
		CAN_cmd_chassis(drive_motor_pid[0].output,drive_motor_pid[1].output,drive_motor_pid[2].output,drive_motor_pid[3].output);					
		//HAL_Delay(2);
	
}	




/**
* @brief 获取四个电机的状态值，便于debug ..一般不用
  */

void chassis_state_update(){
	
	motor_data[0] = get_chassis_motor_measure_point(0);//获取ID为1号的电机数据指针
	motor_data[1] = get_chassis_motor_measure_point(1);//获取ID为2号的电机数据指针
	motor_data[2] = get_chassis_motor_measure_point(2);//获取ID为3号的电机数据指针
	motor_data[3] = get_chassis_motor_measure_point(3);//获取ID为4号的电机数据指针
}
	
//
void compute_pid()
{
		motor_data[0] = get_chassis_motor_measure_point(0);//获取ID为1号的电机数据指针
		motor_data[1] = get_chassis_motor_measure_point(1);//获取ID为2号的电机数据指针
	  motor_data[2] = get_chassis_motor_measure_point(2);//获取ID为3号的电机数据指针
	  motor_data[3] = get_chassis_motor_measure_point(3);//获取ID为4号的电机数据指针
	
	  drive_motor_pid[0].f_cal_pid(&drive_motor_pid[0],motor_data[0]->speed_rpm);   //1号电机的pid电流计算值
		drive_motor_pid[1].f_cal_pid(&drive_motor_pid[1],motor_data[1]->speed_rpm);   //2号电机的pid电流计算值
		drive_motor_pid[2].f_cal_pid(&drive_motor_pid[2],motor_data[2]->speed_rpm);   //3号电机的pid电流计算值
		drive_motor_pid[3].f_cal_pid(&drive_motor_pid[3],motor_data[3]->speed_rpm);   //4号电机的pid电流计算值
}



void send_can()
{
	/// 将电流值发送给底盘电机
		CAN_cmd_chassis(drive_motor_pid[0].output,drive_motor_pid[1].output,drive_motor_pid[2].output,drive_motor_pid[3].output);					
		//HAL_Delay(2);
}

