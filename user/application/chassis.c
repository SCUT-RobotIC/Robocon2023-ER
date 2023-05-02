/**
  *   @brief 这里是底盘控制代码
**/
#include "chassis.h"
#include "pid.h"
#include "CAN_receive.h"
#include "stm32f4xx_hal.h"

//*****************PID控制部分代码*******************************/
	
pid_type_def              motor_pid;      //声明PID数据结构体1
extern PID_TypeDef   drive_motor_pid[4];  //声明PID数据结构体2
const motor_measure_t   *motor_data[4];  //声明电机结构体指针

/**
  * @brief          PID初始化
  */
void chassis_init(){
	for(int di=0; di<4; di++)
  {
      pid_init(&drive_motor_pid[di]);
      drive_motor_pid[di].f_param_init(&drive_motor_pid[di], PID_Speed, 16384, 5000, 10, 0, 8000, 0,  10, 0.02, 0);	
  }
}

//uint16_t remotedata[13];
fp32 vx=0.0f;
fp32 vy=0.0f;
fp32 wz=0.0f;
fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
/**
  * @brief          四个麦轮速度是通过三个参数计算出来的
  * @param[in]      vx_set: 纵向速度
  * @param[in]      vy_set: 横向速度
  * @param[in]      wz_set: 旋转速度
  * @param[out]     wheel_speed: 四个轮子速度
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const float wz_set, fp32 wheel_speed[4]){
	
    wheel_speed[0] = 19*(vx_set - (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;
    wheel_speed[1] = -19*(vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;
    wheel_speed[2] = -19*(vx_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;
    wheel_speed[3] = 19*(vy_set - (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;	
}

/**
  * @brief          发送电流数据给电机
  * @param[in]      wheel_speed: 四个轮子速度
	* @note           底盘的轮子正电流为顺时针转动
	* @warning        一定要使用速度环PID控制电机，直接用电流控制会导致电机全速运转 
  */
static void chassis_control_loop(fp32 wheel_speed[4])  
{
		/// 设置各电机的的目标转速值（单位rpm）目前还未确定映射关系
		drive_motor_pid[0].target = wheel_speed[0]*30/PI;  //   
		drive_motor_pid[1].target = wheel_speed[1]*30/PI;  //
		drive_motor_pid[2].target = wheel_speed[2]*30/PI;  //
		drive_motor_pid[3].target = wheel_speed[3]*30/PI;  //
				
		drive_motor_pid[0].f_cal_pid(&drive_motor_pid[0],motor_data[0]->speed_rpm);   //1号电机的pid电流计算值
		drive_motor_pid[1].f_cal_pid(&drive_motor_pid[1],motor_data[1]->speed_rpm);   //2号电机的pid电流计算值
		drive_motor_pid[2].f_cal_pid(&drive_motor_pid[2],motor_data[2]->speed_rpm);   //3号电机的pid电流计算值
		drive_motor_pid[3].f_cal_pid(&drive_motor_pid[3],motor_data[3]->speed_rpm);   //4号电机的pid电流计算值
		
		/// 将电流值发送给底盘电机
		CAN_cmd_chassis(drive_motor_pid[0].output,drive_motor_pid[1].output,drive_motor_pid[2].output,drive_motor_pid[3].output);					
		HAL_Delay(2);
}	


/**
  * @brief 执行底盘控制任务
  */

void chassis_task()
{	
	chassis_vector_to_mecanum_wheel_speed(vx, vy, wz, wheel_speed);	
	chassis_control_loop(wheel_speed);
}

/**
  * @brief 获取四个电机的状态值，便于debug
  */
void chassis_state(){
	
	motor_data[0] = get_chassis_motor_measure_point(0);//获取ID为1号的电机数据指针
	motor_data[1] = get_chassis_motor_measure_point(1);//获取ID为2号的电机数据指针
	motor_data[2] = get_chassis_motor_measure_point(2);//获取ID为3号的电机数据指针
	motor_data[3] = get_chassis_motor_measure_point(3);//获取ID为4号的电机数据指针
}
	
static void compute_pid()
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
void chassis_pid()
{
	compute_pid();
}

static void send_can()
{
	/// 将电流值发送给底盘电机
		CAN_cmd_chassis(drive_motor_pid[0].output,drive_motor_pid[1].output,drive_motor_pid[2].output,drive_motor_pid[3].output);					
		HAL_Delay(2);
}
void chassis_sendcan()
{
	send_can();
}
	
static void change_speed()
{
	drive_motor_pid[0].target = wheel_speed[0]*30/PI;  //   
	drive_motor_pid[1].target = wheel_speed[1]*30/PI;  //
	drive_motor_pid[2].target = wheel_speed[2]*30/PI;  //
	drive_motor_pid[3].target = wheel_speed[3]*30/PI;  //
}
void chassis_changeSpeed(const uint16_t vx_set, const uint16_t vy_set, const uint16_t wz_set)
{
	if((float)wz_set>=33535)
		wz=((float)wz_set - 33535.0f)/32000.0f;				
	else if((float)wz_set<=32000)
		wz=-(32000-(float)wz_set)/32000.0f;						
	else wz=0;
					
	vx=-(((float)vx_set)/65535.0f-0.5f);
	vy=((float)vy_set)/65535.0f-0.5f;
	
	chassis_vector_to_mecanum_wheel_speed(vx, vy, wz, wheel_speed);
	change_speed();
}
