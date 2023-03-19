/**
  *   @brief �����ǵ��̿��ƴ���
**/
#include "chassis.h"
#include "pid.h"
#include "CAN_receive.h"
#include "stm32f4xx_hal.h"

//*****************PID���Ʋ��ִ���*******************************/
	
pid_type_def              motor_pid;      //����PID���ݽṹ��1
extern PID_TypeDef   drive_motor_pid[4];  //����PID���ݽṹ��2
const motor_measure_t   *motor_data[4];  //��������ṹ��ָ��

/**
  * @brief          PID��ʼ��
  */
void chassis_init(){
	for(int di=0; di<4; di++)
  {
      pid_init(&drive_motor_pid[di]);
      drive_motor_pid[di].f_param_init(&drive_motor_pid[di], PID_Speed, 16384, 5000, 10, 0, 8000, 0,  10, 0.02, 0);	
  }
}

/**
  * @brief          �ĸ������ٶ���ͨ�������������������
  * @param[in]      vx_set: �����ٶ�
  * @param[in]      vy_set: �����ٶ�
  * @param[in]      wz_set: ��ת�ٶ�
  * @param[out]     wheel_speed: �ĸ������ٶ�
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const float wz_set, fp32 wheel_speed[4]){
	
    wheel_speed[0] = (vx_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;
    wheel_speed[1] = (vy_set - (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;
    wheel_speed[2] = (vy_set - (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;
    wheel_speed[3] = (vx_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;	
}

/**
  * @brief          ���͵������ݸ����
  * @param[in]      wheel_speed: �ĸ������ٶ�
  */
static void chassis_control_loop(fp32 wheel_speed[4])  
{
		//���ø�����ĵ�Ŀ��ת��ֵ����λrpm��Ŀǰ��δȷ��ӳ���ϵ
		drive_motor_pid[0].target = wheel_speed[0]*3000/PI;  //   
		drive_motor_pid[1].target = wheel_speed[1]*3000/PI;  //
		drive_motor_pid[2].target = -wheel_speed[2]*3000/PI;  //
		drive_motor_pid[3].target = -wheel_speed[3]*3000/PI;  //
				
		drive_motor_pid[0].f_cal_pid(&drive_motor_pid[0],motor_data[0]->speed_rpm);   //1�ŵ����pid��������ֵ
		drive_motor_pid[1].f_cal_pid(&drive_motor_pid[1],motor_data[1]->speed_rpm);   //2�ŵ����pid��������ֵ
		drive_motor_pid[2].f_cal_pid(&drive_motor_pid[2],motor_data[2]->speed_rpm);   //3�ŵ����pid��������ֵ
		drive_motor_pid[3].f_cal_pid(&drive_motor_pid[3],motor_data[3]->speed_rpm);   //4�ŵ����pid��������ֵ
		
		// ������ֵ���͸����̵��
		CAN_cmd_chassis(drive_motor_pid[0].output,drive_motor_pid[1].output,drive_motor_pid[2].output,drive_motor_pid[3].output);					
		HAL_Delay(2);
}	

/**
  * @brief ִ�е��̿�������
  */

void chassis_task()
{
	//����ǰ�����ң���ת�ٶ�
	fp32 vx_set=0.1f;
	fp32 vy_set=0.1f;
	fp32 wz_set=0.0f;
	
	fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	chassis_vector_to_mecanum_wheel_speed(vx_set, vy_set, wz_set, wheel_speed);
	
	chassis_control_loop(wheel_speed);
}

/**
  * @brief ��ȡ�ĸ������״ֵ̬������debug
  */
void chassis_state(){
	
	motor_data[0] = get_chassis_motor_measure_point(0);//��ȡIDΪ1�ŵĵ������ָ��
	motor_data[1] = get_chassis_motor_measure_point(1);//��ȡIDΪ2�ŵĵ������ָ��
	motor_data[2] = get_chassis_motor_measure_point(2);//��ȡIDΪ3�ŵĵ������ָ��
	motor_data[3] = get_chassis_motor_measure_point(3);//��ȡIDΪ4�ŵĵ������ָ��
}
