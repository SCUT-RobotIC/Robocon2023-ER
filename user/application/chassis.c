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

//uint16_t remotedata[13];
fp32 vx=0.0f;
fp32 vy=0.0f;
fp32 wz=0.0f;
fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
/**
  * @brief          �ĸ������ٶ���ͨ�������������������
  * @param[in]      vx_set: �����ٶ�
  * @param[in]      vy_set: �����ٶ�
  * @param[in]      wz_set: ��ת�ٶ�
  * @param[out]     wheel_speed: �ĸ������ٶ�
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const float wz_set, fp32 wheel_speed[4]){
	
    wheel_speed[0] = 19*(vx_set - (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;
    wheel_speed[1] = -19*(vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;
    wheel_speed[2] = -19*(vx_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;
    wheel_speed[3] = 19*(vy_set - (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER *wz_set)/Wheel_Radius;	
}

/**
  * @brief          ���͵������ݸ����
  * @param[in]      wheel_speed: �ĸ������ٶ�
	* @note           ���̵�����������Ϊ˳ʱ��ת��
	* @warning        һ��Ҫʹ���ٶȻ�PID���Ƶ����ֱ���õ������ƻᵼ�µ��ȫ����ת 
  */
static void chassis_control_loop(fp32 wheel_speed[4])  
{
		/// ���ø�����ĵ�Ŀ��ת��ֵ����λrpm��Ŀǰ��δȷ��ӳ���ϵ
		drive_motor_pid[0].target = wheel_speed[0]*30/PI;  //   
		drive_motor_pid[1].target = wheel_speed[1]*30/PI;  //
		drive_motor_pid[2].target = wheel_speed[2]*30/PI;  //
		drive_motor_pid[3].target = wheel_speed[3]*30/PI;  //
				
		drive_motor_pid[0].f_cal_pid(&drive_motor_pid[0],motor_data[0]->speed_rpm);   //1�ŵ����pid��������ֵ
		drive_motor_pid[1].f_cal_pid(&drive_motor_pid[1],motor_data[1]->speed_rpm);   //2�ŵ����pid��������ֵ
		drive_motor_pid[2].f_cal_pid(&drive_motor_pid[2],motor_data[2]->speed_rpm);   //3�ŵ����pid��������ֵ
		drive_motor_pid[3].f_cal_pid(&drive_motor_pid[3],motor_data[3]->speed_rpm);   //4�ŵ����pid��������ֵ
		
		/// ������ֵ���͸����̵��
		CAN_cmd_chassis(drive_motor_pid[0].output,drive_motor_pid[1].output,drive_motor_pid[2].output,drive_motor_pid[3].output);					
		HAL_Delay(2);
}	


/**
  * @brief ִ�е��̿�������
  */

void chassis_task()
{	
	chassis_vector_to_mecanum_wheel_speed(vx, vy, wz, wheel_speed);	
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
	
static void compute_pid()
{
		motor_data[0] = get_chassis_motor_measure_point(0);//��ȡIDΪ1�ŵĵ������ָ��
		motor_data[1] = get_chassis_motor_measure_point(1);//��ȡIDΪ2�ŵĵ������ָ��
	  motor_data[2] = get_chassis_motor_measure_point(2);//��ȡIDΪ3�ŵĵ������ָ��
	  motor_data[3] = get_chassis_motor_measure_point(3);//��ȡIDΪ4�ŵĵ������ָ��
	
	  drive_motor_pid[0].f_cal_pid(&drive_motor_pid[0],motor_data[0]->speed_rpm);   //1�ŵ����pid��������ֵ
		drive_motor_pid[1].f_cal_pid(&drive_motor_pid[1],motor_data[1]->speed_rpm);   //2�ŵ����pid��������ֵ
		drive_motor_pid[2].f_cal_pid(&drive_motor_pid[2],motor_data[2]->speed_rpm);   //3�ŵ����pid��������ֵ
		drive_motor_pid[3].f_cal_pid(&drive_motor_pid[3],motor_data[3]->speed_rpm);   //4�ŵ����pid��������ֵ
}
void chassis_pid()
{
	compute_pid();
}

static void send_can()
{
	/// ������ֵ���͸����̵��
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
