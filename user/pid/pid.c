#include "pid.h"
#include "math.h"


//��������ݱ�֤����ȷ�ķ�Χ��

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
	
PID_TypeDef    drive_motor_pid[4];           //�ĸ����

/*������ʼ��-----------------------------*/
static void pid_param_init(
	PID_TypeDef * pid, 
	PID_ID    id,
	uint32_t  maxout,
	uint32_t  intergral_limit,
	float     deadband,
	uint16_t  period,
	int16_t   max_err,
	int16_t   target,

	float 	kp, 
	float 	ki, 
	float 	kd)
{
	pid->id = id;		
	
	pid->ControlPeriod = period;  //û�õ�
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;         //Ŀ��ֵ
	
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	pid->output = 0;
}

/*--------------------------------------------------------------

 ��;���Ĳ����趨

*/
static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/*pid����-----------------------------------------------------------------------*/	
static float pid_calculate(PID_TypeDef* pid, float measure)
{
	//  ���ݵĸ���
	pid->measure = measure;          //����ֵ���ڱ������²���ֵ
	pid->last_err  = pid->err;       //�ϴ������ڱ����������
	pid->last_output = pid->output;  //�ϴ�������ڱ����������
	
	pid->err = pid->target - pid->measure;  //���ֵ = Ŀ��ֵ - ����ֵ
	
	//�Ƿ��������
	if((fabsf(pid->err) > pid->DeadBand))   //����������
	{
			pid->pout = pid->kp * pid->err;      //p���ΪKp*���
			pid->iout += (pid->ki * pid->err);   //i���Ϊi+ki*���
			pid->dout =  pid->kd * (pid->err - pid->last_err);  //d���Ϊkd*�����-�ϴ���
			
			//�����Ƿ񳬳�����
			if(pid->iout > pid->IntegralLimit)
				   pid->iout = pid->IntegralLimit;       
			if(pid->iout < - pid->IntegralLimit)
				   pid->iout = - pid->IntegralLimit;
			
			//pid�����
			pid->output = pid->pout + pid->iout + pid->dout;   	

			//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //�˲���
			if(pid->output>pid->MaxOutput)         
			{
				   pid->output = pid->MaxOutput;
			}
			if(pid->output < -(pid->MaxOutput))
			{
				   pid->output = -(pid->MaxOutput);
			}
	
	}
	return pid->output;
}


void pid_init(PID_TypeDef* pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}
