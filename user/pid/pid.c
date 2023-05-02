#include "pid.h"
#include "math.h"


//输入的数据保证在正确的范围内

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
	
PID_TypeDef    drive_motor_pid[4];           //四个电机

/*参数初始化-----------------------------*/
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
	
	pid->ControlPeriod = period;  //没用到
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;         //目标值
	
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	pid->output = 0;
}

/*--------------------------------------------------------------

 中途更改参数设定

*/
static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/*pid计算-----------------------------------------------------------------------*/	
static float pid_calculate(PID_TypeDef* pid, float measure)
{
	//  数据的更新
	pid->measure = measure;          //测量值等于本次最新测量值
	pid->last_err  = pid->err;       //上次误差等于本次最新误差
	pid->last_output = pid->output;  //上次输出等于本次最新输出
	
	pid->err = pid->target - pid->measure;  //误差值 = 目标值 - 测量值
	
	//是否进入死区
	if((fabsf(pid->err) > pid->DeadBand))   //误差大于死区
	{
			pid->pout = pid->kp * pid->err;      //p输出为Kp*误差
			pid->iout += (pid->ki * pid->err);   //i输出为i+ki*误差
			pid->dout =  pid->kd * (pid->err - pid->last_err);  //d输出为kd*（误差-上次误差）
			
			//积分是否超出限制
			if(pid->iout > pid->IntegralLimit)
				   pid->iout = pid->IntegralLimit;       
			if(pid->iout < - pid->IntegralLimit)
				   pid->iout = - pid->IntegralLimit;
			
			//pid输出和
			pid->output = pid->pout + pid->iout + pid->dout;   	

			//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //滤波？
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
