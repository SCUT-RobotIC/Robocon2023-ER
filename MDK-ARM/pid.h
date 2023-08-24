
#ifndef PID_H
#define PID_H
#include <stdint.h> 

#define NULL 0
///////////////////////////////////////////////
typedef enum
{

	PID_Position,
	PID_Speed     // 增量式
	
}PID_ID;



typedef struct _PID_TypeDef
{
	PID_ID id;
	
	float target;							//目标值
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;

	float   measure;					//测量值
	float   err;							//误差
	float   last_err;      		//上次误差
	
	float pout;
	float iout;
	float dout;
	
	float output;						//本次输出
	float last_output;			//上次输出
	
	float MaxOutput;				//输出限幅
	float IntegralLimit;		//积分限幅
	float DeadBand;			    //死区（绝对值）
	float ControlPeriod;		//控制周期
	float  Max_Err;					//最大误差
	
	
	uint8_t servo ;
	
	uint32_t thistime;
	uint32_t lasttime;
  uint8_t  dtime;	
	
	void (*f_param_init)(struct _PID_TypeDef *pid,  //PID参数初始化
				   PID_ID id,
				   uint32_t  maxOutput,
				   uint32_t integralLimit,
				   float    deadband,
				   uint16_t controlPeriod,
					 int16_t  max_err,     
					 int16_t  target,
				   float kp,
				   float ki,
				   float kd );
				   
void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		//pid三个参数修改
					 
float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);   //输入速度实测值，输出控制电流值


}PID_TypeDef;

void pid_init(PID_TypeDef* pid);
static float pid_calculate(PID_TypeDef* pid, float measure);		

///////////////////////////////////////////////
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    
	//PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern float PID_calc(pid_type_def *pid, float ref, float set);

/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
