#ifndef _MODEL_H
#define _MODEL_H

#define AngleToPwmAdd(angle) ((angle)/9.0*50.0+1500)
#define AngleToPwm(angle) ((angle)/9.0*50.0)
#define PwmToAngleAdd(pwm) ((((float)pwm)-1500.0)*9.0/50.0)
#define PwmToAngle(pwm) (((float)pwm)*9.0/50.0)
#define ABS(x) ( (x)>0?(x):-(x) )
#define VAL_LIMIT( x,min,max ) x=( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

//PID控制器参数
typedef struct
{
	//用户可调参数
	float kp;
	float ki;
	float kd;
	float IntiMax;//积分限幅
	//PID控制器输入量
	float err;
	//PID控制器输出量
	float out;
	//中间变量
	float integral;//积分项输出
	float derivativeOut;//微分项输出
	float derivativeInti;//微分回路中的积分器输出
}PIDparameter;

//双P控制器参数
typedef struct
{
	//用户可调参数
	float kp;//内环P
	//用户不可调参数
	float InerOut;//内环输出
	float OuterOut;//外环输出
	float OuterP;//外环P
}Pparameter;

void Delay_ms(unsigned short time_ms);
void PID_Controller(PIDparameter *pid);
void PID_Reset(PIDparameter *pid);
void P_Reset(Pparameter *p);

#endif
