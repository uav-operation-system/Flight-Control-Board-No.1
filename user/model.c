#include "model.h"
/**************文件说明**********************
延时、PID控制器、双P控制器、限幅器、单位转换
********************************************/
void Delay_ms(unsigned short time_ms)
{
  unsigned short i,j;
  for( i=0;i<time_ms;i++ )
  {
		for( j=0;j<10309;j++ );
  }
}

void PID_Controller(PIDparameter *pid)
{
	pid->derivativeInti += 0.5 * pid->derivativeOut;
	pid->derivativeOut = pid->err - pid->derivativeInti;
	if(ABS(pid->err)<10)
	{
		pid->integral += 0.1* pid->err;
		VAL_LIMIT(pid->integral , -pid->IntiMax , pid->IntiMax);
		pid->out = pid->kp * pid->err + pid->ki * pid->integral + pid->kd * pid->derivativeOut;
	}
	else
	{
		pid->integral=0;
		pid->out = pid->kp * pid->err + pid->kd * pid->derivativeOut;
	}
}
void PID_Reset(PIDparameter *pid)
{
	pid->integral=0;
	pid->derivativeInti=0;
	pid->derivativeOut=0;
	pid->err=0;
	pid->out=0;
}
void P_Reset(Pparameter *p)
{
	p->InerOut=0;
	p->OuterP=1/(p->kp);
	p->OuterOut=0;
}
