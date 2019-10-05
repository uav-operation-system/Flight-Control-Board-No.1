#include "task.h"
/**************文件说明**********************
控制飞行器的4个函数，分别为电机控制的内、外环和舵机控制的内、外环
Update_Iner_loop();   电机内环控制
Update_Outer_Loop();  电机外环控制
Aileron_Control();    舵机内环控制
Outer_Aileron();      舵机外环控制
与task.c共享头文件task.h。
该文件内对程序的注释用文字来表述过于苍白，因此附上simulink和stateflow的框图文件取代文字作为注释。若未附上文件则自带加密效果。
********************************************/
short PwmOut[8]={1000,1000,1000,1000,1000,1000,1000,1000};  //把值赋给定时器，输出PWM
/***********************
*@function:速度内环更新，输出控制电机转速
*@period:20ms
**********************/
void Update_Iner_loop(void)
{
	switch(AircraftMode)
	{
	case SIZHOU:
		if(!Armed)
		{
			PwmOut[0]=1000;
			PwmOut[1]=1000;
			PwmOut[2]=1000;
			PwmOut[3]=1000;
			TIM3->CCR1=PwmOut[0];
			TIM3->CCR2=PwmOut[1];
			TIM3->CCR3=PwmOut[2];
			TIM3->CCR4=PwmOut[3];
			return;
		}
		if(PwmInTemp[2]<LOWSPEED)
		{
			PwmOut[0]=LOWSPEED;
			PwmOut[1]=LOWSPEED;
			PwmOut[2]=LOWSPEED;
			PwmOut[3]=LOWSPEED;
			TIM3->CCR1=PwmOut[0];
			TIM3->CCR2=PwmOut[1];
			TIM3->CCR3=PwmOut[2];
			TIM3->CCR4=PwmOut[3];
			return;
		}
		PidInerRoll.err=PidOuterRoll.out-GYRO_TO_DEG(gyrox);
		PidInerPitch.err=PidOuterPitch.out-GYRO_TO_DEG(gyroy);
		PID_Controller(&PidInerRoll);
		PID_Controller(&PidInerPitch);
		PwmOut[0]=AngleToPwmAdd(PwmToAngleAdd(PwmInTemp[2])+PidInerPitch.out)-0.2*(PwmInTemp[3]-1500);
		PwmOut[1]=AngleToPwmAdd(PwmToAngleAdd(PwmInTemp[2])-PidInerPitch.out)-0.2*(PwmInTemp[3]-1500);
		PwmOut[2]=AngleToPwmAdd(PwmToAngleAdd(PwmInTemp[2])-PidInerRoll.out) +0.2*(PwmInTemp[3]-1500);
		PwmOut[3]=AngleToPwmAdd(PwmToAngleAdd(PwmInTemp[2])+PidInerRoll.out) +0.2*(PwmInTemp[3]-1500);
		VAL_LIMIT(PwmOut[0],LOWSPEED,2000);
		VAL_LIMIT(PwmOut[1],LOWSPEED,2000);
		VAL_LIMIT(PwmOut[2],LOWSPEED,2000);
		VAL_LIMIT(PwmOut[3],LOWSPEED,2000);
		TIM3->CCR1=PwmOut[0];
		TIM3->CCR2=PwmOut[1];
		TIM3->CCR3=PwmOut[2];
		TIM3->CCR4=PwmOut[3];
		break;
	case CHUIQI:
		if(!Armed)
		{
			PwmOut[0]=1000;
			PwmOut[1]=1000;
			TIM3->CCR1=PwmOut[0];
			TIM3->CCR2=PwmOut[1];
			return;
		}
		if(PwmInTemp[2]<LOWSPEED)
		{
			PwmOut[0]=LOWSPEED;
			PwmOut[1]=LOWSPEED;
			TIM3->CCR1=PwmOut[0];
			TIM3->CCR2=PwmOut[1];
			return;
		}
		PidInerRoll.err=PidOuterRoll.out-GYRO_TO_DEG(gyrox);
		PID_Controller(&PidInerRoll);
		PwmOut[0]=PwmInTemp[2]+AngleToPwm(PidInerRoll.out);
		PwmOut[1]=PwmInTemp[2]-AngleToPwm(PidInerRoll.out);
		VAL_LIMIT(PwmOut[0],LOWSPEED,2000);
		VAL_LIMIT(PwmOut[1],LOWSPEED,2000);
		TIM3->CCR1=PwmOut[0];
		TIM3->CCR2=PwmOut[1];
		break;
	case YUYING:
		Undead=(PwmInTemp[6]<LOW_THRESHOLD)? 1 : 0;
		if(!Armed)
		{
			PwmOut[0]=1000;
			PwmOut[1]=1000;
			TIM3->CCR1=PwmOut[0];
			TIM3->CCR2=PwmOut[1];
			return;
		}
		if(PwmInTemp[6]>LOW_THRESHOLD)//旋翼
		{
			if(PwmInTemp[2]<LOWSPEED)
			{
				PwmOut[0]=LOWSPEED;
				PwmOut[1]=LOWSPEED;
				TIM3->CCR1=PwmOut[0];
				TIM3->CCR2=PwmOut[1];
				return;
			}
			PidInerRoll.err=PidOuterRoll.out+GYRO_TO_DEG(gyrox);
			PID_Controller(&PidInerRoll);
			PwmOut[0]=PwmInTemp[2]+AngleToPwm(PidInerRoll.out);
			PwmOut[1]=PwmInTemp[2]-AngleToPwm(PidInerRoll.out);
			VAL_LIMIT(PwmOut[0],LOWSPEED,2000);
			VAL_LIMIT(PwmOut[1],LOWSPEED,2000);
			TIM3->CCR1=PwmOut[0];
			TIM3->CCR2=PwmOut[1];
		}
		else//固定翼
		{
			PwmOut[0]=PwmInTemp[2]-PwmInTemp[3]+1500;
			PwmOut[1]=PwmInTemp[2]+PwmInTemp[3]-1500;
			VAL_LIMIT(PwmOut[0],1000,2000);
			VAL_LIMIT(PwmOut[1],1000,2000);
			TIM3->CCR1=PwmOut[0];
			TIM3->CCR2=PwmOut[1];
		}
		break;
	case ZHIFEIJI:
	case LAOZUZONG:
	case DUIDI:
		if(!Armed)
		{
			PwmOut[2]=1000;
			TIM3->CCR3=PwmOut[2];
			return;
		}
		PwmOut[2]=PwmInTemp[2];
		VAL_LIMIT(PwmOut[2],1000,2000);
		TIM3->CCR3=PwmOut[2];
		break;
	default:break;
	}
}
/***********************
*@function:姿态外环更新，输出与角速度做差输入速度内环
*@period:50ms
**********************/
void Update_Outer_Loop(void)
{
	if(!Armed)return;
	switch(AircraftMode)
	{
	case SIZHOU:
		PidOuterRoll.err=PwmToAngleAdd(PwmInTemp[0])-roll;
		PidOuterPitch.err=-pitch-PwmToAngleAdd(PwmInTemp[1]);
		PID_Controller(&PidOuterRoll);
		PID_Controller(&PidOuterPitch);
		break;
	case CHUIQI:
		PidOuterRoll.err=PwmToAngleAdd(3000-PwmInTemp[0])-roll;
		PID_Controller(&PidOuterRoll);
		break;
	case YUYING:
		PidOuterRoll.err=roll-PwmToAngleAdd(PwmInTemp[0]);
		PID_Controller(&PidOuterRoll);
		break;
	case ZHIFEIJI:break;
	case LAOZUZONG:break;
	case DUIDI:break;
	default:break;
	}
}
/***********************
*@function:控制横滚、俯仰、偏航，内环控制
*@period:50ms
**********************/
void Aileron_Control(void)
{
	switch(AircraftMode)
	{
	case SIZHOU:break;
	case CHUIQI:
		pPitch.InerOut=pPitch.kp*(PidOuterPitch.out-GYRO_TO_DEG(gyroy));
		pYaw.InerOut=pYaw.kp*(pYaw.OuterOut-GYRO_TO_DEG(gyroz));
		PwmOut[2]=1500-AngleToPwm(pYaw.InerOut)-AngleToPwm(pPitch.InerOut);
		PwmOut[3]=1500-AngleToPwm(pYaw.InerOut)+AngleToPwm(pPitch.InerOut);
		VAL_LIMIT(PwmOut[2],1000,2000);
		VAL_LIMIT(PwmOut[3],1000,2000);
		break;
	case YUYING:
		if(PwmInTemp[6]>LOW_THRESHOLD)//旋翼
		{
			PidInerPitch.err=PidOuterPitch.out+GYRO_TO_DEG(gyroy);
			pYaw.InerOut=pYaw.kp*(pYaw.OuterOut-GYRO_TO_DEG(gyroz));
			PID_Controller(&PidInerPitch);
			PwmOut[2]=AngleToPwmAdd(pYaw.InerOut)-AngleToPwm(PidInerPitch.out);
			PwmOut[3]=AngleToPwmAdd(pYaw.InerOut)+AngleToPwm(PidInerPitch.out);
			PwmOut[2]=(PwmOut[2]-1500)/ReduceServoIndex1+LeftPos+(PwmInTemp[5]-1000)*LeftServoIndex;
			PwmOut[3]=(PwmOut[3]-1500)/ReduceServoIndex1+RightPos+(PwmInTemp[5]-1000)*RightServoIndex;
			PwmOut[4]=1500;
			PwmOut[5]=1500;
		}
		else//固定翼
		{
			PwmOut[2]=(1500-PwmInTemp[0])/ReduceServoIndex2+LeftPos+(PwmInTemp[5]-1000)*LeftServoIndex;
			PwmOut[3]=(1500-PwmInTemp[0])/ReduceServoIndex2+RightPos+(PwmInTemp[5]-1000)*RightServoIndex;
			PwmOut[4]=1500;//3000-PwmInTemp[0];
			PwmOut[5]=PwmInTemp[1];
		}
		VAL_LIMIT(PwmOut[2],LOW_SERVO,HIGH_SERVO);
		VAL_LIMIT(PwmOut[3],LOW_SERVO,HIGH_SERVO);
		VAL_LIMIT(PwmOut[4],1000,2000);
		VAL_LIMIT(PwmOut[5],1000,2000);
		TIM3->CCR3=PwmOut[2];
		TIM3->CCR4=PwmOut[3];
		TIM4->CCR1=PwmOut[4];
		TIM4->CCR2=PwmOut[5];
		break;
	case ZHIFEIJI:
		if(PwmInTemp[6]<LOW_THRESHOLD)//手动
		{
			//普通行程
			PwmOut[0]=(3000-PwmInTemp[0]+PwmInTemp[1])>>1;
			PwmOut[1]=(6000-PwmInTemp[0]-PwmInTemp[1])>>1;
		}
		else//半自动
		{
			pRoll.InerOut=pRoll.kp*(pRoll.OuterOut-GYRO_TO_DEG(gyrox));
			pPitch.InerOut=pPitch.kp*(pPitch.OuterOut-GYRO_TO_DEG(gyroy));
			PwmOut[0]=1500-AngleToPwm(pRoll.InerOut)-AngleToPwm(pPitch.InerOut);
			PwmOut[1]=1500-AngleToPwm(pRoll.InerOut)+AngleToPwm(pPitch.InerOut);
		}
		VAL_LIMIT(PwmOut[0],LOW_SERVO,HIGH_SERVO);
		VAL_LIMIT(PwmOut[1],LOW_SERVO,HIGH_SERVO);
		TIM3->CCR1=PwmOut[0];
		TIM3->CCR2=PwmOut[1];
		break;
	case LAOZUZONG:
		if(PwmInTemp[6]<LOW_THRESHOLD)//手动
		{
			PwmOut[0]=3000-PwmInTemp[0];
			PwmOut[1]=PwmInTemp[1];
		}
		else//半自动
		{
			pRoll.InerOut=pRoll.kp*(pRoll.OuterOut-GYRO_TO_DEG(gyrox));
			pPitch.InerOut=pPitch.kp*(pPitch.OuterOut-GYRO_TO_DEG(gyroy));
			PwmOut[0]=AngleToPwmAdd(pRoll.InerOut);
			PwmOut[1]=AngleToPwmAdd(pPitch.InerOut);
		}
		PwmOut[3]=PwmInTemp[3];
		VAL_LIMIT(PwmOut[0],LOW_SERVO,HIGH_SERVO);
		VAL_LIMIT(PwmOut[1],LOW_SERVO,HIGH_SERVO);
		VAL_LIMIT(PwmOut[3],LOW_SERVO,HIGH_SERVO);
		TIM3->CCR1=PwmOut[0];
		TIM3->CCR2=PwmOut[1];
		TIM3->CCR4=PwmOut[3];
		break;
	case DUIDI:
		if(PwmInTemp[6]<LOW_THRESHOLD)//手动
		{
			PwmOut[0]=3000-PwmInTemp[0];
			PwmOut[1]=3000-PwmInTemp[1];
			PwmOut[3]=3000-PwmInTemp[3];
			PwmOut[4]=1500;
			PwmOut[5]=1500;
		}
		else//半自动
		{
			pRoll.InerOut=pRoll.kp*(pRoll.OuterOut-GYRO_TO_DEG(gyrox));
			pPitch.InerOut=pPitch.kp*(pPitch.OuterOut-GYRO_TO_DEG(gyroy));
			PwmOut[0]=AngleToPwmAdd(-pRoll.InerOut);
			PwmOut[1]=AngleToPwmAdd(pPitch.InerOut);
			PwmOut[3]=3000-PwmInTemp[3];
			PwmOut[4]=1500;
			PwmOut[5]=1500;
		}
		VAL_LIMIT(PwmOut[0],LOW_SERVO,HIGH_SERVO);
		VAL_LIMIT(PwmOut[1],LOW_SERVO,HIGH_SERVO);
		VAL_LIMIT(PwmOut[3],LOW_SERVO,HIGH_SERVO);
		VAL_LIMIT(PwmOut[4],LOW_SERVO,HIGH_SERVO);
		VAL_LIMIT(PwmOut[5],LOW_SERVO,HIGH_SERVO);
		TIM3->CCR1=PwmOut[0];
		TIM3->CCR2=PwmOut[1];
		TIM3->CCR4=PwmOut[3];
		TIM4->CCR1=PwmOut[4];
		TIM4->CCR2=PwmOut[5];
		break;
	default:break;
	}
}
/***********************
*@function:副翼外环控制
*@period:200ms
**********************/
void Outer_Aileron(void)
{
	switch(AircraftMode)
	{
	case SIZHOU:break;
	case CHUIQI:
		pPitch.OuterOut=pPitch.OuterP*(PwmToAngleAdd(PwmInTemp[1])-pitch);
		pYaw.OuterOut=pYaw.OuterP*PwmToAngleAdd(PwmInTemp[3]);
		break;
	case YUYING:
		PidOuterPitch.err=PwmToAngleAdd(PwmInTemp[1])+pitch;
		PID_Controller(&PidOuterPitch);
		pYaw.OuterOut=pYaw.OuterP*(PwmToAngleAdd(PwmInTemp[3]));
		break;
	case ZHIFEIJI:
		pRoll.OuterOut=pRoll.OuterP*(PwmToAngleAdd(PwmInTemp[0]+AutoRollBias)-roll);
		pPitch.OuterOut=pPitch.OuterP*(-PwmToAngleAdd(PwmInTemp[1]+AutoPitchBias)-pitch);
		break;
	case LAOZUZONG:
		pRoll.OuterOut=pRoll.OuterP*(PwmToAngleAdd(PwmInTemp[0]+AutoRollBias)-roll);
		pPitch.OuterOut=pPitch.OuterP*(-PwmToAngleAdd(PwmInTemp[1]+AutoPitchBias)-pitch);
		break;
	case DUIDI:
		pRoll.OuterOut=pRoll.OuterP*(PwmToAngleAdd(PwmInTemp[0]+AutoRollBias)-roll);
		pPitch.OuterOut=pPitch.OuterP*(-PwmToAngleAdd(PwmInTemp[1]+AutoPitchBias)-pitch);
		break;
	default:break;
	}
}
