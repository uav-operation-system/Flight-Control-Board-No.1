#include "task.h"
/**************文件说明**********************
除了飞行器控制函数之外的定时函数，分别为：
Para_Init();                   各种参数初始化
Lock_And_Unlock();             锁定，解锁
IMU_Processing();              姿态解算更新，MPU6050数据校准
Send_Data_To_Computer_20ms();  给地面站发送数据，更新全局计数
Send_Data_To_Computer_50ms();  给地面站发送数据
Send_Data_To_Computer_200ms(); 给地面站发送数据
PID_Set_Parameter();           PID等参数设置
Self_Test();                   传感器校准
与control.c共享头文件task.h。
********************************************/
u8 AircraftMode;  //飞行器类型
u8 WholeCommand=0;  //飞控状态标志位
u8 Undead=0;  //固定翼机型时该值为1，始终保持解锁状态
short accx,accy,accz;  //加速度传感器原始数据
short gyrox,gyroy,gyroz;  //陀螺仪原始数据
float roll=0,pitch=0,yaw=0;  //姿态角
u8 Armed=0;  //默认锁定
short PwmInTemp[7]={1000,1000,1000,1000,1000,1000,1000};  //防止在运算过程中被中断所更改（临界代码保护）
short RollBias,PitchBias,YawBias;  //手动模式初始误差补偿，用于固定翼的手动模式和所有旋翼，可代替遥控器的微调
short AutoRollBias,AutoPitchBias,AutoYawBias;  //自动模式初始误差补偿，用于固定翼的半自动和全自动模式
PIDparameter PidOuterRoll,PidOuterPitch;  //两个方向外环姿态PID
PIDparameter PidInerRoll,PidInerPitch;  //两个方向内环角速度PID
Pparameter pRoll,pPitch,pYaw;  //三个方向舵机双P控制
//以下变量均为鱼鹰专用
short LeftPos,RightPos;  //旋翼时左右倾转舵机的中心位置
short LeftFinal,RightFinal;  //固定翼时左右倾转舵机的位置
float LeftServoIndex,RightServoIndex;
/**********************
*@function:各种参数初始化
*@note:必须在上电后至少一个PWM周期后进行
**********************/
void Para_Init(void)
{
	switch(AircraftMode)
	{
	case SIZHOU:
		PidOuterRoll.kp=1.6;PidOuterRoll.ki=0;PidOuterRoll.kd=0.2;
		PidInerRoll.kp=0.27;PidInerRoll.ki=0;PidInerRoll.kd=0;
		PidOuterPitch.kp=1.5;PidOuterPitch.ki=0;PidOuterPitch.kd=0.18;
		PidInerPitch.kp=0.28;PidInerPitch.ki=0;PidInerPitch.kd=0;
		RollBias=0;PitchBias=0;YawBias=0;
		PID_Reset(&PidOuterRoll);
		PID_Reset(&PidInerRoll);
		PID_Reset(&PidOuterPitch);
		PID_Reset(&PidInerPitch);
		PidOuterRoll.IntiMax=100;PidInerRoll.IntiMax=250;
		PidOuterPitch.IntiMax=100;PidInerPitch.IntiMax=250;
		break;
	case CHUIQI:
		PidOuterRoll.kp=1;PidOuterRoll.ki=0;PidOuterRoll.kd=0;
		PidInerRoll.kp=0.5;PidInerRoll.ki=0;PidInerRoll.kd=0.1;
		pPitch.kp=1;pYaw.kp=1;
		RollBias=0;PitchBias=0;YawBias=100;
		PID_Reset(&PidOuterRoll);
		PID_Reset(&PidInerRoll);
		P_Reset(&pPitch);
		P_Reset(&pYaw);
		PidOuterRoll.IntiMax=100;PidInerRoll.IntiMax=250;
		PidOuterPitch.IntiMax=100;PidInerPitch.IntiMax=250;
		break;
	case YUYING:
		PidOuterRoll.kp=1;PidOuterRoll.ki=0;PidOuterRoll.kd=0;
		PidInerRoll.kp=0.5;PidInerRoll.ki=0;PidInerRoll.kd=0.1;
		PidOuterPitch.kp=1;PidOuterPitch.ki=0;PidOuterPitch.kd=0;
		PidInerPitch.kp=1;PidInerPitch.ki=0;PidInerPitch.kd=0;
		pYaw.kp=0.4;
		RollBias=0;PitchBias=0;YawBias=0;
		PID_Reset(&PidOuterRoll);
		PID_Reset(&PidOuterPitch);
		PID_Reset(&PidInerRoll);
		PID_Reset(&PidInerPitch);
		P_Reset(&pYaw);
		PidOuterRoll.IntiMax=100;PidInerRoll.IntiMax=250;
		PidOuterPitch.IntiMax=100;PidInerPitch.IntiMax=250;
		LeftPos=1900;RightPos=1000;
		LeftFinal=1100;RightFinal=1900;
		LeftServoIndex=(LeftFinal-LeftPos)/1000.0;
		RightServoIndex=(RightFinal-RightPos)/1000.0;
		break;
	case ZHIFEIJI:
		pRoll.kp=0.4;pPitch.kp=0.4;
		RollBias=0;PitchBias=0;YawBias=0;
		AutoRollBias=0;AutoPitchBias=0;AutoYawBias=0;
		P_Reset(&pRoll);
		P_Reset(&pPitch);
		break;
	case LAOZUZONG:
		pRoll.kp=0.25;pPitch.kp=0.25;
		RollBias=0;PitchBias=0;YawBias=0;
		AutoRollBias=0;AutoPitchBias=0;AutoYawBias=0;
		P_Reset(&pRoll);
		P_Reset(&pPitch);
		RollBias=0;PitchBias=0;YawBias=0;
		AutoRollBias=0;AutoPitchBias=0;AutoYawBias=0;
		break;
	case DUIDI:
		pRoll.kp=0.25;pPitch.kp=0.25;
		RollBias=0;PitchBias=0;YawBias=0;
		AutoRollBias=0;AutoPitchBias=0;AutoYawBias=0;
		P_Reset(&pRoll);
		P_Reset(&pPitch);
		RollBias=0;PitchBias=0;YawBias=0;
		AutoRollBias=0;AutoPitchBias=0;AutoYawBias=0;
		break;
	default:break;
	}
}
/***********************
*@function:锁定，解锁
*@period:100ms
*@note:左操纵杆推到右下方解锁;左操纵杆推到左下方锁定
**********************/
void Lock_And_Unlock(void)
{
	static u8 LockMode=LOCKED;//锁定/解锁过程状态机
	static u16 t=0;//解锁过程需要的时间
	if(!(WholeCommand&NORMAL_WORK)){LockMode=LOCKED;Armed=0;return;}//上电自检未完成不能解锁
	if(Undead){LockMode=UNLOCKED;Armed=1;return;}//固定翼的免死令牌
	switch(LockMode)
	{
		case LOCKED:
			if((PwmInTemp[0]>LOCK_HIGH)&&(PwmInTemp[1]<LOCK_LOW)&&(PwmInTemp[2]<LOCK_LOW)&&(PwmInTemp[3]<LOCK_LOW))//自检完成并且有解锁操作
			{LockMode=TOUNLOCK;t++;}
			break;
		case TOUNLOCK:
			if(t>TIME)//达到解锁时间后解锁
			{LockMode=UNLOCKED;t=0;Armed=1;}
			else if((PwmInTemp[0]>LOCK_HIGH)&&(PwmInTemp[1]<LOCK_LOW)&&(PwmInTemp[2]<LOCK_LOW)&&(PwmInTemp[3]<LOCK_LOW))//依然在尝试解锁
				t++;
			else//放弃解锁
			{LockMode=LOCKED;t=0;}
			break;
		case UNLOCKED:
			if((PwmInTemp[2]<LOCK_LOW)&&(PwmInTemp[3]>LOCK_HIGH))//有锁定操作
			{LockMode=LOCKED;Armed=0;}//与解锁操作不同，有锁定操作立即锁定
			break;
		default:break;
	}
}
/***********************
*@function:姿态解算更新，MPU6050数据校准
*@period:20ms
**********************/
void IMU_Processing(void)
{
	MPU_Get_Accelerometer(&accx,&accy,&accz);//获取加速度计原始数据
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);//获取陀螺仪原始数据
	Acc_Calibrate(&accx,&accy,&accz);
	IMUupdate(&gyrox,&gyroy,&gyroz,accx,accy,accz,&roll,&pitch,&yaw);//姿态更新
	if(ANO_CMD & ACC_CALI)//收到校准命令。上电初始化默认收到校准命令，之后也可以多次校准
	{
		if(Acc_Get_ErrData(accx,accy,accz,1))//校准完成
		{
			ANO_CMD&=~ACC_CALI;//清除校准命令
			Acc_Get_ErrData(accx,accy,accz,0);//校准复位，等待可能的下一次校准
		}
	}
	if(ANO_CMD & GYRO_CALI)//同上
	{
		if(Gyro_Get_ErrData(gyrox,gyroy,gyroz,1))
		{
			ANO_CMD&=~GYRO_CALI;
			Gyro_Get_ErrData(gyrox,gyroy,gyroz,0);
		}
	}
}
/***********************
*@function:给地面站发送数据，更新全局计数
*@period:20ms
**********************/
void Send_Data_To_Computer_20ms(void)
{
	s16 mydata1[8];//向地面站发送自定义数据
	PwmInTemp[0]=PwmIn[0]-20+RollBias;
	PwmInTemp[1]=PwmIn[1]-20+PitchBias;
	PwmInTemp[2]=PwmIn[2]-20;
	PwmInTemp[3]=PwmIn[3]-20+YawBias;
	PwmInTemp[4]=PwmIn[4]-20;
	PwmInTemp[5]=PwmIn[5]-20;
	PwmInTemp[6]=PwmIn[6]-20;//以上程序为对接收机的数据进行初步处理
	mydata1[0]=GYRO_TO_DEG(gyrox);
	mydata1[1]=PidInerRoll.out*100;
	mydata1[2]=PidOuterRoll.out*100;
	mydata1[3]=TIM3->CCR1;
	mydata1[4]=TIM3->CCR2;
	mydata1[5]=PidInerRoll.err*100;
	mydata1[6]=PidOuterRoll.err*100;
	mydata1[7]=PidOuterRoll.integral*100;
//	if(PwmInTemp[4]>HIGH_THRESHOLD)
	{
		ANO_Send_User_Data(mydata1,8,0xF1);
	}
	ANO_DT_Send_Senser(accx,accy,accz,gyrox,gyroy,gyroz,0,0,0);
	ANO_DT_Send_Status(roll,pitch,yaw,bmp180.altitude,0,Armed);
	//ANO开头的发送函数只是往DMA缓冲中填入数据而并不发送
	MYDMA_Enable();//DMA开始发送
}
/***********************
*@function:给地面站发送数据
*@period:50ms
**********************/
void Send_Data_To_Computer_50ms(void)
{
}
/***********************
*@function:给地面站发送数据
*@period:200ms
**********************/
void Send_Data_To_Computer_200ms(void)
{
	ANO_DT_Send_RCData(PwmInTemp[2],PwmInTemp[3],PwmInTemp[0],PwmInTemp[1],PwmIn[4],PwmIn[5],PwmIn[6],1000,1000,1000);
}
/***********************
*@function:PID等参数设置
*@period:100ms
**********************/
void PID_Set_Parameter(void)
{
	if(ANO_CMD&PID_REQUIRE)//地面站要求读取PID
	{
		ANO_DT_Send_PID(1,PidOuterRoll.kp,PidOuterRoll.ki,PidOuterRoll.kd,PidOuterPitch.kp,PidOuterPitch.ki,PidOuterPitch.kd,(float)LeftPos/1000.0,(float)LeftFinal/1000.0,0);
		ANO_DT_Send_PID(2,PidInerRoll.kp,PidInerRoll.ki,PidInerRoll.kd,PidInerPitch.kp,PidInerPitch.ki,PidInerPitch.kd,(float)RightPos/1000.0,(float)RightFinal/1000.0,0);
		ANO_CMD&=~PID_REQUIRE;
	}
	if(ANO_CMD&PID_SENDBACK)//地面站要求写入PID
	{
		PidOuterRoll.kp=(PIDReceiveTemp[0][0]*256.0+PIDReceiveTemp[0][1])/1000.0;
		PidOuterRoll.ki=(PIDReceiveTemp[0][2]*256.0+PIDReceiveTemp[0][3])/1000.0;
		PidOuterRoll.kd=(PIDReceiveTemp[0][4]*256.0+PIDReceiveTemp[0][5])/1000.0;
		PidOuterPitch.kp=(PIDReceiveTemp[0][6]*256.0+PIDReceiveTemp[0][7])/1000.0;
		PidOuterPitch.ki=(PIDReceiveTemp[0][8]*256.0+PIDReceiveTemp[0][9])/1000.0;
		PidOuterPitch.kd=(PIDReceiveTemp[0][10]*256.0+PIDReceiveTemp[0][11])/1000.0;
		LeftPos=(PIDReceiveTemp[0][12]*256.0+PIDReceiveTemp[0][13]);
		LeftFinal=(PIDReceiveTemp[0][14]*256.0+PIDReceiveTemp[0][15]);
		
		PidInerRoll.kp=(PIDReceiveTemp[1][0]*256.0+PIDReceiveTemp[1][1])/1000.0;
		PidInerRoll.ki=(PIDReceiveTemp[1][2]*256.0+PIDReceiveTemp[1][3])/1000.0;
		PidInerRoll.kd=(PIDReceiveTemp[1][4]*256.0+PIDReceiveTemp[1][5])/1000.0;
		PidInerPitch.kp=(PIDReceiveTemp[1][6]*256.0+PIDReceiveTemp[1][7])/1000.0;
		PidInerPitch.ki=(PIDReceiveTemp[1][8]*256.0+PIDReceiveTemp[1][9])/1000.0;
		PidInerPitch.kd=(PIDReceiveTemp[1][10]*256.0+PIDReceiveTemp[1][11])/1000.0;
		RightPos=(PIDReceiveTemp[1][12]*256.0+PIDReceiveTemp[1][13]);
		RightFinal=(PIDReceiveTemp[1][14]*256.0+PIDReceiveTemp[1][15]);
		
		ANO_CMD&=~PID_SENDBACK;
	}
}
/***********************
*@function:上电0.5秒后等待水平静止后开始耗时1秒的传感器校准
*@period:500ms
**********************/
void Self_Test(void)
{
	if(WholeCommand & NORMAL_WORK)//如果自检完毕不执行该函数
		return;
	//上电后以当前4和6通道状态决定机型，上电后只执行一次，
	//之后这两个通道就可以自由使用了
	if(!(WholeCommand & AIRCRAFT_MODE))
	{
		AircraftMode=THREE_SWITCH(PwmInTemp[4])*3-3+THREE_SWITCH(PwmInTemp[6]);
		if(AircraftMode>6)
			AircraftMode=0;
		else if(AircraftMode>3)
			Undead=1;////固定翼的免死令牌
		WholeCommand |= AIRCRAFT_MODE;
		Para_Init();//机型确定后初始化参数
	}
	//以下4个if语句顺序虽然有点乱但不能更改
	//语句1。为了区分未校准与校准完成的情况而加入prepare标志位，该标志位置1则清零并置位calibrating标志位，开始校准。
	//       prepare标志位还有一个作用是满足水平静止条件后再等待一个时钟周期，等完全放好后再开始校准
	if(WholeCommand & PREPARE)
	{
		ANO_CMD |= 0x03;
		WholeCommand |= CALIBRATING;
		WholeCommand &=~ PREPARE;
	}
	//语句2。校准完毕情况下执行
	if( (WholeCommand & CALIBRATING) && !(ANO_CMD & 0x03) )
	{
		WholeCommand &=~ CALIBRATING;
		WholeCommand |= CALIBRATED;
	}
	//语句3。只要没有校准完成则全程检测是否保持水平静止
	if( !(WholeCommand & CALIBRATED) )
	{
		if(ABS(roll)>10 || ABS(pitch)>10 || ABS(gyrox)>1000 || ABS(gyroy)>1000 || ABS(gyroz)>1000 ||
		ABS(accx)>2000 || ABS(accy)>2000 || accz>20000 || accz<10000)//如果没有保持水平静止就不进行校准
		{
			if(WholeCommand & CALIBRATING)//如果校准过程中又移动了要退出并等待重新校准
			{
				WholeCommand &=~ CALIBRATING;
				ANO_CMD = 0;
			}
		}
		else
		{
			if( !(WholeCommand & CALIBRATING) )//如果满足条件但还未开始校准，则产生prepare高电平脉冲准备开始校准
				WholeCommand |= PREPARE;
		}
	}
	//语句4。机型已确定并校准完毕后进入正常工作状态
	if(WholeCommand == 0x12)
	{
		WholeCommand |= NORMAL_WORK;
		LED_ONOFF(Bit_SET,LED0_GPIO_PIN|LED1_GPIO_PIN|LED2_GPIO_PIN);
		LED_SELECT(AircraftMode);
	}
	//以下对各种情况进行说明：(中括号[]表示重复)
	//情况一：先放好再上电，全程保持水平静止。(02)->语句3(06)->语句1(0A)->[语句3(0A)]->语句2(12)->语句4(13)
	//情况二：先上电再放好，放好后全程保持水平静止。(02)->[语句3(02)]->语句3(06)->语句1(0A)->[语句3(0A)]->语句2(12)->语句4(13)
	//情况三：放好后移动(放好前、移动并再次放好后均同情况一二)。(02)...->语句1(0A)->[语句3(0A)]->[语句3(02)]->语句3(06)->...->语句4(13)
}
