#include "task.h"
/**************�ļ�˵��**********************
���˷��������ƺ���֮��Ķ�ʱ�������ֱ�Ϊ��
Para_Init();                   ���ֲ�����ʼ��
Lock_And_Unlock();             ����������
IMU_Processing();              ��̬������£�MPU6050����У׼
Send_Data_To_Computer_20ms();  ������վ�������ݣ�����ȫ�ּ���
Send_Data_To_Computer_50ms();  ������վ��������
Send_Data_To_Computer_200ms(); ������վ��������
PID_Set_Parameter();           PID�Ȳ�������
Self_Test();                   ������У׼
��control.c����ͷ�ļ�task.h��
********************************************/
u8 AircraftMode;  //����������
u8 WholeCommand=0;  //�ɿ�״̬��־λ
u8 Undead=0;  //�̶������ʱ��ֵΪ1��ʼ�ձ��ֽ���״̬
short accx,accy,accz;  //���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;  //������ԭʼ����
float roll=0,pitch=0,yaw=0;  //��̬��
u8 Armed=0;  //Ĭ������
short PwmInTemp[7]={1000,1000,1000,1000,1000,1000,1000};  //��ֹ����������б��ж������ģ��ٽ���뱣����
short RollBias,PitchBias,YawBias;  //�ֶ�ģʽ��ʼ���������ڹ̶�����ֶ�ģʽ�����������ɴ���ң������΢��
short AutoRollBias,AutoPitchBias,AutoYawBias;  //�Զ�ģʽ��ʼ���������ڹ̶���İ��Զ���ȫ�Զ�ģʽ
PIDparameter PidOuterRoll,PidOuterPitch;  //���������⻷��̬PID
PIDparameter PidInerRoll,PidInerPitch;  //���������ڻ����ٶ�PID
Pparameter pRoll,pPitch,pYaw;  //����������˫P����
//���±�����Ϊ��ӥר��
short LeftPos,RightPos;  //����ʱ������ת���������λ��
short LeftFinal,RightFinal;  //�̶���ʱ������ת�����λ��
float LeftServoIndex,RightServoIndex;
/**********************
*@function:���ֲ�����ʼ��
*@note:�������ϵ������һ��PWM���ں����
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
*@function:����������
*@period:100ms
*@note:����ݸ��Ƶ����·�����;����ݸ��Ƶ����·�����
**********************/
void Lock_And_Unlock(void)
{
	static u8 LockMode=LOCKED;//����/��������״̬��
	static u16 t=0;//����������Ҫ��ʱ��
	if(!(WholeCommand&NORMAL_WORK)){LockMode=LOCKED;Armed=0;return;}//�ϵ��Լ�δ��ɲ��ܽ���
	if(Undead){LockMode=UNLOCKED;Armed=1;return;}//�̶������������
	switch(LockMode)
	{
		case LOCKED:
			if((PwmInTemp[0]>LOCK_HIGH)&&(PwmInTemp[1]<LOCK_LOW)&&(PwmInTemp[2]<LOCK_LOW)&&(PwmInTemp[3]<LOCK_LOW))//�Լ���ɲ����н�������
			{LockMode=TOUNLOCK;t++;}
			break;
		case TOUNLOCK:
			if(t>TIME)//�ﵽ����ʱ������
			{LockMode=UNLOCKED;t=0;Armed=1;}
			else if((PwmInTemp[0]>LOCK_HIGH)&&(PwmInTemp[1]<LOCK_LOW)&&(PwmInTemp[2]<LOCK_LOW)&&(PwmInTemp[3]<LOCK_LOW))//��Ȼ�ڳ��Խ���
				t++;
			else//��������
			{LockMode=LOCKED;t=0;}
			break;
		case UNLOCKED:
			if((PwmInTemp[2]<LOCK_LOW)&&(PwmInTemp[3]>LOCK_HIGH))//����������
			{LockMode=LOCKED;Armed=0;}//�����������ͬ��������������������
			break;
		default:break;
	}
}
/***********************
*@function:��̬������£�MPU6050����У׼
*@period:20ms
**********************/
void IMU_Processing(void)
{
	MPU_Get_Accelerometer(&accx,&accy,&accz);//��ȡ���ٶȼ�ԭʼ����
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);//��ȡ������ԭʼ����
	Acc_Calibrate(&accx,&accy,&accz);
	IMUupdate(&gyrox,&gyroy,&gyroz,accx,accy,accz,&roll,&pitch,&yaw);//��̬����
	if(ANO_CMD & ACC_CALI)//�յ�У׼����ϵ��ʼ��Ĭ���յ�У׼���֮��Ҳ���Զ��У׼
	{
		if(Acc_Get_ErrData(accx,accy,accz,1))//У׼���
		{
			ANO_CMD&=~ACC_CALI;//���У׼����
			Acc_Get_ErrData(accx,accy,accz,0);//У׼��λ���ȴ����ܵ���һ��У׼
		}
	}
	if(ANO_CMD & GYRO_CALI)//ͬ��
	{
		if(Gyro_Get_ErrData(gyrox,gyroy,gyroz,1))
		{
			ANO_CMD&=~GYRO_CALI;
			Gyro_Get_ErrData(gyrox,gyroy,gyroz,0);
		}
	}
}
/***********************
*@function:������վ�������ݣ�����ȫ�ּ���
*@period:20ms
**********************/
void Send_Data_To_Computer_20ms(void)
{
	s16 mydata1[8];//�����վ�����Զ�������
	PwmInTemp[0]=PwmIn[0]-20+RollBias;
	PwmInTemp[1]=PwmIn[1]-20+PitchBias;
	PwmInTemp[2]=PwmIn[2]-20;
	PwmInTemp[3]=PwmIn[3]-20+YawBias;
	PwmInTemp[4]=PwmIn[4]-20;
	PwmInTemp[5]=PwmIn[5]-20;
	PwmInTemp[6]=PwmIn[6]-20;//���ϳ���Ϊ�Խ��ջ������ݽ��г�������
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
	//ANO��ͷ�ķ��ͺ���ֻ����DMA�������������ݶ���������
	MYDMA_Enable();//DMA��ʼ����
}
/***********************
*@function:������վ��������
*@period:50ms
**********************/
void Send_Data_To_Computer_50ms(void)
{
}
/***********************
*@function:������վ��������
*@period:200ms
**********************/
void Send_Data_To_Computer_200ms(void)
{
	ANO_DT_Send_RCData(PwmInTemp[2],PwmInTemp[3],PwmInTemp[0],PwmInTemp[1],PwmIn[4],PwmIn[5],PwmIn[6],1000,1000,1000);
}
/***********************
*@function:PID�Ȳ�������
*@period:100ms
**********************/
void PID_Set_Parameter(void)
{
	if(ANO_CMD&PID_REQUIRE)//����վҪ���ȡPID
	{
		ANO_DT_Send_PID(1,PidOuterRoll.kp,PidOuterRoll.ki,PidOuterRoll.kd,PidOuterPitch.kp,PidOuterPitch.ki,PidOuterPitch.kd,(float)LeftPos/1000.0,(float)LeftFinal/1000.0,0);
		ANO_DT_Send_PID(2,PidInerRoll.kp,PidInerRoll.ki,PidInerRoll.kd,PidInerPitch.kp,PidInerPitch.ki,PidInerPitch.kd,(float)RightPos/1000.0,(float)RightFinal/1000.0,0);
		ANO_CMD&=~PID_REQUIRE;
	}
	if(ANO_CMD&PID_SENDBACK)//����վҪ��д��PID
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
*@function:�ϵ�0.5���ȴ�ˮƽ��ֹ��ʼ��ʱ1��Ĵ�����У׼
*@period:500ms
**********************/
void Self_Test(void)
{
	if(WholeCommand & NORMAL_WORK)//����Լ���ϲ�ִ�иú���
		return;
	//�ϵ���Ե�ǰ4��6ͨ��״̬�������ͣ��ϵ��ִֻ��һ�Σ�
	//֮��������ͨ���Ϳ�������ʹ����
	if(!(WholeCommand & AIRCRAFT_MODE))
	{
		AircraftMode=THREE_SWITCH(PwmInTemp[4])*3-3+THREE_SWITCH(PwmInTemp[6]);
		if(AircraftMode>6)
			AircraftMode=0;
		else if(AircraftMode>3)
			Undead=1;////�̶������������
		WholeCommand |= AIRCRAFT_MODE;
		Para_Init();//����ȷ�����ʼ������
	}
	//����4��if���˳����Ȼ�е��ҵ����ܸ���
	//���1��Ϊ������δУ׼��У׼��ɵ����������prepare��־λ���ñ�־λ��1�����㲢��λcalibrating��־λ����ʼУ׼��
	//       prepare��־λ����һ������������ˮƽ��ֹ�������ٵȴ�һ��ʱ�����ڣ�����ȫ�źú��ٿ�ʼУ׼
	if(WholeCommand & PREPARE)
	{
		ANO_CMD |= 0x03;
		WholeCommand |= CALIBRATING;
		WholeCommand &=~ PREPARE;
	}
	//���2��У׼��������ִ��
	if( (WholeCommand & CALIBRATING) && !(ANO_CMD & 0x03) )
	{
		WholeCommand &=~ CALIBRATING;
		WholeCommand |= CALIBRATED;
	}
	//���3��ֻҪû��У׼�����ȫ�̼���Ƿ񱣳�ˮƽ��ֹ
	if( !(WholeCommand & CALIBRATED) )
	{
		if(ABS(roll)>10 || ABS(pitch)>10 || ABS(gyrox)>1000 || ABS(gyroy)>1000 || ABS(gyroz)>1000 ||
		ABS(accx)>2000 || ABS(accy)>2000 || accz>20000 || accz<10000)//���û�б���ˮƽ��ֹ�Ͳ�����У׼
		{
			if(WholeCommand & CALIBRATING)//���У׼���������ƶ���Ҫ�˳����ȴ�����У׼
			{
				WholeCommand &=~ CALIBRATING;
				ANO_CMD = 0;
			}
		}
		else
		{
			if( !(WholeCommand & CALIBRATING) )//���������������δ��ʼУ׼�������prepare�ߵ�ƽ����׼����ʼУ׼
				WholeCommand |= PREPARE;
		}
	}
	//���4��������ȷ����У׼��Ϻ������������״̬
	if(WholeCommand == 0x12)
	{
		WholeCommand |= NORMAL_WORK;
		LED_ONOFF(Bit_SET,LED0_GPIO_PIN|LED1_GPIO_PIN|LED2_GPIO_PIN);
		LED_SELECT(AircraftMode);
	}
	//���¶Ը����������˵����(������[]��ʾ�ظ�)
	//���һ���ȷź����ϵ磬ȫ�̱���ˮƽ��ֹ��(02)->���3(06)->���1(0A)->[���3(0A)]->���2(12)->���4(13)
	//����������ϵ��ٷźã��źú�ȫ�̱���ˮƽ��ֹ��(02)->[���3(02)]->���3(06)->���1(0A)->[���3(0A)]->���2(12)->���4(13)
	//��������źú��ƶ�(�ź�ǰ���ƶ����ٴηźú��ͬ���һ��)��(02)...->���1(0A)->[���3(0A)]->[���3(02)]->���3(06)->...->���4(13)
}
