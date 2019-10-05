#ifndef __TASK_H
#define __TASK_H

#include "mpu6050.h"
#include "bmp180.h"
#include "imu.h"
#include "dma.h"
#include "niming.h"
#include "pwm_in.h"
#include "led.h"

#define LOWSPEED          1100	 //����
#define LOW_THRESHOLD     1250  //5��7ͨ����1,2����ֵ
#define HIGH_THRESHOLD    1750  //5��7ͨ����2,3����ֵ
#define LOCK_HIGH         1800  //�����ͽ���ҡ������ֵ
#define LOCK_LOW          1200  //�����ͽ���ҡ������ֵ
#define LOW_SERVO         500  //����г��¼���
#define HIGH_SERVO        2500  //����г��ϼ���
#define THREE_SWITCH(x)   ( (x)<(LOW_THRESHOLD)? 1 : ( (x)<(HIGH_THRESHOLD)? 2:3) )
//LockMode
#define LOCKED   0	//����״̬���޲���
#define TOUNLOCK 1	//����״̬�ҳ��Խ���
#define UNLOCKED 2	//����״̬
#define TIME     20	//����ʱ��,2��
//WholeCommand
#define NORMAL_WORK     0x01  //�ߵ�ƽ��ʾ�Լ���ϣ�������������״̬
#define AIRCRAFT_MODE   0x02  //�ߵ�ƽ��ʾ������ȷ��
#define PREPARE         0x04  //�ߵ�ƽ���崥��У׼
#define CALIBRATING     0x08  //�ߵ�ƽ��ʾ����У׼
#define CALIBRATED      0x10  //�ߵ�ƽ��ʾУ׼���
#define BUF_OVERFLOW    0x20  //�ߵ�ƽ��ʾ�������
//AircraftMode
#define SIZHOU      1  //������
#define CHUIQI      2  //��ֱ��˫����
#define YUYING      3  //��ӥ
#define ZHIFEIJI    4  //ֽ�ɻ�
#define LAOZUZONG   5  //������
#define DUIDI       6  //����̨�ĶԵ����̶���
//��ӥר��
#define ReduceServoIndex1  8   //��С����ģʽ�µ���ת����г�
#define ReduceServoIndex2  10   //��С�̶���ģʽ�µ���ת����г�

extern u8 AircraftMode;
extern u8 Undead;
extern short gyrox,gyroy,gyroz;
extern float roll,pitch,yaw;
extern u8 Armed;
extern short PwmInTemp[7];
extern short RollBias,PitchBias,YawBias;
extern short AutoRollBias,AutoPitchBias,AutoYawBias;
extern PIDparameter PidOuterRoll,PidOuterPitch;
extern PIDparameter PidInerRoll,PidInerPitch;
extern Pparameter pRoll,pPitch,pYaw;
extern short LeftPos,RightPos;
extern float LeftServoIndex,RightServoIndex;

//��task.c��
void Para_Init(void);
void Lock_And_Unlock(void);
void IMU_Processing(void);
void Barometer_Processing(void);
void Send_Data_To_Computer_20ms(void);
void Send_Data_To_Computer_50ms(void);
void Send_Data_To_Computer_200ms(void);
void PID_Set_Parameter(void);
void Self_Test(void);
//��control.c��
void Update_Iner_loop(void);
void Update_Outer_Loop(void);
void Aileron_Control(void);
void Outer_Aileron(void);

#endif
