#ifndef __TASK_H
#define __TASK_H

#include "mpu6050.h"
#include "bmp180.h"
#include "imu.h"
#include "dma.h"
#include "niming.h"
#include "pwm_in.h"
#include "led.h"

#define LOWSPEED          1100	 //怠速
#define LOW_THRESHOLD     1250  //5和7通道的1,2档阈值
#define HIGH_THRESHOLD    1750  //5和7通道的2,3档阈值
#define LOCK_HIGH         1800  //锁定和解锁摇杆上阈值
#define LOCK_LOW          1200  //锁定和解锁摇杆下阈值
#define LOW_SERVO         500  //舵机行程下极限
#define HIGH_SERVO        2500  //舵机行程上极限
#define THREE_SWITCH(x)   ( (x)<(LOW_THRESHOLD)? 1 : ( (x)<(HIGH_THRESHOLD)? 2:3) )
//LockMode
#define LOCKED   0	//锁定状态且无操作
#define TOUNLOCK 1	//锁定状态且尝试解锁
#define UNLOCKED 2	//解锁状态
#define TIME     20	//解锁时间,2秒
//WholeCommand
#define NORMAL_WORK     0x01  //高电平表示自检完毕，进入正常工作状态
#define AIRCRAFT_MODE   0x02  //高电平表示机型已确定
#define PREPARE         0x04  //高电平脉冲触发校准
#define CALIBRATING     0x08  //高电平表示正在校准
#define CALIBRATED      0x10  //高电平表示校准完毕
#define BUF_OVERFLOW    0x20  //高电平表示缓冲溢出
//AircraftMode
#define SIZHOU      1  //四旋翼
#define CHUIQI      2  //垂直起降双旋翼
#define YUYING      3  //鱼鹰
#define ZHIFEIJI    4  //纸飞机
#define LAOZUZONG   5  //老祖宗
#define DUIDI       6  //带云台的对地侦查固定翼
//鱼鹰专用
#define ReduceServoIndex1  8   //减小旋翼模式下的倾转舵机行程
#define ReduceServoIndex2  10   //减小固定翼模式下的倾转舵机行程

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

//在task.c中
void Para_Init(void);
void Lock_And_Unlock(void);
void IMU_Processing(void);
void Barometer_Processing(void);
void Send_Data_To_Computer_20ms(void);
void Send_Data_To_Computer_50ms(void);
void Send_Data_To_Computer_200ms(void);
void PID_Set_Parameter(void);
void Self_Test(void);
//在control.c中
void Update_Iner_loop(void);
void Update_Outer_Loop(void);
void Aileron_Control(void);
void Outer_Aileron(void);

#endif
