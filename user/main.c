#include "usart.h"
#include "mpu6050.h"
#include "bmp180.h"
#include "SysTick.h"
#include "dma.h"
#include "led.h"
#include "pwm_in.h"
#include "pwm_out.h"
#include "iwdg.h"
#include "task.h"
#include "bd.h"
//#include "delay.h"
#include "niming.h"
#include <string.h>
//#include "sys.h"
//定时任务的各相应标志位
#define TASK_1ms		0x01
#define TASK_20ms		0x02
#define TASK_50ms		0x04
#define TASK_100ms		0x08
#define TASK_200ms		0x10
#define TASK_500ms		0x20

u8 Time1ms=0;	//每1ms增1
u8 Time100ms=0;	//每100ms增1
u8 TaskFlag=0;	//定时任务的标志位寄存器
char rxdatabufer;
u16 point2 = 0;
int main(void)
{
/*初始化*/
	LED_GPIO_Config();
	LED_ONOFF(Bit_RESET,LED0_GPIO_PIN|LED1_GPIO_PIN|LED2_GPIO_PIN);	//上电时指示灯亮
	PWM_OUT_Configuration();	//上电时先设置油门为最低点，舵机归中
	IWDG_Init();	//若初始化失败可复位重新初始化
	uart1_init(115200);	//数传的波特率要单独设置，所以尽量不要改波特率
	uart3_init(38400);	
	MPU_Init();
	BMP_Init();
	PWM_IN_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	MYDMA_Config();
	SysTick_Init();
/*死循环*/
 	while(1)
	{
		if(TaskFlag & TASK_1ms)
		{
			IWDG_ReloadCounter();	//喂狗：重新装载计数器
			TaskFlag&=~TASK_1ms;
		}
		if(TaskFlag & TASK_20ms)
		{
			IMU_Processing();
			Update_Iner_loop();
			Send_Data_To_Computer_20ms();
			TaskFlag&=~TASK_20ms;
		}
		if(TaskFlag & TASK_50ms)
		{
			Update_Outer_Loop();
			Aileron_Control();
			Send_Data_To_Computer_50ms();
			TaskFlag&=~TASK_50ms;
		}
		if(TaskFlag & TASK_100ms)
		{
			Lock_And_Unlock();
			PID_Set_Parameter();
			bmp180Convert();
			TaskFlag&=~TASK_100ms;
		}
		if(TaskFlag & TASK_200ms)
		{
			Outer_Aileron();
			Send_Data_To_Computer_200ms();
			 //printGpsBuffer();
			 BEIDOU_Update();
			TaskFlag&=~TASK_200ms;
			
		}
		if(TaskFlag & TASK_500ms)
		{
			Self_Test();
			TaskFlag&=~TASK_500ms;
		}
	}
}

/*1ms中断一次
*避免使每个任务的开始时刻为整数倍关系，
这样可以避免同一时刻执行多个任务，保证计时准确
*/
void SysTick_Handler(void)
{
	TaskFlag|=TASK_1ms;
	switch(Time1ms)
	{
		case 5:
		case 25:
		case 45:
		case 65:
		case 85:TaskFlag|=TASK_20ms;break;
		case 10:
		case 60:TaskFlag|=TASK_50ms;break;
		case 100:
			TaskFlag|=TASK_100ms;
			Time100ms++;
			Time1ms=0;
			break;
		default:break;
	}
	Time1ms++;
	if(Time1ms>100)
		Time1ms=0;
	if(Time100ms & 0x01)
		TaskFlag|=TASK_200ms;
	if(Time100ms>=5)
	{
		TaskFlag|=TASK_500ms;
		Time100ms=0;		
	}
}
