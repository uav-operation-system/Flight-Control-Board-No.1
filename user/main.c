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
//��ʱ����ĸ���Ӧ��־λ
#define TASK_1ms		0x01
#define TASK_20ms		0x02
#define TASK_50ms		0x04
#define TASK_100ms		0x08
#define TASK_200ms		0x10
#define TASK_500ms		0x20

u8 Time1ms=0;	//ÿ1ms��1
u8 Time100ms=0;	//ÿ100ms��1
u8 TaskFlag=0;	//��ʱ����ı�־λ�Ĵ���
char rxdatabufer;
u16 point2 = 0;
int main(void)
{
/*��ʼ��*/
	LED_GPIO_Config();
	LED_ONOFF(Bit_RESET,LED0_GPIO_PIN|LED1_GPIO_PIN|LED2_GPIO_PIN);	//�ϵ�ʱָʾ����
	PWM_OUT_Configuration();	//�ϵ�ʱ����������Ϊ��͵㣬�������
	IWDG_Init();	//����ʼ��ʧ�ܿɸ�λ���³�ʼ��
	uart1_init(115200);	//�����Ĳ�����Ҫ�������ã����Ծ�����Ҫ�Ĳ�����
	uart3_init(38400);	
	MPU_Init();
	BMP_Init();
	PWM_IN_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	MYDMA_Config();
	SysTick_Init();
/*��ѭ��*/
 	while(1)
	{
		if(TaskFlag & TASK_1ms)
		{
			IWDG_ReloadCounter();	//ι��������װ�ؼ�����
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

/*1ms�ж�һ��
*����ʹÿ������Ŀ�ʼʱ��Ϊ��������ϵ��
�������Ա���ͬһʱ��ִ�ж�����񣬱�֤��ʱ׼ȷ
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
