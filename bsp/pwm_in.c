#include "pwm_in.h"

short PwmIn[8];
u8 TIM2Capture_STA=0;		//����λ��־����״̬
u8 TIM8Capture_STA=0;		//����λ��־����״̬
u16 Rise[8]={0};
u16 Drop[8]={0};
u16 t2sr;		//��ʱ��2״̬�Ĵ���
u16 t8sr;		//��ʱ��4״̬�Ĵ���
void TIM2_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}
void TIM2_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period =0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 71;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_Cmd(TIM2, ENABLE);
}
void TIM2_Input_config(void)
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
}
void TIM8_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7| GPIO_Pin_8 | GPIO_Pin_9;     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}
void TIM8_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period =0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 71;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
  TIM_Cmd(TIM8, ENABLE);
}
void TIM8_Input_config(void)
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM8, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM8, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM8, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM8, &TIM_ICInitStructure);
}
void NVIC_Capture_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void PWM_IN_Configuration(void)
{
  TIM2_GPIO_Config();
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
  TIM2_Mode_Config();
  TIM2_Input_config();

  TIM8_GPIO_Config();
  TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);
  TIM_ITConfig(TIM8, TIM_IT_CC2, ENABLE);
  TIM_ITConfig(TIM8, TIM_IT_CC3, ENABLE);
  TIM_ITConfig(TIM8, TIM_IT_CC4, ENABLE);
  TIM8_Mode_Config();
  TIM8_Input_config();
  NVIC_Capture_Config();
	PwmIn[0]=1000;
	PwmIn[1]=1000;
	PwmIn[2]=1000;
	PwmIn[3]=1000;
	PwmIn[4]=1000;
	PwmIn[5]=1000;
	PwmIn[6]=1000;
}
void TIM2_IRQHandler(void) 
{
	t2sr=TIM2->SR;
	//CH1�ж�
	if(t2sr&0x02)
	{
		if(TIM2Capture_STA&0x01)		//����һ���½���
		{
			Drop[0]=TIM2->CCR1;		//��ȡ��ǰ����ֵ
			if(Rise[0]<=Drop[0])
				PwmIn[0]=Drop[0]-Rise[0];
			else
				PwmIn[0]=0xFFFF-Rise[0]+Drop[0];
			
			Rise[0]=0;
			Drop[0]=0;
			TIM2Capture_STA&=~0x01;		//����
			TIM2->CCER&=~(1<<1);		//����Ϊ�����ز���
		}
		else		//��һ�β���������
		{
			TIM2Capture_STA|=0x01;		//��ǲ���һ��������
			Rise[0]=TIM2->CCR1;		//��ȡ��ǰ����ֵ
			TIM2->CCER|=(1<<1);		//����Ϊ�½��ز���
		}
	}
	//CH2�ж�
	if(t2sr&0x04)
	{
		if(TIM2Capture_STA&0x02)		//����һ���½���
		{
			Drop[1]=TIM2->CCR2;		//��ȡ��ǰ����ֵ
			if(Rise[1]<=Drop[1])
				PwmIn[1]=Drop[1]-Rise[1];
			else
				PwmIn[1]=0xFFFF-Rise[1]+Drop[1];
			
			Rise[1]=0;
			Drop[1]=0;
			TIM2Capture_STA&=~0x02;		//����
			TIM2->CCER&=~(1<<5);		//����Ϊ�����ز���
		}
		else		//��һ�β���������
		{
			TIM2Capture_STA|=0x02;		//��ǲ���һ��������
			Rise[1]=TIM2->CCR2;		//��ȡ��ǰ����ֵ
			TIM2->CCER|=(1<<5);		//����Ϊ�½��ز���
		}
	}
	//CH3�ж�
	if(t2sr&0x08)
	{
		if(TIM2Capture_STA&0x04)		//����һ���½���
		{
			Drop[2]=TIM2->CCR3;		//��ȡ��ǰ����ֵ
			if(Rise[2]<=Drop[2])
				PwmIn[2]=Drop[2]-Rise[2];
			else
				PwmIn[2]=0xFFFF-Rise[2]+Drop[2];
			
			Rise[2]=0;
			Drop[2]=0;
			TIM2Capture_STA&=~0x04;		//����
			TIM2->CCER&=~(1<<9);		//����Ϊ�����ز���
		}
		else		//��һ�β���������
		{
			TIM2Capture_STA|=0x04;		//��ǲ���һ��������
			Rise[2]=TIM2->CCR3;		//��ȡ��ǰ����ֵ
			TIM2->CCER|=(1<<9);		//����Ϊ�½��ز���
		}
	}
	//CH4�ж�
	if(t2sr&0x10)
	{
		if(TIM2Capture_STA&0x08)		//����һ���½���
		{
			Drop[3]=TIM2->CCR4;		//��ȡ��ǰ����ֵ
			if(Rise[3]<=Drop[3])
				PwmIn[3]=Drop[3]-Rise[3];
			else
				PwmIn[3]=0xFFFF-Rise[3]+Drop[3];
			
			Rise[3]=0;
			Drop[3]=0;
			TIM2Capture_STA&=~0x08;		//����
			TIM2->CCER&=~(1<<13);		//����Ϊ�����ز���
		}
		else		//��һ�β���������
		{
			TIM2Capture_STA|=0x08;		//��ǲ���һ��������
			Rise[3]=TIM2->CCR4;		//��ȡ��ǰ����ֵ
			TIM2->CCER|=(1<<13);		//����Ϊ�½��ز���
		}
	}
	TIM2->SR=0;
}

void TIM8_CC_IRQHandler(void) 
{
	t8sr=TIM8->SR;
	//CH1�ж�
	if(t8sr&0x02)
	{
		if(TIM8Capture_STA&0x01)		//����һ���½���
		{
			Drop[4]=TIM8->CCR1;		//��ȡ��ǰ����ֵ
			if(Rise[4]<=Drop[4])
				PwmIn[4]=Drop[4]-Rise[4];
			else
				PwmIn[4]=0xFFFF-Rise[4]+Drop[4];
			
			Rise[4]=0;
			Drop[4]=0;
			TIM8Capture_STA&=~0x01;		//����
			TIM8->CCER&=~(1<<1);		//����Ϊ�����ز���
		}
		else		//��һ�β���������
		{
			TIM8Capture_STA|=0x01;		//��ǲ���һ��������
			Rise[4]=TIM8->CCR1;		//��ȡ��ǰ����ֵ
			TIM8->CCER|=(1<<1);		//����Ϊ�½��ز���
		}
	}
	//CH2�ж�
	if(t8sr&0x04)
	{
		if(TIM8Capture_STA&0x02)		//����һ���½���
		{
			Drop[5]=TIM8->CCR2;		//��ȡ��ǰ����ֵ
			if(Rise[5]<=Drop[5])
				PwmIn[5]=Drop[5]-Rise[5];
			else
				PwmIn[5]=0xFFFF-Rise[5]+Drop[5];
			
			Rise[5]=0;
			Drop[5]=0;
			TIM8Capture_STA&=~0x02;		//����
			TIM8->CCER&=~(1<<5);		//����Ϊ�����ز���
		}
		else		//��һ�β���������
		{
			TIM8Capture_STA|=0x02;		//��ǲ���һ��������
			Rise[5]=TIM8->CCR2;		//��ȡ��ǰ����ֵ
			TIM8->CCER|=(1<<5);		//����Ϊ�½��ز���
		}
	}
	//CH3�ж�
	if(t8sr&0x08)
	{
		if(TIM8Capture_STA&0x04)		//����һ���½���
		{
			Drop[6]=TIM8->CCR3;		//��ȡ��ǰ����ֵ
			if(Rise[6]<=Drop[6])
				PwmIn[6]=Drop[6]-Rise[6];
			else
				PwmIn[6]=0xFFFF-Rise[6]+Drop[6];
			
			Rise[6]=0;
			Drop[6]=0;
			TIM8Capture_STA&=~0x04;		//����
			TIM8->CCER&=~(1<<9);		//����Ϊ�����ز���
		}
		else		//��һ�β���������
		{
			TIM8Capture_STA|=0x04;		//��ǲ���һ��������
			Rise[6]=TIM8->CCR3;		//��ȡ��ǰ����ֵ
			TIM8->CCER|=(1<<9);		//����Ϊ�½��ز���
		}
	}
	//CH4�ж�
	if(t8sr&0x10)
	{
		if(TIM8Capture_STA&0x08)		//����һ���½���
		{
			Drop[7]=TIM8->CCR4;		//��ȡ��ǰ����ֵ
			if(Rise[7]<=Drop[7])
				PwmIn[7]=Drop[7]-Rise[7];
			else
				PwmIn[7]=0xFFFF-Rise[7]+Drop[7];
			
			Rise[7]=0;
			Drop[7]=0;
			TIM8Capture_STA&=~0x08;		//����
			TIM8->CCER&=~(1<<13);		//����Ϊ�����ز���
		}
		else		//��һ�β���������
		{
			TIM8Capture_STA|=0x08;		//��ǲ���һ��������
			Rise[7]=TIM8->CCR4;		//��ȡ��ǰ����ֵ
			TIM8->CCER|=(1<<13);		//����Ϊ�½��ز���
		}
	}
	TIM8->SR=0;
}
