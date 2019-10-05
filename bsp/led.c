#include "led.h"

void LED_GPIO_Config(void)
{	
    //����һ��GPIO_InitTypeDef ���͵Ľṹ��
    GPIO_InitTypeDef  GPIO_InitStructure;	
    RCC_APB2PeriphClockCmd(LED_GPIO_RCC,ENABLE);//ʹ��GPIO������ʱ��

		GPIO_InitStructure.GPIO_Pin =LED0_GPIO_PIN|LED1_GPIO_PIN|LED2_GPIO_PIN;//ѡ��Ҫ�õ�GPIO����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //��������ģʽΪ�������ģʽ						 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//���������ٶ�Ϊ50MHZ         
    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);//���ÿ⺯������ʼ��GPIO
}
void LED_Toggle(void)
{
   //LED1_GPIO_PORT->ODR^=LED1_GPIO_PIN;
}

void LED_SELECT(u8 AircraftMode){
    switch(AircraftMode){
		  case SIZHOU: LED_ONOFF(Bit_RESET,LED0_GPIO_PIN);  //D1 is enabled which means the module is'SIZHOU'
			             LED_ONOFF(Bit_SET,LED1_GPIO_PIN);
			             LED_ONOFF(Bit_SET,LED2_GPIO_PIN);
			break;
			
			case CHUIQI: LED_ONOFF(Bit_SET,LED0_GPIO_PIN);    //D2 is enabled which means the module is'CHUIQI'
			             LED_ONOFF(Bit_RESET,LED1_GPIO_PIN);
			             LED_ONOFF(Bit_SET,LED2_GPIO_PIN);
			break;
			case YUYING: LED_ONOFF(Bit_SET,LED0_GPIO_PIN);    //D3 is enabled which means the module is'YUYING'
			             LED_ONOFF(Bit_SET,LED1_GPIO_PIN);
			             LED_ONOFF(Bit_RESET,LED2_GPIO_PIN);
			break;
			case ZHIFEIJI: LED_ONOFF(Bit_RESET,LED0_GPIO_PIN);   //D1 and D2 is enabled which means the module is'ZHIFEIJI'
			               LED_ONOFF(Bit_RESET,LED1_GPIO_PIN);
			               LED_ONOFF(Bit_SET,LED2_GPIO_PIN);
			break;
			case LAOZUZONG: LED_ONOFF(Bit_RESET,LED0_GPIO_PIN);    //D1 and D3 is enabled which means the module is'LAOZUZONG'
			                LED_ONOFF(Bit_SET,LED1_GPIO_PIN);
			                LED_ONOFF(Bit_RESET,LED2_GPIO_PIN);
			break;
			case DUIDI: LED_ONOFF(Bit_SET,LED0_GPIO_PIN);       //D2 and D3 is enabled which means the module is'DUIDI'
			            LED_ONOFF(Bit_RESET,LED1_GPIO_PIN);
			            LED_ONOFF(Bit_RESET,LED2_GPIO_PIN);
		  break;
			default : LED_ONOFF(Bit_SET,LED0_GPIO_PIN);       //No one is enabled which means the module is unkown
			          LED_ONOFF(Bit_SET,LED1_GPIO_PIN);
			          LED_ONOFF(Bit_SET,LED2_GPIO_PIN);
								TIM6_Config();
								//NVIC_Config();
								//TIM6_IRQHandler();
			break;					
		}
}

/**********************************************************
** ������: TIM6_Config
** ��������: ������ʱ������LED��˸
** �������: ��
** �������: ��
** ˵��:��ʱʱ��=(Ԥ��Ƶ��+1)*(����ֵ+1) /TIM6ʱ��(72MHz),��λ(s)
   �������ʱ��t=(7200*10000)/72000000s=1s
***********************************************************/
void TIM6_Config(void){
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //ʹ��TIM6ʱ��


/*��������*/
TIM_TimeBaseStructure.TIM_Period = 5000-1;//����ֵ10000   
TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;    //Ԥ��Ƶ,��ֵ+1Ϊ��Ƶ�ĳ���
TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;  //������Ƶ
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);


TIM_ITConfig(TIM6,TIM_IT_Update, ENABLE);     //ʹ��TIM6�ж�
TIM_Cmd(TIM6, ENABLE);     //ʹ�ܶ�ʱ��6
}


/**********************************************************
** ������: TIM6_IRQHandler
** ��������: TIM6�����жϷ������
** ����˵��: ��
** ���˵��: ��
***********************************************************/
void TIM6_IRQHandler(void){
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET){
       TIM_ClearITPendingBit(TIM6,TIM_IT_Update);//��������жϱ�־λ
       GPIOE->ODR^=GPIO_Pin_0;//LED��
		   GPIOE->ODR^=GPIO_Pin_1;
		   GPIOE->ODR^=GPIO_Pin_2;
  }
}
