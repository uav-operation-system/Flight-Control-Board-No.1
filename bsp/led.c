#include "led.h"

void LED_GPIO_Config(void)
{	
    //定义一个GPIO_InitTypeDef 类型的结构体
    GPIO_InitTypeDef  GPIO_InitStructure;	
    RCC_APB2PeriphClockCmd(LED_GPIO_RCC,ENABLE);//使能GPIO的外设时钟

		GPIO_InitStructure.GPIO_Pin =LED0_GPIO_PIN|LED1_GPIO_PIN|LED2_GPIO_PIN;//选择要用的GPIO引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //设置引脚模式为推免输出模式						 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//设置引脚速度为50MHZ         
    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);//调用库函数，初始化GPIO
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
** 函数名: TIM6_Config
** 功能描述: 基本定时器控制LED闪烁
** 输入参数: 无
** 输出参数: 无
** 说明:定时时间=(预分频数+1)*(计数值+1) /TIM6时钟(72MHz),单位(s)
   这里溢出时间t=(7200*10000)/72000000s=1s
***********************************************************/
void TIM6_Config(void){
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //使能TIM6时钟


/*基础设置*/
TIM_TimeBaseStructure.TIM_Period = 5000-1;//计数值10000   
TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;    //预分频,此值+1为分频的除数
TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;  //采样分频
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);


TIM_ITConfig(TIM6,TIM_IT_Update, ENABLE);     //使能TIM6中断
TIM_Cmd(TIM6, ENABLE);     //使能定时器6
}


/**********************************************************
** 函数名: TIM6_IRQHandler
** 功能描述: TIM6更新中断服务程序
** 输入说明: 无
** 输出说明: 无
***********************************************************/
void TIM6_IRQHandler(void){
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET){
       TIM_ClearITPendingBit(TIM6,TIM_IT_Update);//清除更新中断标志位
       GPIOE->ODR^=GPIO_Pin_0;//LED闪
		   GPIOE->ODR^=GPIO_Pin_1;
		   GPIOE->ODR^=GPIO_Pin_2;
  }
}
