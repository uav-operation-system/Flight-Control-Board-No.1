#include "usart.h"

void uart1_init(u32 bound)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
  
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1
}

void uart3_init(u32 bound_usart3)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART2，GPIOA时钟
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	 
     //USART3_TX   PB.10
		 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //USART3_RX	  PB.11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

   //Usart3 NVIC 配置

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4 ;//抢占优先级4
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;		//子优先级4
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	 USART_InitStructure.USART_BaudRate =bound_usart3;//一般设置为38400;
	 USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	 USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	 USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

   USART_Init(USART3, &USART_InitStructure); //初始化串口
   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启中断
   USART_Cmd(USART3, ENABLE);                    //使能串口
}

u8 USART1_RxBuffer[20];
u8 USART3_RxBuffer[70];
void USART1_IRQHandler(void)
{
	u8 Res1;
	if(USART1->SR&(1<<5))
	{
		Res1 =USART1->DR;
		if(!ANO_Data_Receive_Prepare(Res1,USART1_RxBuffer))
			ANO_Data_Receive_Anl(USART1_RxBuffer);
	}
}
void USART3_IRQHandler(void)
{
	u8 Res2;
	if(USART3->SR&(1<<5))
	{
		Res2 =USART3->DR;
		if(!Beidou_Data_Receive_Prepare(Res2,USART3_RxBuffer))
			Beidou_Data_Receive_Anl(USART3_RxBuffer);
	}
}
