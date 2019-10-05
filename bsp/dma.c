#include "dma.h"

u8 SendBuff[SENDBUF_SIZE];
u16 TotalLen=0;
void NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
//DMA1�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_CHx:DMAͨ��CHx
//cpar:�����ַ
//cmar:�洢����ַ
//cndtr:���ݴ����� 
void MYDMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����
	
  DMA_DeInit(DMA1_Channel4);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;  //DMA����ADC����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SendBuff;  //DMA����ADC����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);  //����DMA������ɺ�����ж�
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	NVIC_Config();
}
//����һ��DMA����
void MYDMA_Enable(void)
{
	DMA_Cmd(DMA1_Channel4, DISABLE );  //�ر�USART1 TX DMA1 ��ָʾ��ͨ��      
	DMA1_Channel4->CNDTR=TotalLen;  //DMAͨ����DMA����Ĵ�С
 	DMA_Cmd(DMA1_Channel4, ENABLE);  //ʹ��USART1 TX DMA1 ��ָʾ��ͨ�� 
}
void DMA_Stuff(u8 *Data,u8 len)
{
	if(len==0)
	{
		TotalLen=0;
		return;
	}
	u8 i;
	for(i=0;i<len;i++)
	{
//		if(TotalLen+i>=SENDBUF_SIZE) while(1);//���������ʲ����쵼�»���������ǿ�Ƹ�λ�����ã�
		SendBuff[TotalLen+i]=Data[i];
	}
	TotalLen+=len;
}
void DMA1_Channel4_IRQHandler(void)
{
//�ж��Ƿ�ΪDMA��������ж�
	if(DMA_GetFlagStatus(DMA1_FLAG_TC4)==SET) 
	{
		DMA_ClearFlag(DMA1_FLAG_TC4);
		DMA_Stuff(0,0);
	}
}
