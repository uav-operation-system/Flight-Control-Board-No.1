#include "iwdg.h"
void IWDG_Init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����
	IWDG_SetPrescaler(IWDG_Prescaler_32);		  //����IWDGԤ��Ƶֵ��256��Ƶ���
	IWDG_SetReload(40000/128);	   //����IWDG����װ��ֵ	:��Χ0~0x0FFF
	IWDG_ReloadCounter();	   //ι��������װ�ؼ�����
	IWDG_Enable();			   //ʹ��IWDG��ʱ��
}//250ms
