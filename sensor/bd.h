#ifndef __BD_H
#define __BD_H

#include "stm32f10x.h"

typedef struct beidou 
{
	u8 state;
	u8 sat_num;
	s32 lon;
	s32 lat;
	float back_home_angle;
}_beidou;

extern _beidou beidou;

void BEIDOU_Update(void);
void Beidou_Data_Receive_Prepare(u8 data,u8 *RxBuffer);

#endif
