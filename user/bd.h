#ifndef __BD_H
#define __BD_H

#include "stm32f10x.h"

typedef struct beidou 
{
	u8 state;
	u8 sat_num;
	float lon;
	float lat;
	float back_home_angle;
	float speed_x;
	float speed_y;
	float speed_z;
}_beidou;

extern _beidou beidou;

u8 Beidou_Data_Receive_Prepare(u8 data,u8 *RxBuffer);
u8 Beidou_Data_Receive_Anl(u8 *RxBuffer);

#endif
