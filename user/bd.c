#include "bd.h"

#define PI 3.14159265358979323846
#define my_cos(rad) (my_sin(rad+PI/2))

_beidou beidou;

/**
将含有小数点的字符串形式的数据转化为浮点数
*@DataStr:待转化的字符串
*@len:字符串长度
*/
float my_atof(char *DataStr,u8 len)
{
	u8 i = 0, decimal = 0;
	float ans = 0, ans2 = 0;
	for (i = 0; DataStr[i] != '.'; i++)
		ans = ans * 10 + DataStr[i] - 48;
	for (i++; i < len; i++)
	{
		ans2 = ans2 * 10 + DataStr[i] - 48;
		decimal++;
	}
	for (i = 0; i < decimal; i++)
		ans2 /= 10.0;
	ans += ans2;
	return ans;
}

float Angle_To_Float(float angle)
{
	float ans = (short)angle / 100;
	ans = (angle - ans * 100) / 60.0 + ans;
	return ans;
}

double my_sin(double rad)
{
	double sine;
	while (rad < -PI)
		rad += 2 * PI;
	while (rad > PI)
		rad -= 2 * PI;
	if (rad < 0)
		sine = rad*(1.27323954f + 0.405284735f * rad);
	else
		sine = rad * (1.27323954f - 0.405284735f * rad);
	if (sine < 0)
		sine = sine*(-0.225f * (sine + 1) + 1);
	else
		sine = sine * (0.225f *(sine - 1) + 1);
	return sine;
}
/**
//预解析，根据帧头选出有效数据并保存，准备进一步处理
//串口每收到一字节数据，则调用此函数一次
//数据示例：
$GNRMC,084852.000,A,2236.9453,N,11408.4790,E,0.53,292.44,141216,,,A*75
*/
u8 Beidou_Data_Receive_Prepare(u8 data,u8 *RxBuffer)
{
	static u8 state,sum,cnt,cmp;
	if(data == '$')//接收新的一行数据
	{
		state=0;//状态清零
		sum=0;//校验和清零
		cnt=0;//计数清零
	}
	switch(state)
	{
		case 0://确定是否收到"GPRMC/GNRMC"这一帧数据
			RxBuffer[cnt++] = data;
			if(RxBuffer[3] == 'R' && RxBuffer[4] == 'M' && RxBuffer[5] == 'C')
				state=1;
			break;
		case 1:
			if(data=='A')//丢弃帧头，从有效数据开始
			{
				state=2;
				cnt=0;
			}
			break;
		case 2:
			RxBuffer[cnt++]=data;
			break;
		case 3:
			cmp=data-48;
			state=4;
			break;
		case 4:
			cmp=data-48+(cmp<<4);
			if(sum==cmp) return 0;//校验成功
			else return 1;//校验失败
			//break;
		default:break;
	}
	if(data!='$' && data!='*' && state<3)//$与*之间所有字符ASCII码的校验和
		sum^=data;
	if(data=='*')
		state=3;
	return 2;//未接收完成
}

/**
//从有效数据中提取信息
//务必在预解析成功（返回0）后调用
//数据示例：
,2236.9453,N,11408.4790,E,0.53,292.44,141216,,,A*
*/
u8 Beidou_Data_Receive_Anl(u8 *RxBuffer)
{
	char ch[10];
	float speed,angle;//航速，航向
	u8 i=0,j=0,cnt;
	while(i<4)
	{
		cnt=0;j++;
		if(RxBuffer[j]<0x30 || RxBuffer[j]>0x3F)continue;
		for(;RxBuffer[j]!=',';j++)
		{
			ch[cnt]=RxBuffer[j];
			if(j>49)
				return 1;//数据错误
			cnt++;
		}
		switch(i)
		{
			case 0:beidou.lat=Angle_To_Float(my_atof(ch,cnt));break;
			case 1:beidou.lon=Angle_To_Float(my_atof(ch,cnt));break;
			case 2:speed=my_atof(ch,cnt);break;
			case 3:angle=my_atof(ch,cnt);break;
			default:break;
		}
		i++;
	}
	beidou.speed_x=666.72*speed*my_sin(angle*PI/180.0);//cm/s
	beidou.speed_y=666.72*speed*my_cos(angle*PI/180.0);
	return 0;
}
