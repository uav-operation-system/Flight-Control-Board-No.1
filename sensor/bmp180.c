#include "bmp180.h"
#include "task.h"

//�洢BMP180���ݵĽṹ
_bmp180 bmp180;
short BMP_ReadTwoByte(uint8_t ReadAddr);

//BMP180��ʼ��
//��ʹ�õ�IIC�˿ڽ��г�ʼ��
void BMP_Init(void)
{
	IIC_Init();
	bmp180.AC1 = BMP_ReadTwoByte(0xAA);
	bmp180.AC2 = BMP_ReadTwoByte(0xAC);
	bmp180.AC3 = BMP_ReadTwoByte(0xAE);
	bmp180.AC4 = BMP_ReadTwoByte(0xB0);
	bmp180.AC5 = BMP_ReadTwoByte(0xB2);
	bmp180.AC6 = BMP_ReadTwoByte(0xB4);
	bmp180.B1  = BMP_ReadTwoByte(0xB6);
	bmp180.B2  = BMP_ReadTwoByte(0xB8);
	bmp180.MB  = BMP_ReadTwoByte(0xBA);
	bmp180.MC  = BMP_ReadTwoByte(0xBC);
	bmp180.MD  = BMP_ReadTwoByte(0xBE);
	bmp180.state = 0;
}

//дһ�����ݵ�BMP180
void BMP_WriteOneByte(uint8_t WriteAddr,uint8_t DataToWrite)
{
	IIC_Start();
	
	IIC_Send_Byte(0xEE);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(WriteAddr);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(DataToWrite);
	IIC_Wait_Ack();
	IIC_Stop();
}

//��BMP180��һ���ֽ�����
uint8_t BMP_ReadOneByte(uint8_t ReadAddr)
{
	uint8_t data = 0;
	
	IIC_Start();
	
	IIC_Send_Byte(0xEE);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(ReadAddr);
	IIC_Wait_Ack();
	
	IIC_Start();
	
	IIC_Send_Byte(0xEF);
	IIC_Wait_Ack();
	
	data = IIC_Read_Byte(1);
	IIC_Stop();
	
	return data;
}

//��BMP180��һ��16λ������
short BMP_ReadTwoByte(uint8_t ReadAddr)
{
	short data;
	uint8_t msb,lsb;
	
	IIC_Start();
	
	IIC_Send_Byte(0xEE);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(ReadAddr);
	IIC_Wait_Ack();
	
	IIC_Start();
	
	IIC_Send_Byte(0xEF);
	IIC_Wait_Ack();
	
	msb = IIC_Read_Byte(1);
	lsb = IIC_Read_Byte(0);
	
	IIC_Stop();
	
	data = msb*256 + lsb;
	
	return data;
}

//�û�ȡ�Ĳ������¶Ⱥʹ���ѹ���������������㺣��
void bmp180Convert(void)
{
	if(bmp180.state==0)
	{
		BMP_WriteOneByte(0xF4,0x2E);
		bmp180.state++;
		return;
	}
	if(bmp180.state==1)
	{
		bmp180.UT = (long)BMP_ReadTwoByte(0xF6);
		BMP_WriteOneByte(0xF4,0x34);
		bmp180.state++;
		return;
	}
	bmp180.UP = (long)BMP_ReadTwoByte(0xF6);
	bmp180.UP &= 0x0000FFFF;
	
/*���³��������ϳ����������ӣ�Ϊ����������ȸĳ������ϳ����д����
	//��BMP180��ȡδ�������¶�
	BMP_WriteOneByte(0xF4,0x2E);
	delay_ms(5);
	bmp180.UT = (long)BMP_ReadTwoByte(0xF6);
	//��BMP180��ȡδ�����Ĵ���ѹ
	BMP_WriteOneByte(0xF4,0x34);
	delay_ms(5);
	bmp180.UP = (long)BMP_ReadTwoByte(0xF6);
	bmp180.UP &= 0x0000FFFF;
*/
	bmp180.X1 = ((bmp180.UT - bmp180.AC6) * bmp180.AC5) >> 15;
	bmp180.X2 = (((long)bmp180.MC) << 11) / (bmp180.X1 + bmp180.MD);
	bmp180.B5 = bmp180.X1 + bmp180.X2;
	bmp180.Temp  = (bmp180.B5 + 8) >> 4;
	
	bmp180.B6 = bmp180.B5 - 4000;
	bmp180.X1 = ((long)bmp180.B2 * (bmp180.B6 * bmp180.B6 >> 12)) >> 11;
	bmp180.X2 = ((long)bmp180.AC2) * bmp180.B6 >> 11;
	bmp180.X3 = bmp180.X1 + bmp180.X2;
	
	bmp180.B3 = ((((long)bmp180.AC1) * 4 + bmp180.X3) + 2) /4;
	bmp180.X1 = ((long)bmp180.AC3) * bmp180.B6 >> 13;
	bmp180.X2 = (((long)bmp180.B1) *(bmp180.B6*bmp180.B6 >> 12)) >>16;
	bmp180.X3 = ((bmp180.X1 + bmp180.X2) + 2) >> 2;
	bmp180.B4 = ((long)bmp180.AC4) * (unsigned long)(bmp180.X3 + 32768) >> 15;
	bmp180.B7 = ((unsigned long)bmp180.UP - bmp180.B3) * 50000;
	
	if(bmp180.B7 < 0x80000000)
		bmp180.p = (bmp180.B7 * 2) / bmp180.B4;
	else
		bmp180.p = (bmp180.B7 / bmp180.B4) * 2;
	
	bmp180.X1 = (bmp180.p >> 8) * (bmp180.p >>8);
	bmp180.X1 = (((long)bmp180.X1) * 3038) >> 16;
	bmp180.X2 = (-7357 * bmp180.p) >> 16;
	bmp180.p = bmp180.p + ((bmp180.X1 + bmp180.X2 + 3791) >> 4);
	bmp180.altitude = 44330 * (1-my_pow(((bmp180.p) / 101325.0),(1.0/5.255)));
	bmp180.state=0;
}
