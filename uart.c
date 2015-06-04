
#include "hic.h"
#include "string.h"

#define u8	unsigned char
#define u16	unsigned int
#define u32	unsigned long

#if 1
const char digits[] = "0123456789abcdef";
void send(u16 sData)
{
	signed char i;

	for(i=7; i>=0; i--)
	{
        while(!TRMT);           //等待发送移位寄存器TXR空
        TXB  =  digits[(sData>>(i*4))&0xf];   //发送字符串
        while(!TXIF);           //等待发送中断标志位 
        TXIF = 0;               //清发送中断标志位 
	}

	  while(!TRMT);           //等待发送移位寄存器TXR空
        TXB  =  ' ';   //发送字符串
        while(!TXIF);           //等待发送中断标志位 
        TXIF = 0;               //清发送中断标志位 
}

void sendF(u32 sData)
{
	signed char i;

	for(i=7; i>=0; i--)
	{
        while(!TRMT);           //等待发送移位寄存器TXR空
        TXB  =  digits[(sData>>(i*4))&0xf];   //发送字符串
        while(!TXIF);           //等待发送中断标志位 
        TXIF = 0;               //清发送中断标志位 
	}

	  while(!TRMT);           //等待发送移位寄存器TXR空
        TXB  =  ' ';   //发送字符串
        while(!TXIF);           //等待发送中断标志位 
        TXIF = 0;               //清发送中断标志位 
}
void sendStr(char str[])
{
	u8 i;
	u8 len = strlen(str);
	
	for(i=0;i<len;i++)
	{
        while(!TRMT);           //等待发送移位寄存器TXR空
        TXB  =  str[i];   //发送字符串
        while(!TXIF);           //等待发送中断标志位 
        TXIF = 0;               //清发送中断标志位 	
	}
/*
	while(!TRMT);           //等待发送移位寄存器TXR空
        TXB  =  ' ';   //发送字符串
        while(!TXIF);           //等待发送中断标志位 
        TXIF = 0;               //清发送中断标志位 
        */
}
#endif
