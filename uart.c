#include "nimh.h"
#include "hic.h"

#if 0
const char digits[] = "0123456789abcdef";
void send(u16 sData)
{
	signed char i;

	for(i=3; i>=0; i--)
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

	for(i=0;i<5;i++)
	{
        while(!TRMT);           //等待发送移位寄存器TXR空
        TXB  =  str[i];   //发送字符串
        while(!TXIF);           //等待发送中断标志位 
        TXIF = 0;               //清发送中断标志位 	
	}

	while(!TRMT);           //等待发送移位寄存器TXR空
        TXB  =  ' ';   //发送字符串
        while(!TXIF);           //等待发送中断标志位 
        TXIF = 0;               //清发送中断标志位 
}
#endif
