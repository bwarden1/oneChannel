#include "nimh.h"
#include <hic.h>


extern u32 shortTick;
extern u32 tick;

void delay_ms(u16 nus)
{
    unsigned int i , j;
    for(i=0;i<nus;i++)
   {
   	__Asm CWDT;
	for(j=0;j<1000;j++);
    }
}

void delay_us(u16 nus)
{
	u16 i;
	for(i=0;i<nus;i++);
}
u32 getSysTick()
{
	u32 tempTick;

	GIE = 0;
	__Asm NOP; __Asm NOP;__Asm NOP;__Asm NOP;
	tempTick = shortTick;
	__Asm NOP; __Asm NOP;__Asm NOP;__Asm NOP;
	GIE=1;

	return tempTick;
}

/*
u32 getBatTick()
{
	u32 tempTick;

	GIE=0;
	__Asm NOP; __Asm NOP;
	tempTick = tick;
	GIE=1;

	return tempTick;
}
*/

u32 getDiffTickFromNow(u32 ttick)
{
	u32 ticknow =  getSysTick();

	if(ttick <= ticknow)
		return (ticknow - ttick); //getDiffTick(ttick, ticknow);
	else
		return (0xFFFFFFFF-ttick+ticknow);
}

