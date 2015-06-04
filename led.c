#include "nimh.h"
#include <hic.h>

extern u8 gBatStateBuf[5];
void LED_ON(u8 led)
{
	switch(led)
	{
		case 1:
			PA0=0;break;
		case 2:
			PA1=0;break;
		case 3:
			PC1=0;break;
		case 4:
			PC0=0;break;
	}
}

void LED_OFF(u8 led)
{
	switch(led)
	{
		case 1:
			PA0=1;break;
		case 2:
			PA1=1;break;
		case 3:
			PC1=1;break;
		case 4:
			PC0=1;break;
	}	
}

void ledHandler(void)
{
	u8 i;
	
	//charging state
	if(getSysTick() & SHOW_CHARGING_TICK)
	{
		for(i=1;i<5;i++)
		{	
			if(gBatStateBuf[i] & 0x38)
				LED_ON(i);
		}
	}
	else
	{
		for(i=1;i<5;i++)
		{
			if((gBatStateBuf[i] & 0x38) && (gBatStateBuf[i] & CHARGE_STATE_FULL)==0)
				LED_OFF(i);
		}
	}



	//error state

	if(getSysTick() & 0x08)
	{
		for(i=1;i<5;i++)
		{
			if(gBatStateBuf[i] & ((CHARGE_STATE_ERROR)|(BAT_TYPE_ERROR) ))
				LED_ON(i);	
		}
	}
	else
	{
		for(i=1;i<5;i++)
		{
			if(gBatStateBuf[i] & ((CHARGE_STATE_ERROR)|(BAT_TYPE_ERROR) ))
				LED_OFF(i);		
		}
	}

	
	
}

