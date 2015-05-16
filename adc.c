#include "nimh.h"
#include "hic.h"

//u8 tempStr1[] = "temp1:";
//u8 tempStr2[] = "temp2:";

extern u16 gChargeCurrent;

u16 getAdcValue(u8 channel)
{
	
	u16 temp;
	u8 tempH,tempL;


	ADCCL = 5| channel;


	ADTRG=1;
	while(ADTRG);

	tempL = ADCRL;	
	tempH = ADCRH;
	

	
	temp = ((u16)tempH)<<4;
	temp += (tempL>>4);

	ADEN=0;
	
	return temp;
}

u16 getAverage(u8 channel)
{
	u8 i;
	u16 temp,max,min,ret;


	temp = getAdcValue(channel<<4);
	ret= temp;
	max =temp;
	min = temp;
	for(i=0;i<9;i++)
	{
		delay_us(200);
		 temp = getAdcValue(channel<<4);
	 	if(temp > max)
	 	{
			max = temp;
	 	}

		 if(temp < min)
		{
			min = temp;
	 	}
	 	ret += temp;
	}	

	return (ret - max - min)>>3;
}

u16 getVbatAdc(u8 channel)
{
	u16 tempV;


	switch(channel)
	{
		case 1:
			channel = CHANNEL_VBAT_1;break;
		case 2:
			channel = CHANNEL_VBAT_2;break;
		case 3:
			channel = CHANNEL_VBAT_3;break;
		case 4:
			channel = CHANNEL_VABT_4;break;
		default:
			break;
	}


	gChargeCurrent= getAverage(CHANNEL_20_RES);
	tempV = getAverage(channel);

//	sendStr(tempStr1);
//	send(gChargeCurrent);
//	sendStr(tempStr2);
//	send(temp2);

	if(tempV < gChargeCurrent)
	{
		gChargeCurrent = 0;
		return 0;
	}
	return (tempV-gChargeCurrent);
}
