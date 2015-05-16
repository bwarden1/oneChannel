#include <hic.h>
#include "nimh.h"

u32 tick=0;
u32 shortTick=0;
u8 gIsChargingBatPos=1;   //�˿����ڳ���صı��    


u32 ChargingTimeTick = 0;
u16 gChargeCurrent;

//type_error charge_error  pre  fast sup  trick  charging(on_off)  is_detect(��ؼ��) bat_state(valid)
//the first byte is a dummy						
u8 gBatStateBuf[5] = {0x0,0x00,0x00,0x00,0x00};

/*
u16 gBatVoltArray[4][6] = {	{0,0,0,0,0,0},
						{0,0,0,0,0,0},
						{0,0,0,0,0,0},
						{0,0,0,0,0,0},
					  };
*/

u16 gBatVoltArray[24];

//#define gBatVoltArray(x,y)  *(p[x]+y)
//#define gBatVoltArray(x,y) gBatVoltArrayTemp[x*6+y]

//u16 *p[4] = {&(gBatVoltArrayTemp[0]),&(gBatVoltArrayTemp[6]),&(gBatVoltArrayTemp[12]),&(gBatVoltArrayTemp[18])};

u8 gChargeSkipCount[] = {0,0,0,0};    //����PWM����

u8 gBatNumNow = 0;


		// 1/2/3/4�ŵ��
u8 gBatNowBuf[5]={0,0,0,0,0};  //��ų�����ϵ�صı��

void isr(void) interrupt
{
	if(T8NIF)
	{
		T8NIF = 0;
		tick++;
	}
	if(T8P1IF)
	{
		T8P1IF=0;
		shortTick++;
	}
}


u32 gChargingTimeTick[4] ={0,0,0,0};
void PreCharge(u8 batNum)
{
		if(getDiffTickFromNow(gChargingTimeTick[batNum-1]) > BAT_CHARGING_PRE_MAX_TIME)
		{
			gBatStateBuf[batNum] &= ~CHARGE_STATE_PRE;
			gBatStateBuf[batNum] |= CHARGE_STATE_ERROR;
			gChargingTimeTick[batNum-1] = 0;
		}
		else
		{
			gBatVoltArray[0] = getVbatAdc(batNum);
			if(gBatVoltArray[0] >= CHARGING_PRE_END_VOLT)
			{
				gChargingTimeTick[batNum-1] = 0;
				gBatStateBuf[batNum] &= ~CHARGE_STATE_PRE;
				gBatStateBuf[batNum] |= CHARGE_STATE_FAST;
			}
		}
}

void FastCharge(u8 batNum)
{
		if(getDiffTickFromNow(gChargingTimeTick[batNum-1]) > BAT_CHARGING_PRE_MAX_TIME)
		{
			gBatStateBuf[batNum] &= ~CHARGE_STATE_PRE;
			gBatStateBuf[batNum] |= CHARGE_STATE_SUP;
			gChargingTimeTick[batNum-1] = 0;
		}
		else
		{
			gBatVoltArray[0] = getVbatAdc(batNum);
			if(gBatVoltArray[0] >= CHARGING_FAST_END_VOLT)
			{
				gChargingTimeTick[batNum-1] = 0;
				gBatStateBuf[batNum] &= ~CHARGE_STATE_PRE;
				gBatStateBuf[batNum] |= CHARGE_STATE_SUP;
			}
			else if(getDiffTickFromNow(gChargingTimeTick[batNum-1]) > BAT_START_DV_CHECK)
			{
				
			}
		}
}

void SupCharge(u8 batLabel)
{
	if((PB & batLabel) == 0)  //not charging
	{
		if(gChargeSkipCount[batLabel] >= FAST_SKIP_COUNT)
		{
			PB |= batLabel;
			gChargeSkipCount[batLabel] =0;
		}
	}
}

void TrickCharge(u8 batLabel)
{
	if((PB & batLabel) == 0)  //not charging
	{
		if(gChargeSkipCount[batLabel] >= FAST_SKIP_COUNT)
		{
			PB |= batLabel;
			gChargeSkipCount[batLabel] =0;
		}
	}	
}


void removeBat(u8 toChangeBatPos)
{
	u8 i;

	gChargeSkipCount[gBatNowBuf[toChangeBatPos] -1 ] = 0;
	gBatStateBuf[gBatNowBuf[toChangeBatPos]] =0;

	LED_OFF(gBatNowBuf[toChangeBatPos]);
	PB &= 0xF0;   //close current pwm channel
	
	for(i=toChangeBatPos;i<gBatNumNow;i++)
	{
		gBatNowBuf[i] = gBatNowBuf[i+1];
	}
	gBatNowBuf[gBatNumNow] = 0;
	gBatNumNow--;
}

void chargeHandler(void)
{
	u32 ticknow = getBatTick();
	u16 tempV;

	static u8 skipCount = 0;
	
	//close all pwm
	//PB &= 0xF0;

	if(gBatNumNow ==0)
		return;

	if(ChargingTimeTick == 0)   
	{
		switch(gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] & 0x3C)	
		{
			case CHARGE_STATE_FAST:
					skipCount = FAST_SKIP_COUNT; break; //BatFastCharge(ticknow%4);break;
			case CHARGE_STATE_SUP:
					skipCount = SUP_SKIP_COUNT;break;// BatSupCharge(ticknow%4);break;
			case CHARGE_STATE_PRE:
					skipCount = PRE_SKIP_COUNT;break;//BatPreCharge(ticknow%4);break;
			case CHARGE_STATE_TRICK:
					skipCount = TRI_SKIP_COUNT;break;//BatTrickCharge(ticknow%4);break;
			default:
					break;
		}

		ChargingTimeTick = getSysTick();
		if(ChargingTimeTick == 0)
		{
			GIE =0 ;
			shortTick = 1;
			GIE =1;
			ChargingTimeTick =1;
		}
		if(gIsChargingBatPos !=0)    //0 pos is a dummy
		{
			//������ʹ���    ������
			if(gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] & ((BAT_TYPE_ERROR) |(CHARGE_STATE_ERROR)))
			{
				tempV = getVbatAdc(gBatNowBuf[gIsChargingBatPos]);
				if(tempV < BAT_MIN_VOLT_OPEN)
				{
					ChargingTimeTick = 0;
					removeBat(gIsChargingBatPos);
				}	
				return;	
			}
	
			if(gChargeSkipCount[gBatNowBuf[gIsChargingBatPos]-1] >=skipCount)   //ok, it's pulse now
			{
				PB &= 0xF0;   //close all pwm
				PB |= (1<<(gBatNowBuf[gIsChargingBatPos]-1));
			}
			else
				gChargeSkipCount[gBatNowBuf[gIsChargingBatPos] - 1]++;
		}
	}
	else
	{
		 if(gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] & BAT_DETECT_BIT)   //��ؼ��
		{
			if(getDiffTickFromNow(ChargingTimeTick)>BAT_CHARGING_DETECT_TIME)
			{
				tempV = getVbatAdc(gBatNowBuf[gIsChargingBatPos]);
				if(tempV<BAT_MIN_VOLT_OPEN)   //��ر��γ�
				{
					ChargingTimeTick = 0;
					removeBat(gIsChargingBatPos);
				}
				else if(tempV>BAT_MAX_VOLT_CLOSE  || (gChargeCurrent<<3) <= ( tempV-gBatVoltArray[0]))  //
				{
					gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] &= ~(BAT_DETECT_BIT |CHARGE_STATE_ALL);
					gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] |= BAT_TYPE_ERROR;
					gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] |= HAS_BATTERY;
					PB &= 0xF0;   //close current pwm channel
					ChargingTimeTick = 0;
				}
				else   //��ؼ��ok
				{
					//gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] &= ~ BAT_DETECT_BIT;
					gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] = 0;
					gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] |= HAS_BATTERY;
					if(gBatVoltArray[0] < CHARGING_PRE_END_VOLT)
						gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] |= CHARGE_STATE_PRE;
					else if(gBatVoltArray[0] < CHARGING_PRE_END_VOLT)
						gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] |= CHARGE_STATE_FAST;
					else
						gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] |= CHARGE_STATE_TRICK;
						
				}
  					
  			}
		}
		 else if(getDiffTickFromNow(ChargingTimeTick)  > BAT_CHARGING_PULSE_TIME)   //change to next channel
		{
			PB &= 0xF0;   //close current pwm channel
			ChargingTimeTick = 0;
			if(gIsChargingBatPos >= gBatNumNow)
			{
				if(gBatNumNow%2)
				{
					gIsChargingBatPos =0;
				}
				else
					gIsChargingBatPos =1;
			}
			else
				gIsChargingBatPos++;
		}
		else if(gIsChargingBatPos !=0)
		{ 
			tempV = getVbatAdc(gBatNowBuf[gIsChargingBatPos]);
			if(tempV < BAT_MIN_VOLT_OPEN  || (PB&(1<<(gBatNowBuf[gIsChargingBatPos]-1)) && gChargeCurrent <=2))
			{
					ChargingTimeTick = 0;
					removeBat(gIsChargingBatPos);
			}
			else
			{
				if(gChargingTimeTick[gBatNowBuf[gIsChargingBatPos]-1] == 0)
				{
					gChargingTimeTick[gBatNowBuf[gIsChargingBatPos]-1] = getSysTick();
					if(gChargingTimeTick[gBatNowBuf[gIsChargingBatPos]-1] == 0)
					{
						GIE =0;
						shortTick=1;
						GIE= 1;
						gChargingTimeTick[gBatNowBuf[gIsChargingBatPos]-1] = 1;
					}
				}
				switch(gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] & 0x3C)
				{
					case CHARGE_STATE_FAST:
							FastCharge(gBatNowBuf[gIsChargingBatPos]);break;
					case CHARGE_STATE_SUP:
							 SupCharge(gBatNowBuf[gIsChargingBatPos]);break;
					case CHARGE_STATE_PRE:
							PreCharge(gBatNowBuf[gIsChargingBatPos]);break;
					case CHARGE_STATE_TRICK:
							TrickCharge(gBatNowBuf[gIsChargingBatPos]);break;
					default:
							break;
				}
			}
		}
	}
}

void InitConfig()
{
	DisWatchdog();

	//ADC	
	ANSL = 0x03;   //AIN2~AIN6
	ANSH = 0x9F;	 //AIN12 AIN13

	#if 1
	ADCTST = 0x00;   //ADת���ٶ�Ϊ����
	ADCCL	= 0x05;  //Ӳ�����Ʋ���
	ADCCH = 0x4C;   //ADC����ʱ��Ϊ16��ADCʱ�ӣ�����ֵ��λ���룬A/Dת��ʱ��Ƶ��ΪFosc/16���ο���ѹΪ�ڲ�2.6V
	#else
	ADCTST = 0x00;   //ADת���ٶ�Ϊ����
	ADCCL	= 0x05;  //Ӳ�����Ʋ���
	ADCCH = 0x4D;
	VRC1 = 0x80;      // internal 2.6V enbale
	#endif
	
	//IO led1~4  �����
	PAT 	&=	0xFC;
	PCT	&=	0xFC;
	PA	|= 0x3;
	PC   |= 0x3;

	//pwm
	PBT &= 0xF0;
	PB  &= 0xF0;

	//timerN
	//bit    000(1:128) 	 1(en)	  NE	      0(timer mode)       1(ʱ��ԴΪWDT)      0( dis)
	//	T8NPRS	    T8NPRE	T8NEG  T8NM	                T8NCLK		                      T8EN
	T8NC =   0x4E;
	T8N = 0;
	T8NIF = 0;
	T8NIE=1;

	//t8p1
	T8P1=0; //celar count
	T8P1P=0xFF; //��������
	T8P1C= 0x7B; ///��ʱ��ģʽ��Ԥ��Ƶ��1:16, ���Ƶ��1:16
	T8P1IF = 0;
	T8P1IE = 1;	

/*
	//uart
    PC = 0;         //����PC�˿�����͵�ƽ 
    PCT1 = 0;       //TX�������
    PCT0 = 1;       //RX��������
    BRGH = 1;       //�����ʸ���ģʽ��Fosc/(16*(BRR+1))
    BRR = 51;       //���ò�����9600��BRR=8MHz/9600/16-1
    TXM = 0;        //����8λ���ݸ�ʽ
    TXEN = 1;       //UART����ʹ��   
// RXM = 0;        //����8λ���ݸ�ʽ
   //RXEN = 1;       //UART����ʹ��
*/
	
}


extern void LED_OFF(u8 led);
extern void LED_ON(u8 led);
extern void delay_ms(u16 nus);
void main() 
{
	
	u8 cur_detect_pos=1;
	u16 tempVoltClose,tempVoltOpen;

	
	InitConfig();

	RCEN=1;
	GIE=1;

	t8n_start();
	t8p1_start();

	WDTC = 0x0F;

	while(1)
	{
		if(gBatStateBuf[2] & HAS_BATTERY)
		{
			//getVbatAdc(cur_detect_pos);
		}
		else
		{
			if((gBatStateBuf[2] & BAT_DETECT_BIT) == 0)
			{	
				gBatVoltArray[0]=  getVbatAdc(2);	//gBatVoltArray[cur_detect_pos-1][0]= getVbatAdc(cur_detect_pos);
				
				if(gBatVoltArray[0] >= BAT_MIN_VOLT_OPEN)
				{
					if(gBatVoltArray[0] >= BAT_MAX_VOLT_OPEN)
					{
						gBatStateBuf[2] |= BAT_TYPE_ERROR;
					}
					else
					{
						gBatStateBuf[2] |= (CHARGE_STATE_FAST|BAT_DETECT_BIT);
					}
					gBatNumNow++;	
					gBatNowBuf[gBatNumNow] = 2;
				}
			}
		}
		chargeHandler();
		ledHandler();
		__Asm CWDT;

	}
		
	
	
}
