#if 1
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

u16 gBatVoltArray[4][1]={{0},{0},{0},{0}};

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
u32 gChargingIntervalTick[4] = {0,0,0,0};
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
			gBatVoltArray[batNum-1][0] = getVbatAdc(batNum);
			if(gBatVoltArray[batNum-1][0] >= CHARGING_PRE_END_VOLT )
			{
				if(batNum<3)
					gBatVoltArray[batNum-1][0] = getAverage(CHANNEL_TEMP_1);
				else
					gBatVoltArray[batNum-1][0] = getAverage(CHANNEL_TEMP_2);
				if(gBatVoltArray[batNum-1][0]>ADC_TEMP_MAX && gBatVoltArray[batNum-1][0] <ADC_TEMP_MIN)
				{
					gChargingTimeTick[batNum-1] = 0;
					gBatStateBuf[batNum] &= ~CHARGE_STATE_PRE;
					gBatStateBuf[batNum] |= CHARGE_STATE_FAST;
				}
				else
				{
					gChargingTimeTick[batNum-1] = 0;
					gBatStateBuf[batNum] &= ~CHARGE_STATE_PRE;
					gBatStateBuf[batNum] |= CHARGE_STATE_TRICK;
				}
			}
		}
}

void FastCharge(u8 batNum)
{
	u16 tempV,tempT;
		
	if(getDiffTickFromNow(gChargingTimeTick[batNum-1]) > BAT_START_DV_CHECK)  //hod-off time, in this period, we do NOT detect -dv
	{
		if(getDiffTickFromNow(gChargingIntervalTick[batNum-1]) > BAT_DV_CHECK_INTERVAL)
		{
			gChargingIntervalTick[batNum-1] = getSysTick();
			if(gChargingIntervalTick[batNum-1] == 0)
			{
				GIE = 0;
				shortTick = 1;
				GIE = 1;
				gChargingIntervalTick[batNum-1] = 1;
			}

			tempV = getVbatAdc(batNum);
			if(batNum <3)		
				tempT = getAverage(CHANNEL_TEMP_1);
			else
				tempT = getAverage(CHANNEL_TEMP_2);
			if(tempV >= CHARGING_FAST_END_VOLT || getDiffTickFromNow(gChargingTimeTick[batNum-1]) > BAT_CHARGING_FAST_MAX_TIME || tempT < ADC_TEMP_MAX)
			{
				gBatStateBuf[batNum] &= ~CHARGE_STATE_FAST;
				gBatStateBuf[batNum] |= CHARGE_STATE_TRICK;
				gBatStateBuf[batNum] |= CHARGE_STATE_FULL;
				gChargingTimeTick[batNum-1] = 0;
				gChargingIntervalTick[batNum-1] = 0;
				return;
			}
			if(tempV > gBatVoltArray[batNum-1][0])
				gBatVoltArray[batNum-1][0] = tempV;
			else
			{
				if((tempV - gBatVoltArray[batNum-1][0]) > ADC_DV_VOLT)
				{
					gBatStateBuf[batNum] &= ~CHARGE_STATE_FAST;
					gBatStateBuf[batNum] |= CHARGE_STATE_TRICK;
					gBatStateBuf[batNum] |= CHARGE_STATE_FULL;
					gChargingTimeTick[batNum-1] = 0;
					gChargingIntervalTick[batNum-1] = 0;
				}
			}
		}
	}

}

/*
void SupCharge(u8 batLabel)
{

}
*/
void TrickCharge(u8 batNum)
{
	if(gBatStateBuf[batNum] & CHARGE_STATE_FULL)
	{
	}
	else
	{
		if(batNum < 3)
			gBatVoltArray[batNum-1][0] = getAverage(CHANNEL_TEMP_1);
		else
			gBatVoltArray[batNum-1][0] = getAverage(CHANNEL_TEMP_2);
		if(gBatVoltArray[batNum-1][0]>ADC_TEMP_MAX && gBatVoltArray[batNum-1][0] <ADC_TEMP_MIN)
		{
			gBatStateBuf[batNum] &= ~CHARGE_STATE_TRICK;
			gBatStateBuf[batNum] |= CHARGE_STATE_FAST;
			gBatStateBuf[batNum] &= ~CHARGE_STATE_FULL;
			gChargingTimeTick[batNum-1] = 0;
			gChargingIntervalTick[batNum-1] = 0;	
		}
	}
}


void removeBat(u8 toChangeBatPos)
{
	u8 i;

	gChargeSkipCount[gBatNowBuf[toChangeBatPos] -1 ] = 0;
	gChargingTimeTick[gBatNowBuf[toChangeBatPos] - 1] = 0;
	gChargingIntervalTick[gBatNowBuf[toChangeBatPos]-1] = 0;
	gBatStateBuf[gBatNowBuf[toChangeBatPos]] =0;

	LED_OFF(gBatNowBuf[toChangeBatPos]);
	PB &= 0xF0;   //close current pwm channel
	
	for(i=toChangeBatPos;i<gBatNumNow;i++)
	{
		gBatNowBuf[i] = gBatNowBuf[i+1];
	}
	gBatNowBuf[gBatNumNow] = 0;
	gBatNumNow--;
	gIsChargingBatPos = 1;
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
		switch(gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] & 0x38)	
		{
			case CHARGE_STATE_FAST:
					skipCount = FAST_SKIP_COUNT; break; //BatFastCharge(ticknow%4);break;
		//	case CHARGE_STATE_SUP:
		//			skipCount = SUP_SKIP_COUNT;break;// BatSupCharge(ticknow%4);break;
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
				gChargeSkipCount[gBatNowBuf[gIsChargingBatPos]-1] = 0;
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
				else if(tempV>BAT_MAX_VOLT_CLOSE || gChargeCurrent <3/*|| (gChargeCurrent<<3) <= ( tempV-gBatVoltArray[gBatNowBuf[gIsChargingBatPos]-1][0])*/)  //
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
					if(gBatNowBuf[gIsChargingBatPos]<3)
						tempV = getAverage(CHANNEL_TEMP_1);
					else
						tempV = getAverage(CHANNEL_TEMP_2);
					if(tempV < ADC_TEMP_MAX || tempV > ADC_TEMP_MIN)
						gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] |= CHARGE_STATE_TRICK;
					else
					{
						if(gBatVoltArray[gBatNowBuf[gIsChargingBatPos]-1][0] < CHARGING_PRE_END_VOLT)
							gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] |= CHARGE_STATE_PRE;
						else if(gBatVoltArray[gBatNowBuf[gIsChargingBatPos]-1][0] < CHARGING_FAST_END_VOLT)
							gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] |= CHARGE_STATE_FAST;
						else
							gBatStateBuf[gBatNowBuf[gIsChargingBatPos]] |= CHARGE_STATE_TRICK;
					}
						
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
		else if(gIsChargingBatPos !=0 && (getDiffTickFromNow(ChargingTimeTick) > BAT_CHARGING_TEST_TIME))
		{ 
			tempV = getVbatAdc(gBatNowBuf[gIsChargingBatPos]);
			/*
			if(gChargeCurrent<44)    // <1800mA
			{
				LED_OFF(3);LED_OFF(4);LED_ON(1);
			}
			else if(gChargeCurrent<54)  //<2200mA
			{
				LED_OFF(1);LED_OFF(3);LED_ON(4);
			}
			else
			{
				LED_OFF(1);LED_OFF(4);LED_ON(3);
			}
			*/
			if(tempV < BAT_MIN_VOLT_OPEN || ((PB & (1<<(gBatNowBuf[gIsChargingBatPos]-1))) &&gChargeCurrent  < 3))
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
				//	case CHARGE_STATE_SUP:
				//			 SupCharge(gBatNowBuf[gIsChargingBatPos]);break;
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
	
	//IO led1~4  �����
	//PAT 	&=	0xFC; 
	PAT &= 0xC4;
	PCT	&=	0xFC;
	PA	|= 0x3;
	PC   |= 0x3;

	//pwm
	//PBT &= 0xF0;
	PBT &= 0xD0;
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


	//ADC	
	//ANSL = 0x03;   //AIN2~AIN6
	//ANSH = 0x9F;	 //AIN12 AIN13
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
		if(gBatStateBuf[cur_detect_pos] & HAS_BATTERY)
		{
			//getVbatAdc(cur_detect_pos);
		}
		else
		{
			if((gBatStateBuf[cur_detect_pos] & BAT_DETECT_BIT) == 0)
			{	
			//	gBatVoltArray[cur_detect_pos-1][0]=  getVbatAdc(cur_detect_pos);	//gBatVoltArray[cur_detect_pos-1][0]= getVbatAdc(cur_detect_pos);
				
				if(gBatVoltArray[cur_detect_pos-1][0] >= BAT_MIN_VOLT_33_OPEN)
				{
					//if(gBatVoltArray[cur_detect_pos-1][0] >= BAT_MAX_VOLT_OPEN)
					//{
						gBatStateBuf[cur_detect_pos] |= BAT_TYPE_ERROR;
					//}
					//else
					//{
					//	gBatStateBuf[cur_detect_pos] |= (CHARGE_STATE_FAST|BAT_DETECT_BIT);
					//}
					//gBatNumNow++;	
					//gBatNowBuf[gBatNumNow] = cur_detect_pos;
				}
			}
		}
	//	chargeHandler();
	//	ledHandler();

	PA4 = 1;
	PA3=1;
	PB5 =1;
	PA5 = 1;
	LED_ON(1);
	delay_ms(1000);
	PA4 = 0;
	PA3 = 0;
	PB5 = 0;
	PA5 = 0;
	LED_OFF(1);
	delay_ms(1000);

/*
		cur_detect_pos++;
		if(cur_detect_pos > 4)
			cur_detect_pos=1;
*/
/*
			send(gBatVoltArray[0][0]);
			send(gBatVoltArray[1][0]);
			send(gBatVoltArray[2][0]);
			send(gBatVoltArray[3][0]);
			send(0xAAAA);
			*/

	//	delay_ms(100);
		__Asm CWDT;

	}
		
	
	
}

#endif
#if 0
#include <hic.h>
#include <string.h>

unsigned char g_adc_value_h;
unsigned char g_adc_value_l;
unsigned int ADCRH1;    //ADת�����
unsigned int ADCRH2;    //ADת�����

#define u16 unsigned int
#define u8 unsigned char

const char digits[] = "0123456789abcdef";

void send(u16 sData)
{
	signed char i;

	for(i=3; i>=0; i--)
	{
        while(!TRMT);           //�ȴ�������λ�Ĵ���TXR��
        TXB  =  digits[(sData>>(i*4))&0xf];   //�����ַ���
        while(!TXIF);           //�ȴ������жϱ�־λ 
        TXIF = 0;               //�巢���жϱ�־λ 
	}

	  while(!TRMT);           //�ȴ�������λ�Ĵ���TXR��
        TXB  =  ' ';   //�����ַ���
        while(!TXIF);           //�ȴ������жϱ�־λ 
        TXIF = 0;               //�巢���жϱ�־λ 

}

void sendStr(char str[])
{
	u8 i;

	for(i=0;i<5;i++)
	{
        while(!TRMT);           //�ȴ�������λ�Ĵ���TXR��
        TXB  =  str[i];   //�����ַ���
        while(!TXIF);           //�ȴ������жϱ�־λ 
        TXIF = 0;               //�巢���жϱ�־λ 	
	}

	while(!TRMT);           //�ȴ�������λ�Ĵ���TXR��
        TXB  =  ' ';   //�����ַ���
        while(!TXIF);           //�ȴ������жϱ�־λ 
        TXIF = 0;               //�巢���жϱ�־λ 
}


void Delay_nus(unsigned int nus)
{
    unsigned int i;
    for(i=0;i<nus;i++);
}

/***********************************************************
*�� �� ����  void init_io(void)  
*����������  I/O��ʼ������           
*���������  ��                                
*��������ֵ����                   
*���ú�����  
*���������  
*�޶���Ҫ��  2014.01.07 V1.0                         
***********************************************************/  
void init_io(void)
{
    ANSL = 0x00;                  //AIN0~6����Ϊģ��˿�
    ANSH = 0x00;                  //AIN7~13����Ϊģ��˿� 
}

/***********************************************************
*�� �� ����  void  init_adc(void)   
*����������  adc��ʼ������           
*���������  ��                                
*��������ֵ����                   
*���ú�����  
*���������  
*�޶���Ҫ��  2014.01.07 V1.0                         
***********************************************************/ 
void init_adc(void)
{
    ANSL = 0x00;                  //AIN0~6����Ϊģ��˿�0x5f ain5
    ANSH = 0x00;                  //AIN7~13����Ϊģ��˿�  0xff 
    ADCTST = 0x00;                //ADת���ٶ�Ϊ����
    ADCCL = 0x05;                 //����A/Dת����,Ӳ�����Ʋ���
    ADCCH = 0x48;                 //ADC����ʱ��Ϊ8��ADCʱ�ӣ�����ֵ��λ���룬A/Dת��ʱ��Ƶ��ΪFosc/16���ο���ѹΪVDD
}

/***********************************************************
*�� �� ����  void convert_adc(void)  
*����������  adcת������           
*���������  ��                                
*��������ֵ����                    
*���ú�����  
*���������  
*�޶���Ҫ��  2014.01.07 V1.0                         
***********************************************************/ 
void convert_adc(void)
{
    ADTRG = 1;
    while(ADTRG);
    ADIF = 0;
    g_adc_value_h = ADCRH;
    g_adc_value_l = ADCRL;
    ADEN = 0;                       //�ر�AD����
}

/***********************************************************
*�� �� ����  void clear_ram(void)    
*����������  ram������ȫ������           
*���������  ��                                
*��������ֵ����                   
*���ú�����  MCU��ʼ��ʱ����
*���������  
*�޶���Ҫ��  2014.01.07 V1.0                         
***********************************************************/  
void clear_ram(void) 
{
    __asm 
    { 
	     CLR   IAAL;
          CLR   IAAH;
	     CLR   IAD;
	     INC   IAAL,1;
	     JBS   PSW,C
	     GOTO  $-3;
	     INC   IAAH,1;
	     JBS   IAAH,2
	     GOTO  $-6;
    } 
}

/***********************************************************
*�� �� ����  void  init_mcu(void)   
*����������  MCU��ʼ������������ʱ����            
*���������  ��                                
*��������ֵ����                   
*���ú�����  
*���������  
*�޶���Ҫ��  2014.01.07 V1.0                         
***********************************************************/  
void  init_mcu(void)
{
	clear_ram();  		         //��ʼ��RAM
    INTG = 0;                    //�ж�ȫ�ּĴ���
    INTE2 = 0;                   //�ж�ʹ�ܼĴ���2
    INTF2 = 0;                   //�жϱ�־�Ĵ���2
    init_io();                   //I/O�ڳ�ʼ��
    init_adc();                  //ADC��ʼ��


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
}

/***********************************************************
*�� �� ����  void main(void)    
*����������  ������        
*���������  ��                                
*��������ֵ����   
*���ú�����  
*���������  
*�޶���Ҫ��  2014.01.07 V1.0                                       
***********************************************************/  
void main(void)
{
    init_mcu();
    while(1)
    {
		ADCCL = 0x45;	              //ʹ��ADCת������ѡ��ͨ��4
    convert_adc();
		ADCRH1 =((unsigned int)g_adc_value_h) << 4;
		ADCRH1+=( g_adc_value_l >> 4);
		send(ADCRH1);
    Delay_nus(8000);
    }
}

#endif

