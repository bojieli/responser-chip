#include "avr/io.h"
#include "util/delay.h"
#include "avr/interrupt.h"
#include "avr/eeprom.h"

#include "OLED12864.h"
#include "keyboard.h"
#include "nRF24L01.h"

#define uchar unsigned char
#define uint unsigned int

volatile uchar system_time=0;
volatile uchar system_counter=0;

void POWER_OFF() {
	PORTC&=~(1<<PC3);
	display_off;
}
volatile uchar adcL,adcH;

#define CHARGE_R (PIND&(1<<PD0))
#define VIN_R    (PIND&(1<<PD1))

#define NO_CHARGE   0
#define CHARGING    1
#define CHARGE_OVER 2
uchar charge_state=0;

volatile uchar key_flag=0;//按键扫描标志
volatile uchar POWER_flag=0;//ADC测量标志
volatile uchar dis_flag=0;//系统时钟显示标志
volatile uchar sending_state=0;//射频发送状态

uint power_NEW=500;
uint power_OLD=500;
uchar power_update_en=0;

uchar stu_num[10]={0};//学号
uchar ID[4]={0x00,0x00,0x00,0x00};//编号
uchar station[3]={0x00,0x00,0x00};//基站 即频道 1~125
volatile uchar mode=0xff;//模式 开机默认输入基站模式
#define MODE_BUFORE   0
#define MODE_ANSWER   1
#define MODE_REGISTER 2
volatile uchar xpos=0;//光标位置
volatile uchar ypos=6;
volatile uchar display_cursor_en=1;//光标显示允许
volatile uchar mode_change=0;//更换模式
volatile uint sleep_counter=0;//自动休眠计数器
volatile uchar sleep_state=1;//休眠状态 0休眠
/************************nRF24L01*************************/
volatile uchar rx_buf[9]={0};
volatile uchar tx_buf[9]={0x00,0x00,0x00,0x00,0xff,0x00,0x00,0x00,0x00};//前4字节为ID
void SPI_init(void)
{
	SPCR|=(1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);
	SPSR=0x00;
}
void INT0_init(void)
{
	MCUCR|=(1<<ISC01);
	GICR|=(1<<INT0);
}
ISR(INT0_vect)//数据接收中断
{
	if(nRF24L01_RxPacket(rx_buf))//接收到数据
	{
		if((rx_buf[4]&0xf0)==0xe0)//切换模式命令
		{
			switch(rx_buf[4])
			{
				case 0xe0://切换到课前模式
				{
					//if(mode!=MODE_BEFORE)
					break;
				}
				case 0xe1://切换到答题模式
				{
					break;
				}
				case 0xe2://切换到注册模式
				{
					break;
				}
			}
		}
		else//确认信号
		{
		
		}
	}
	
}
/************************ADC*************************/
void ADC_init(void)
{
	ADMUX|=(1<<REFS1)|(1<<REFS0)|0x07;//内部参考电压源2.56V，输出数据右对齐，PC7输入通道	
	ADCSRA=(1<<ADEN)|(1<<ADIE);//|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}
ISR(ADC_vect)
{
	adcL=ADCL;
	adcH=ADCH;
}
inline uint power_measure(void)
{
	uchar i;
	uint power=0;
	uchar measure;
	for(i=0;i<8;i++)
	{
		ADCSRA|=(1<<ADSC);//开始转换
		while(ADCSRA&(1<<ADSC));
		power+=((uint)adcL+(((uint)adcH)*256));
	}
	power>>=3;
	power+=20;
	power_NEW=power;
	if (!power_update_en && power_NEW < power_OLD+2 && power_NEW > power_OLD-2) 
	{
		measure = 10;//不更新电量值
		goto end;
	}
	uint powers[] = {
				// 自动关机
		783,	// 0格1格闪烁
		803, 814, 818, 827, 835, 851, 866, 
		890		// 显示满格
	};
	for (measure=0; measure<sizeof(powers); measure++)
	{
		if (power <= powers[i])
			break;
	}
end:
	power_update_en=0;
	power_OLD=power_NEW;
	return measure;
}
/************************keyboard*************************/
void timer0_init(void)//定时10ms
{
	TCCR0|=(1<<CS02)|(1<<CS00);//1024分频
	TCNT0=177;
	TIMSK|=(1<<TOIE0);//timer0溢出中断使能
}
ISR(TIMER0_OVF_vect)
{
	TCNT0=178;
	system_time++;
	key_flag=1;
	if(system_time==5)//每50ms一次
	{
		system_time=0;
		system_counter++;
		dis_flag=1;
	}
	if(system_counter==10)
	{
		POWER_flag=1;
	}
	if(system_counter==20)
	{
		system_counter=0;
		sleep_counter++;//每1s加1
		if(sleep_counter==120)//自动休眠
		{
			display_off;
			POWER_DOWN();//nNF24L01休眠
			sleep_state=0;
		}
		else if(sleep_counter==2400)//自动关机
		{
			POWER_OFF();//关机
		}
	}
}
/**********************初始化***************************/
void port_init(void)
{
	DDRC|=(1<<PC3);	PORTC|=(1<<PC3);//POWER_ON
	PORTD|=(1<<PD3);//INT1 ON-OFF 上拉输入
	PORTD|=(1<<PD0);//charge 上拉输入
	PORTD|=(1<<PD1);//VIN 上拉输入
}
void INT1_init(void)
{
	MCUCR|=(1<<ISC11);//下降沿中断
	GICR|=(1<<INT1);
}
ISR(INT1_vect)
{
	_delay_ms(500);
	if(!(PIND&(1<<PD3)) && CHARGE_R)//按下关机键且不在充电状态
	{
		POWER_OFF();//关机
	}
}

void system_init(void)
{
	port_init();//power_on
	OLED_init();
	INT1_init();//关机键
	ADC_init();
	
	INT0_init();
	SPI_init();
	//init_NRF24L01();
	//
	timer0_init();
}
int main(void)
{
	uchar i=0,j=0;//显示用循环变量
	uchar ms;
	uchar station_num;//计算基站编号用

	uchar power=5;//显示电量用
	uchar charging_temp=0;//显示充电状态用
	uchar charge_state_old=0;//
	uchar key_val=0xff;//键值
	uchar answer_temp=0;//存放答案
	uchar send_succeed=0;//发送成功标志
	system_init();
	
	display_picture();
	display_on;
	_delay_ms(2000);
	display_off;
	display_clear();
	
	sei();
	eeprom_read_block(stu_num,0,10);
	eeprom_read_block(station,10,3);
	
	/*display_chinese_char(0,0,0);//编
	display_chinese_char(16,0,1);//号
	display_char(32,0,16);//:
	for(i=0;i<4;i++)display_char(48+(i<<3),0,ID[i]);
	*/
	display_chinese_char(0,0,17);//欢
	display_chinese_char(16,0,18);//迎
	display_chinese_char(32,0,19);//使
	display_chinese_char(48,0,20);//用

	display_chinese_char(0,3,2);//学
	display_chinese_char(16,3,1);//号
	display_char(32,3,16);//:
	if(stu_num[0]==0xff)//显示未注册
	{
		display_chinese_char(48,3,14);
		display_chinese_char(64,3,15);
		display_chinese_char(80,3,16);
	}
	else//显示学号
	{
		for(i=0;i<10;i++)
		{
			if(stu_num[i]!=0xff)
			display_char(48+(i<<3),3,stu_num[i]);
		}
	}
	display_chinese_char(0,6,5);//基
	display_chinese_char(16,6,6);//站
	display_char(32,6,16);//:
	if(station[0]!=0xff)
	{
		for(xpos=0;xpos<3;xpos++)
		{
			display_char(48+(xpos<<3),6,station[xpos]);
		}
	}
	display_on;
	while(1)
	{	
		if(POWER_flag)//进行电量测量
		{
			power=power_measure();//测电压
			if(CHARGE_R)
			{
				charge_state=NO_CHARGE;
				if(charge_state_old==CHARGING)
				{
					power_update_en=1;
				}
			}
			else
			{
				charge_state=CHARGING;//充电中

			}
			charge_state_old=charge_state;
			POWER_flag=0;
		}
		if(dis_flag)//更新显示
		{
			if(display_cursor_en&&(xpos!=10))
			{
				if(system_counter==10)display_char(48+(xpos<<3),ypos,20);
				else if(system_counter==0) display_char(48+(xpos<<3),ypos,19);
			}
			switch(charge_state)
			{
				case NO_CHARGE:
				{
					if (power == 0)
						POWER_OFF();
					else if (power != 10)
						display_power(power-1);
					break;
				}
				case CHARGING:
				{
					display_power((charging_temp++)>>2);
					if(charging_temp==36)charging_temp=0;
					power_update_en=1;
					break;
				}
			}
			if(mode_change)//改变模式
			{
				display_off;
				display_clear();
				if(mode_change==1)
				{
					display_chinese_char(0,3,3);
					display_chinese_char(16,3,4);
					mode=1;
					answer_temp=0;
				}
				else if(mode_change==2)
				{
					display_chinese_char(0,3,2);
					display_chinese_char(16,3,1);
					mode=2;
				}
				display_char(32,3,16);
				display_chinese_char(0,0,5);
				display_chinese_char(16,0,6);
				display_char(32,0,16);
				for(xpos=0;xpos<3;xpos++)
				{
					display_char(48+(xpos<<3),0,station[xpos]);
				}
				xpos=0;
				ypos=3;
				display_cursor_en=1;
				mode_change=0;
				display_on;
			}
			dis_flag=0;
		}
		if(mode==0)//输入基站编号模式
		{
			if(key_flag)
			{
				key_val=key_scan();
				if(key_val!=0xff)
				{
					sleep_counter=0;
					if(sleep_state==0)
					{
						display_on;//唤醒
						sleep_state=1;
						//SetRX_Mode();
					}
					else
					{
						if(key_val<10)
						{
							if(xpos<3)
							{
								station[xpos]=key_val;
								if(station[0]>1)station[0]=1;
								else if(station[0]==1)
								{
									if(station[1]>2)station[1]=2;
									else if(station[1]==2)
									{
										if(station[2]>4)station[2]=4;
									}
								}
								display_char(48+(xpos<<3),6,station[xpos]);
								xpos++;
							}
						}
						else if(key_val==10)//确定
						{
							if(xpos==3)
							{
								display_off;
								eeprom_write_block(station,10,3);
								display_clear();
								station_num=station[0]*100+station[1]*10+station[2];
								init_NRF24L01(station_num);//初始化射频模块 输入基站号
							
								display_chinese_char(0,0,5);
								display_chinese_char(16,0,6);
								display_char(32,0,16);
								for(xpos=0;xpos<3;xpos++)
								{
									display_char(48+(xpos<<3),0,station[xpos]);
								}
								xpos=0;//光标位置清零
								ypos=3;
								display_chinese_char(0,3,3);
								display_chinese_char(16,3,4);
								display_char(32,3,16);
								display_on;
								mode=1;//进入答题模式
							}
						}
						else if(key_val==11)//退格	
						{
							if(xpos>0)
							{
								xpos--;
								display_char(48+(xpos<<3),6,19);
								display_char(56+(xpos<<3),6,19);
							}
						}
					}				
				}
				key_flag=0;
			}
		}
		else if(mode==1)//答题模式
		{			
			if(key_flag)
			{
				key_val=key_scan();
				if(key_val!=0xff)//有按键按下
				{
					sleep_counter=0;
					if(sleep_state==0)
					{
						display_on;//唤醒
						sleep_state=1;
						SetRX_Mode();
					}
					else
					{
						if((key_val<=6)&&(key_val!=0))//按了A~F
						{
							if(!display_cursor_en)
							{
								display_cursor_en=1;
								for(i=0;i<xpos;i++)display_char(48+(i<<3),3,19);
								for(i=0;i<8;i++)display_char(32+(i<<3),6,19);//清空显示
								answer_temp=0;
								xpos=0;
							}
							if(xpos<=6)
							{
								if(!(answer_temp&(1<<key_val)))//如果没有选择过改答案
								{
									answer_temp|=(1<<key_val);
									display_char(48+(xpos<<3),3,key_val+9);//显示所选答案
									xpos++;
								}
							}
						}
						else if(key_val==11)//退格
						{
							if(display_cursor_en)
							{	
								if(xpos>0)
								{
									xpos--;
									answer_temp&=(1<<xpos);
									display_char(48+(xpos<<3),3,19);
									display_char(56+(xpos<<3),3,19);
								}
							}
						}
						else if(key_val==10)//确定键
						{
							if(display_cursor_en)
							{
								if(xpos)//有答案
								{
									tx_buf[5]=answer_temp>>1;
									for(i=0;i<10;i++)
									{
										nRF24L01_TxPacket(tx_buf);//发送答案
										while(!sending_state);
										if(sending_state==2)//发送成功
										{
										
											break;
										}
										ms=100+(((uint)(TCNT0+((uint)system_time)*178))<<4);
										for(j=0;j<ms;j++)
										{
											_delay_us(1);
										}
									}
									send_succeed=sending_state;
									sending_state=0;

									display_chinese_char(32,6,7);//发
									display_chinese_char(48,6,8);//送
									if(send_succeed==2)//发送成功
									{
										display_chinese_char(64,6,10);//成
										display_chinese_char(80,6,11);//功
										display_char(112,6,19);
										display_char(120,6,19);
									}
									else
									{
										display_chinese_char(64,6,12);//失
										display_chinese_char(80,6,13);//败
										display_char(112,6,19);
										display_char(120,6,19);
									}
									display_cursor_en=0;//关闭光标显示
									display_char(48+(xpos<<3),3,19);
								}
							}
						}
					}
				}
				key_flag=0;
			}
		}
		else if(mode==2)//注册模式
		{
			if(key_flag)
			{
				key_val=key_scan();
				if(key_val!=0xff)//有按键按下
				{
					sleep_counter=0;
					if(sleep_state==0)
					{
						display_on;//唤醒
						sleep_state=1;
						SetRX_Mode();
					}
					else
					{
						if(key_val<10)//按了0~9
						{
							if(!display_cursor_en)
							{
								display_cursor_en=1;
								for(i=0;i<xpos;i++)display_char(48+(i<<3),3,19);
								for(i=0;i<8;i++)display_char(32+(i<<3),6,19);//清空显示
								xpos=0;
							}
							if(xpos<=9)
							{
								stu_num[xpos]=key_val;
								display_char(48+(xpos<<3),3,key_val);//显示所选答案
								xpos++;
							}
						}
						else if(key_val==11)//退格
						{
							if(display_cursor_en)
							{	
								if((xpos>0)&&(xpos!=10))
								{
									xpos--;
									stu_num[xpos]=0;
									display_char(48+(xpos<<3),3,19);
									display_char(56+(xpos<<3),3,19);
								}
								else if(xpos==10)
								{
									xpos--;
									stu_num[xpos]=0;
									display_char(120,3,19);
									display_char(120,3,19);
								}
							}
						}
						else if(key_val==10)//确定键
						{
							if(display_cursor_en)
							{
								if(xpos==10)//输完了学号
								{
									tx_buf[4]=(stu_num[0]<<4)+stu_num[1];
									tx_buf[5]=(stu_num[2]<<4)+stu_num[3];
									tx_buf[6]=(stu_num[4]<<4)+stu_num[5];
									tx_buf[7]=(stu_num[6]<<4)+stu_num[7];
									tx_buf[8]=(stu_num[8]<<4)+stu_num[9];

									for(i=0;i<10;i++)
									{
										nRF24L01_TxPacket(tx_buf);//发送学号
										while(!sending_state);
										if(sending_state==2)//发送成功
										{
										
											break;
										}
										_delay_us(100);
									}
									send_succeed=sending_state;
									sending_state=0;

									display_chinese_char(32,6,7);//发
									display_chinese_char(48,6,8);//送
									if(send_succeed==2)//发送成功
									{
										display_chinese_char(64,6,10);//成
										display_chinese_char(80,6,11);//功
										display_char(112,6,19);
										display_char(120,6,19);
										eeprom_write_block(stu_num,0,10);//将学号存入eeprom
									}
									else
									{
										display_chinese_char(64,6,12);//失
										display_chinese_char(80,6,13);//败
										display_char(112,6,19);
										display_char(120,6,19);
									}
									display_cursor_en=0;//关闭光标显示
								
								}
							}
						}
					}
				}
					
				key_flag=0;	
			}
		}
		
	}
}
