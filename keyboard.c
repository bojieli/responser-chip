#include "avr/io.h"
#include "util/delay.h"

#define uchar unsigned char
#define uint unsigned int

/**************************键盘扫描***************************/
//10ms中断扫描
#define NO_KEY     0xf7  //无按键按下
#define KEY_STATE0 0     //判断按键按下
#define KEY_STATE1 1     //确定按键按下
#define KEY_STATE2 2     //按键释放状态
 

/*****************************************************/
uchar key_scan(void)
{
	static uchar key_state=KEY_STATE0;
	uchar key_value=0xff;
	uchar key_temp;
	uchar key1,key2;
	
	DDRD&=~(0xf0);
	PORTD|=(0xf0);//行上拉输入
	DDRC|=0x07;
	PORTC&=~(0x07);//列输出0
	_delay_us(1);
	key1=(PIND&0xf0);//确定哪一行按键按下

	DDRC&=~(0x07);
	PORTC|=(0x07);//列上拉输入
	DDRD|=(0xf0);
	PORTD&=~(0xf0);//行输出0
	_delay_us(1);
	key2=(PINC&0x07);//确定哪一列按键按下
	DDRD&=~0xf0;//行输入
	
	key_temp=key1|key2;//确定按键位置
	switch(key_state)//检测当前状态
	{
		case KEY_STATE0:
		{
			if(key_temp!=NO_KEY)
			{
				key_state=KEY_STATE1;//转换状态
			}
			break;
		}
		case KEY_STATE1:
		{
			if(key_temp==NO_KEY)
			{
				key_state=KEY_STATE0;
			}
			else
			{
				switch(key_temp)//当确定按键按下后，列举按键情况
				{
					case 0xe6:key_value=1;break;
					case 0xe5:key_value=2;break;
					case 0xe3:key_value=3;break;
					case 0xd6:key_value=4;break;
					case 0xd5:key_value=5;break;
					case 0xd3:key_value=6;break;
					case 0xb6:key_value=7;break;
					case 0xb5:key_value=8;break;
					case 0xb3:key_value=9;break;
					case 0x76:key_value=10;break;
					case 0x75:key_value=0;break;
					case 0x73:key_value=11;break;
				}
				key_state=KEY_STATE2;
			}
			break;
		}
		case KEY_STATE2:
		{
			if(key_temp==NO_KEY)
			{
				key_state=KEY_STATE0;
			}
			break;
		}
	}
	return key_value;
}
