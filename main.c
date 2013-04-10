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

volatile uchar key_flag=0;//����ɨ���־
volatile uchar POWER_flag=0;//ADC������־
volatile uchar dis_flag=0;//ϵͳʱ����ʾ��־
volatile uchar sending_state=0;//��Ƶ����״̬

uint power_NEW=500;
uint power_OLD=500;
uchar power_update_en=0;

uchar stu_num[10]={0};//ѧ��
uchar ID[4]={0x00,0x00,0x00,0x00};//���
uchar station[3]={0x00,0x00,0x00};//��վ ��Ƶ�� 1~125
volatile uchar mode=0xff;//ģʽ ����Ĭ�������վģʽ
#define MODE_BUFORE   0
#define MODE_ANSWER   1
#define MODE_REGISTER 2
volatile uchar xpos=0;//���λ��
volatile uchar ypos=6;
volatile uchar display_cursor_en=1;//�����ʾ����
volatile uchar mode_change=0;//����ģʽ
volatile uint sleep_counter=0;//�Զ����߼�����
volatile uchar sleep_state=1;//����״̬ 0����
/************************nRF24L01*************************/
volatile uchar rx_buf[9]={0};
volatile uchar tx_buf[9]={0x00,0x00,0x00,0x00,0xff,0x00,0x00,0x00,0x00};//ǰ4�ֽ�ΪID
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
ISR(INT0_vect)//���ݽ����ж�
{
	if(nRF24L01_RxPacket(rx_buf))//���յ�����
	{
		if((rx_buf[4]&0xf0)==0xe0)//�л�ģʽ����
		{
			switch(rx_buf[4])
			{
				case 0xe0://�л�����ǰģʽ
				{
					//if(mode!=MODE_BEFORE)
					break;
				}
				case 0xe1://�л�������ģʽ
				{
					break;
				}
				case 0xe2://�л���ע��ģʽ
				{
					break;
				}
			}
		}
		else//ȷ���ź�
		{
		
		}
	}
	
}
/************************ADC*************************/
void ADC_init(void)
{
	ADMUX|=(1<<REFS1)|(1<<REFS0)|0x07;//�ڲ��ο���ѹԴ2.56V����������Ҷ��룬PC7����ͨ��	
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
		ADCSRA|=(1<<ADSC);//��ʼת��
		while(ADCSRA&(1<<ADSC));
		power+=((uint)adcL+(((uint)adcH)*256));
	}
	power>>=3;
	power+=20;
	power_NEW=power;
	if (!power_update_en && power_NEW < power_OLD+2 && power_NEW > power_OLD-2) 
	{
		measure = 10;//�����µ���ֵ
		goto end;
	}
	uint powers[] = {
				// �Զ��ػ�
		783,	// 0��1����˸
		803, 814, 818, 827, 835, 851, 866, 
		890		// ��ʾ����
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
void timer0_init(void)//��ʱ10ms
{
	TCCR0|=(1<<CS02)|(1<<CS00);//1024��Ƶ
	TCNT0=177;
	TIMSK|=(1<<TOIE0);//timer0����ж�ʹ��
}
ISR(TIMER0_OVF_vect)
{
	TCNT0=178;
	system_time++;
	key_flag=1;
	if(system_time==5)//ÿ50msһ��
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
		sleep_counter++;//ÿ1s��1
		if(sleep_counter==120)//�Զ�����
		{
			display_off;
			POWER_DOWN();//nNF24L01����
			sleep_state=0;
		}
		else if(sleep_counter==2400)//�Զ��ػ�
		{
			POWER_OFF();//�ػ�
		}
	}
}
/**********************��ʼ��***************************/
void port_init(void)
{
	DDRC|=(1<<PC3);	PORTC|=(1<<PC3);//POWER_ON
	PORTD|=(1<<PD3);//INT1 ON-OFF ��������
	PORTD|=(1<<PD0);//charge ��������
	PORTD|=(1<<PD1);//VIN ��������
}
void INT1_init(void)
{
	MCUCR|=(1<<ISC11);//�½����ж�
	GICR|=(1<<INT1);
}
ISR(INT1_vect)
{
	_delay_ms(500);
	if(!(PIND&(1<<PD3)) && CHARGE_R)//���¹ػ����Ҳ��ڳ��״̬
	{
		POWER_OFF();//�ػ�
	}
}

void system_init(void)
{
	port_init();//power_on
	OLED_init();
	INT1_init();//�ػ���
	ADC_init();
	
	INT0_init();
	SPI_init();
	//init_NRF24L01();
	//
	timer0_init();
}
int main(void)
{
	uchar i=0,j=0;//��ʾ��ѭ������
	uchar ms;
	uchar station_num;//�����վ�����

	uchar power=5;//��ʾ������
	uchar charging_temp=0;//��ʾ���״̬��
	uchar charge_state_old=0;//
	uchar key_val=0xff;//��ֵ
	uchar answer_temp=0;//��Ŵ�
	uchar send_succeed=0;//���ͳɹ���־
	system_init();
	
	display_picture();
	display_on;
	_delay_ms(2000);
	display_off;
	display_clear();
	
	sei();
	eeprom_read_block(stu_num,0,10);
	eeprom_read_block(station,10,3);
	
	/*display_chinese_char(0,0,0);//��
	display_chinese_char(16,0,1);//��
	display_char(32,0,16);//:
	for(i=0;i<4;i++)display_char(48+(i<<3),0,ID[i]);
	*/
	display_chinese_char(0,0,17);//��
	display_chinese_char(16,0,18);//ӭ
	display_chinese_char(32,0,19);//ʹ
	display_chinese_char(48,0,20);//��

	display_chinese_char(0,3,2);//ѧ
	display_chinese_char(16,3,1);//��
	display_char(32,3,16);//:
	if(stu_num[0]==0xff)//��ʾδע��
	{
		display_chinese_char(48,3,14);
		display_chinese_char(64,3,15);
		display_chinese_char(80,3,16);
	}
	else//��ʾѧ��
	{
		for(i=0;i<10;i++)
		{
			if(stu_num[i]!=0xff)
			display_char(48+(i<<3),3,stu_num[i]);
		}
	}
	display_chinese_char(0,6,5);//��
	display_chinese_char(16,6,6);//վ
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
		if(POWER_flag)//���е�������
		{
			power=power_measure();//���ѹ
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
				charge_state=CHARGING;//�����

			}
			charge_state_old=charge_state;
			POWER_flag=0;
		}
		if(dis_flag)//������ʾ
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
			if(mode_change)//�ı�ģʽ
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
		if(mode==0)//�����վ���ģʽ
		{
			if(key_flag)
			{
				key_val=key_scan();
				if(key_val!=0xff)
				{
					sleep_counter=0;
					if(sleep_state==0)
					{
						display_on;//����
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
						else if(key_val==10)//ȷ��
						{
							if(xpos==3)
							{
								display_off;
								eeprom_write_block(station,10,3);
								display_clear();
								station_num=station[0]*100+station[1]*10+station[2];
								init_NRF24L01(station_num);//��ʼ����Ƶģ�� �����վ��
							
								display_chinese_char(0,0,5);
								display_chinese_char(16,0,6);
								display_char(32,0,16);
								for(xpos=0;xpos<3;xpos++)
								{
									display_char(48+(xpos<<3),0,station[xpos]);
								}
								xpos=0;//���λ������
								ypos=3;
								display_chinese_char(0,3,3);
								display_chinese_char(16,3,4);
								display_char(32,3,16);
								display_on;
								mode=1;//�������ģʽ
							}
						}
						else if(key_val==11)//�˸�	
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
		else if(mode==1)//����ģʽ
		{			
			if(key_flag)
			{
				key_val=key_scan();
				if(key_val!=0xff)//�а�������
				{
					sleep_counter=0;
					if(sleep_state==0)
					{
						display_on;//����
						sleep_state=1;
						SetRX_Mode();
					}
					else
					{
						if((key_val<=6)&&(key_val!=0))//����A~F
						{
							if(!display_cursor_en)
							{
								display_cursor_en=1;
								for(i=0;i<xpos;i++)display_char(48+(i<<3),3,19);
								for(i=0;i<8;i++)display_char(32+(i<<3),6,19);//�����ʾ
								answer_temp=0;
								xpos=0;
							}
							if(xpos<=6)
							{
								if(!(answer_temp&(1<<key_val)))//���û��ѡ����Ĵ�
								{
									answer_temp|=(1<<key_val);
									display_char(48+(xpos<<3),3,key_val+9);//��ʾ��ѡ��
									xpos++;
								}
							}
						}
						else if(key_val==11)//�˸�
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
						else if(key_val==10)//ȷ����
						{
							if(display_cursor_en)
							{
								if(xpos)//�д�
								{
									tx_buf[5]=answer_temp>>1;
									for(i=0;i<10;i++)
									{
										nRF24L01_TxPacket(tx_buf);//���ʹ�
										while(!sending_state);
										if(sending_state==2)//���ͳɹ�
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

									display_chinese_char(32,6,7);//��
									display_chinese_char(48,6,8);//��
									if(send_succeed==2)//���ͳɹ�
									{
										display_chinese_char(64,6,10);//��
										display_chinese_char(80,6,11);//��
										display_char(112,6,19);
										display_char(120,6,19);
									}
									else
									{
										display_chinese_char(64,6,12);//ʧ
										display_chinese_char(80,6,13);//��
										display_char(112,6,19);
										display_char(120,6,19);
									}
									display_cursor_en=0;//�رչ����ʾ
									display_char(48+(xpos<<3),3,19);
								}
							}
						}
					}
				}
				key_flag=0;
			}
		}
		else if(mode==2)//ע��ģʽ
		{
			if(key_flag)
			{
				key_val=key_scan();
				if(key_val!=0xff)//�а�������
				{
					sleep_counter=0;
					if(sleep_state==0)
					{
						display_on;//����
						sleep_state=1;
						SetRX_Mode();
					}
					else
					{
						if(key_val<10)//����0~9
						{
							if(!display_cursor_en)
							{
								display_cursor_en=1;
								for(i=0;i<xpos;i++)display_char(48+(i<<3),3,19);
								for(i=0;i<8;i++)display_char(32+(i<<3),6,19);//�����ʾ
								xpos=0;
							}
							if(xpos<=9)
							{
								stu_num[xpos]=key_val;
								display_char(48+(xpos<<3),3,key_val);//��ʾ��ѡ��
								xpos++;
							}
						}
						else if(key_val==11)//�˸�
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
						else if(key_val==10)//ȷ����
						{
							if(display_cursor_en)
							{
								if(xpos==10)//������ѧ��
								{
									tx_buf[4]=(stu_num[0]<<4)+stu_num[1];
									tx_buf[5]=(stu_num[2]<<4)+stu_num[3];
									tx_buf[6]=(stu_num[4]<<4)+stu_num[5];
									tx_buf[7]=(stu_num[6]<<4)+stu_num[7];
									tx_buf[8]=(stu_num[8]<<4)+stu_num[9];

									for(i=0;i<10;i++)
									{
										nRF24L01_TxPacket(tx_buf);//����ѧ��
										while(!sending_state);
										if(sending_state==2)//���ͳɹ�
										{
										
											break;
										}
										_delay_us(100);
									}
									send_succeed=sending_state;
									sending_state=0;

									display_chinese_char(32,6,7);//��
									display_chinese_char(48,6,8);//��
									if(send_succeed==2)//���ͳɹ�
									{
										display_chinese_char(64,6,10);//��
										display_chinese_char(80,6,11);//��
										display_char(112,6,19);
										display_char(120,6,19);
										eeprom_write_block(stu_num,0,10);//��ѧ�Ŵ���eeprom
									}
									else
									{
										display_chinese_char(64,6,12);//ʧ
										display_chinese_char(80,6,13);//��
										display_char(112,6,19);
										display_char(120,6,19);
									}
									display_cursor_en=0;//�رչ����ʾ
								
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
