#ifndef _NRF24L01_H
#define _NRF24L01_H

#define uchar unsigned char
#define uint  unsigned int

/*******************NRF24L01接口定义********************/
//#define NRF24L01_MISO PB4 //输入0
#define Hign_24L01_MISO     PORTB |= (1 << PB4)
#define Low_24L01_MISO      PORTB &= ~(1 << PB4)
#define Read_24L01_MISO     PINB & (1 << PB4)
//#define NRF24L01_MOSI PB3 //输出1
#define Hign_24L01_MOSI     PORTB |= (1 << PB3)
#define Low_24L01_MOSI      PORTB &= ~(1 << PB3)
#define Read_24L01_MOSI     PINB & (1 << PB3)
//#define NRF24L01_CSN PB2 //输出1
#define Low_24L01_CSN       PORTB &= ~(1 << PB2)
#define Hign_24L01_CSN      PORTB |= (1 << PB2)
//#define NRF24L01_SCK PB5 //输出1
#define Hign_24L01_SCK      PORTB |= (1 << PB5)
#define Low_24L01_SCK       PORTB &= ~(1 << PB5)
#define Read_24L01_SCK      PINB & (1 << PB5);
//#define NRF24L01_CE PB1 //输出1
#define Hign_24L01_CE       PORTB |= (1 << PB1)
#define Low_24L01_CE        PORTB &= ~(1 << PB1)
#define Read_24L01_CE       PINB & (1 << PB1)
//#define NRF24L01_IRQ PD2 //输入0
#define Hign_NRF24L01_IRQ   PORTD |= (1 << PD2)
#define Low_NRF24L01_IRQ    PORTD &= ~(1 << PD2)
#define Read_NRF24L01_IRQ   PIND & (1 << PD2)

//*********************************NRF24L01发送数据长度定义***************************************
#define TX_ADR_WIDTH 5 // 5 uints TX address width 地址最长为5
#define RX_ADR_WIDTH 5 // 5 uints RX address width 地址最长为5
#define TX_PLOAD_WIDTH 9 // 32 uints TX payload
#define RX_PLOAD_WIDTH 9 // 32 uints TX payload
//*********************************NRF24L01寄存器指令*********************************************
#define READ_REG 0x00 // 读寄存器指令
#define WRITE_REG 0x20 // 写寄存器指令
#define RD_RX_PLOAD 0x61 // 读取接收数据指令
#define WR_TX_PLOAD 0xA0 // 写待发数据指令
#define FLUSH_TX 0xE1 // 清空发送 FIFO指令
#define FLUSH_RX 0xE2 // 清空接收 FIFO指令
#define REUSE_TX_PL 0xE3 // 重复装载数据指令
#define NOP1 0xFF // 保留
//**************************** **NRF24L01寄存器地址**********************************************
#define CONFIG 0x00 // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA 0x01 // 自动应答功能设置
#define EN_RXADDR 0x02 // 可用信道设置
#define SETUP_AW 0x03 // 收发地址宽度设置
#define SETUP_RETR 0x04 // 自动重发功能设置
#define RF_CH 0x05 // 工作频率设置
#define RF_SETUP 0x06 // 发射速率、功耗功能设置
#define STATUS 0x07 // 状态寄存器
#define OBSERVE_TX 0x08 // 发送监测功能
#define CD 0x09 // 地址检测
#define RX_ADDR_P0 0x0A // 频道0接收数据地址
#define RX_ADDR_P1 0x0B // 频道1接收数据地址
#define RX_ADDR_P2 0x0C // 频道2接收数据地址
#define RX_ADDR_P3 0x0D // 频道3接收数据地址
#define RX_ADDR_P4 0x0E // 频道4接收数据地址
#define RX_ADDR_P5 0x0F // 频道5接收数据地址
#define TX_ADDR 0x10 // 发送地址寄存器
#define RX_PW_P0 0x11 // 接收频道0接收数据长度
#define RX_PW_P1 0x12 // 接收频道0接收数据长度
#define RX_PW_P2 0x13 // 接收频道0接收数据长度
#define RX_PW_P3 0x14 // 接收频道0接收数据长度
#define RX_PW_P4 0x15 // 接收频道0接收数据长度
#define RX_PW_P5 0x16 // 接收频道0接收数据长度
#define FIFO_STATUS 0x17 // FIFO栈入栈出状态寄存器设置
//*****************************变量定义及函数声明***********************************************
//NRF24l01
void init_NRF24L01(uchar station);
uchar SPI_RW(uchar data);
uchar SPI_Read(uchar reg);
void SPI_WRITE(uchar reg, uchar value);
void SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars);
void SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars);
void SetRX_Mode(void);
uchar nRF24L01_RxPacket(uchar* rx_buf);
void nRF24L01_TxPacket(uchar * tx_buf);
//其他
void clr_buffer(uchar *p);
uchar check(uchar *p);
//变量声明
extern uchar TX_ADDRESS[TX_ADR_WIDTH]; //TX address
extern uchar RX_ADDRESS[RX_ADR_WIDTH]; //RX address
uchar TX_ADDRESS[TX_ADR_WIDTH]= {0xff,0xff,0xff,0xff,0xff}; //TX:本地地址  和主机相同
uchar RX_ADDRESS[RX_ADR_WIDTH]= {0xff,0xff,0xff,0xff,0x00}; //RX:通道0地址，和基址相同
uchar sta,l;
uchar station_T=0;
uchar station_R=0;
//************************************ 数据校验*********************************************
uchar check(uchar *p)
{
	uchar result; //10** **01 **** ****
	if(((*p&0x81)!=0)&&((*p&0x02)==0)) //**** **** 10** **01
		if(((*(p+3)&0x81)!=0)&&((*(p+3)&0x42)==0))
			result=1;
		else result=0;
	else result=0;
	return result;
}
/*********************************************************************************************
函数：uint SPI_RW(uint uchar)
功能：NRF24L01的SPI写时序
**********************************************************************************************/
uchar SPI_RW(uchar data)
{
	SPDR=data;
	while(!(SPSR&0x80));
	_delay_us(10);
	data=SPDR;
	return data;
}
/**********************************************************************************************
*函数：uchar SPI_Read(uchar reg)
功能：NRF24L01的SPI读时序
***********************************************************************************************/
uchar SPI_Read(uchar reg)
{
	uchar reg_val;
	Low_24L01_CSN; // 拉低CSN,开始SPI的通信
	SPI_RW(reg); // 发送地址
	reg_val = SPI_RW(0); // 读取从地址返回的数据
	Hign_24L01_CSN; // 置高CSN,结束SPI的通信
	return(reg_val); // 返回接受的数据
}
/************************************************************************************************
函数：void SPI_WRITE(uchar reg, uchar value)
功能：NRF24L01的SPI写时序
*************************************************************************************************/
void SPI_WRITE(uchar reg, uchar value)
{
	Low_24L01_CSN; // 拉低CSN,开始SPI的通信
	SPI_RW(reg); // 发送要写入数据的地址
	SPI_RW(value); // 写入数据
	Hign_24L01_CSN; // 置高CSN,结束SPI的通信
}
/************************************************************************************************
函数：uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
功能: 用于读数据，reg：为寄存器地址，pBuf：为待读出数据地址，uchars：读出数据的个数
************************************************************************************************/
void SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
{
	int uchar_ctr;
	Low_24L01_CSN; // 拉低CSN,开始SPI的通信
	SPI_RW(reg); // 发送要读出数据的地址
	for(uchar_ctr=0;uchar_ctr<uchars;uchar_ctr++)
		pBuf[uchar_ctr] = SPI_RW(0); // 读取数据
	Hign_24L01_CSN; // 置高CSN,结束SPI的通信
}
/************************************************************************************************
函数：uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
功能: 用于写数据：reg为寄存器地址，pBuf：为待写入数据地址，uchars：写入数据的个数
*************************************************************************************************/
void SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
{
	uchar uchar_ctr;
	Low_24L01_CSN; // 拉低CSN,开始SPI的通信
	SPI_RW(reg); // 发送要写入数据的地址
	for(uchar_ctr=0; uchar_ctr<uchars; uchar_ctr++)
		SPI_RW(*pBuf++); // 写入数据
	Hign_24L01_CSN; // 置高CSN,结束SPI的通信
}
/************************************************************************************************
函数：void SetRX_Mode(void)
功能：数据接收配置
************************************************************************************************/
void SetRX_Mode(void)
{
	Low_24L01_CE; // 拉低CE，进入工作模式的选择
	//SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);
	SPI_WRITE(WRITE_REG + RF_CH, station_R);//设置接收频道
	SPI_WRITE(WRITE_REG + CONFIG, 0x3f); // 配置：IRQ收发完成中断响应，16位CRC ，接收模式

	Hign_24L01_CE; // 置高CE，切换工作模式
	_delay_us(500); // 切换模式后至少130us,见数据手册
}
/***********************************************************************************************
函数：void POWER_DOWN(void)
功能：数据掉电模式配置
************************************************************************************************/
void POWER_DOWN(void)
{
	Low_24L01_CE; // 拉低CE，进入工作模式的选择
	SPI_WRITE(WRITE_REG + CONFIG, 0x74); // 配置：IRQ收发完成中断响应，16位CRC ，掉电模式
	_delay_us(150); // 不用置高CE，切换模式后至少130us,见数据手册
}
/************************************************************************************************
函数：uchar nRF24L01_RxPacket(uchar* rx_buf)
功能：数据读取后放如rx_buf接收缓冲区中
*************************************************************************************************/
uchar nRF24L01_RxPacket(uchar* rx_buf)
{
	uchar revale=0;
	sta=SPI_Read(STATUS); // 读取状态寄存器接受标志
	if(sta&0x40) // 判断是否接收到数据
	{
		Low_24L01_CE; // NRF24C01使能
		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH); //接收数据
		revale =1; //读取数据完成标志
	}
	SPI_WRITE(FLUSH_RX,0); //清空接收数据缓存区
	SPI_WRITE(WRITE_REG+STATUS,sta); //接收到数据后，通过写1来清除中断标志

	Hign_24L01_CE; // 置高CE，切换工作模式
	return revale;
}
/************************************************************************************************
函数：void nRF24L01_TxPacket(uchar * tx_buf)
功能：发送 tx_buf中数据
*************************************************************************************************/
void nRF24L01_TxPacket(uchar * tx_buf)
{
	Low_24L01_CE; // NRF24C01使能
	SPI_WRITE(WRITE_REG + RF_CH, station_T);//设置发送频道
	SPI_WRITE(FLUSH_TX,0); //清空发送数据缓存区
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); // 装载数据
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);
	SPI_WRITE(WRITE_REG + CONFIG, 0x7e); // 配置：IRQ收发完成中断响应，16位CRC，发送模式
	Hign_24L01_CE; // 置高CE，数据开始发送
	_delay_us(15); // 等待数据发送完成
	//SPI_WRITE(WRITE_REG+STATUS,0x70); //接收到数据后，通过写1来清除中断标志
	//_delay_ms(1);
	SetRX_Mode(); // 发送完成，进入接收模式
}
/****************************************************************************************
//NRF24L01初始化
***************************************************************************************/
void init_NRF24L01(uchar station)
{
	DDRB|=0x2e;
	station_T=station;
	station_R=(station+5)%125;//接收频道
	Low_24L01_CE; // 拉低CE,NRF24C01使能
	Hign_24L01_CSN; // SPI 停止通讯
	Low_24L01_SCK; // 时钟初始状态
	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH); // 写本地地址
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // 装载通道0的地址，用于ACK
//*********************************配置NRF24C01**************************************
	SPI_WRITE(WRITE_REG + EN_AA, 0x00); // 关闭自动 ACK应答
	SPI_WRITE(WRITE_REG + EN_RXADDR, 0x01); // 允许接收地址只有频道0
	SPI_WRITE(WRITE_REG + SETUP_RETR, 0x00); //设置自动重发功能
	SPI_WRITE(WRITE_REG + RF_CH, station); // 设置信道工作为2.4GHZ，收发必须一致 发送频道
	SPI_WRITE(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度:9字节
	SPI_WRITE(WRITE_REG + RF_SETUP, 0x07); //设置发射速率为1MHZ，发射功率为最大值0dB
	SPI_WRITE(WRITE_REG + CONFIG, 0x3f); // IRQ收发完成中断响应，16位CRC 
	Hign_24L01_CE;
	_delay_ms(200); //初始化完成
}
#endif


