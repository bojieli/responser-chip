#ifndef _NRF24L01_H
#define _NRF24L01_H

#define uchar unsigned char
#define uint  unsigned int

/*******************NRF24L01�ӿڶ���********************/
//#define NRF24L01_MISO PB4 //����0
#define Hign_24L01_MISO     PORTB |= (1 << PB4)
#define Low_24L01_MISO      PORTB &= ~(1 << PB4)
#define Read_24L01_MISO     PINB & (1 << PB4)
//#define NRF24L01_MOSI PB3 //���1
#define Hign_24L01_MOSI     PORTB |= (1 << PB3)
#define Low_24L01_MOSI      PORTB &= ~(1 << PB3)
#define Read_24L01_MOSI     PINB & (1 << PB3)
//#define NRF24L01_CSN PB2 //���1
#define Low_24L01_CSN       PORTB &= ~(1 << PB2)
#define Hign_24L01_CSN      PORTB |= (1 << PB2)
//#define NRF24L01_SCK PB5 //���1
#define Hign_24L01_SCK      PORTB |= (1 << PB5)
#define Low_24L01_SCK       PORTB &= ~(1 << PB5)
#define Read_24L01_SCK      PINB & (1 << PB5);
//#define NRF24L01_CE PB1 //���1
#define Hign_24L01_CE       PORTB |= (1 << PB1)
#define Low_24L01_CE        PORTB &= ~(1 << PB1)
#define Read_24L01_CE       PINB & (1 << PB1)
//#define NRF24L01_IRQ PD2 //����0
#define Hign_NRF24L01_IRQ   PORTD |= (1 << PD2)
#define Low_NRF24L01_IRQ    PORTD &= ~(1 << PD2)
#define Read_NRF24L01_IRQ   PIND & (1 << PD2)

//*********************************NRF24L01�������ݳ��ȶ���***************************************
#define TX_ADR_WIDTH 5 // 5 uints TX address width ��ַ�Ϊ5
#define RX_ADR_WIDTH 5 // 5 uints RX address width ��ַ�Ϊ5
#define TX_PLOAD_WIDTH 9 // 32 uints TX payload
#define RX_PLOAD_WIDTH 9 // 32 uints TX payload
//*********************************NRF24L01�Ĵ���ָ��*********************************************
#define READ_REG 0x00 // ���Ĵ���ָ��
#define WRITE_REG 0x20 // д�Ĵ���ָ��
#define RD_RX_PLOAD 0x61 // ��ȡ��������ָ��
#define WR_TX_PLOAD 0xA0 // д��������ָ��
#define FLUSH_TX 0xE1 // ��շ��� FIFOָ��
#define FLUSH_RX 0xE2 // ��ս��� FIFOָ��
#define REUSE_TX_PL 0xE3 // �ظ�װ������ָ��
#define NOP1 0xFF // ����
//**************************** **NRF24L01�Ĵ�����ַ**********************************************
#define CONFIG 0x00 // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA 0x01 // �Զ�Ӧ��������
#define EN_RXADDR 0x02 // �����ŵ�����
#define SETUP_AW 0x03 // �շ���ַ�������
#define SETUP_RETR 0x04 // �Զ��ط���������
#define RF_CH 0x05 // ����Ƶ������
#define RF_SETUP 0x06 // �������ʡ����Ĺ�������
#define STATUS 0x07 // ״̬�Ĵ���
#define OBSERVE_TX 0x08 // ���ͼ�⹦��
#define CD 0x09 // ��ַ���
#define RX_ADDR_P0 0x0A // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1 0x0B // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2 0x0C // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3 0x0D // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4 0x0E // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5 0x0F // Ƶ��5�������ݵ�ַ
#define TX_ADDR 0x10 // ���͵�ַ�Ĵ���
#define RX_PW_P0 0x11 // ����Ƶ��0�������ݳ���
#define RX_PW_P1 0x12 // ����Ƶ��0�������ݳ���
#define RX_PW_P2 0x13 // ����Ƶ��0�������ݳ���
#define RX_PW_P3 0x14 // ����Ƶ��0�������ݳ���
#define RX_PW_P4 0x15 // ����Ƶ��0�������ݳ���
#define RX_PW_P5 0x16 // ����Ƶ��0�������ݳ���
#define FIFO_STATUS 0x17 // FIFOջ��ջ��״̬�Ĵ�������
//*****************************�������弰��������***********************************************
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
//����
void clr_buffer(uchar *p);
uchar check(uchar *p);
//��������
extern uchar TX_ADDRESS[TX_ADR_WIDTH]; //TX address
extern uchar RX_ADDRESS[RX_ADR_WIDTH]; //RX address
uchar TX_ADDRESS[TX_ADR_WIDTH]= {0xff,0xff,0xff,0xff,0xff}; //TX:���ص�ַ  ��������ͬ
uchar RX_ADDRESS[RX_ADR_WIDTH]= {0xff,0xff,0xff,0xff,0x00}; //RX:ͨ��0��ַ���ͻ�ַ��ͬ
uchar sta,l;
uchar station_T=0;
uchar station_R=0;
//************************************ ����У��*********************************************
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
������uint SPI_RW(uint uchar)
���ܣ�NRF24L01��SPIдʱ��
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
*������uchar SPI_Read(uchar reg)
���ܣ�NRF24L01��SPI��ʱ��
***********************************************************************************************/
uchar SPI_Read(uchar reg)
{
	uchar reg_val;
	Low_24L01_CSN; // ����CSN,��ʼSPI��ͨ��
	SPI_RW(reg); // ���͵�ַ
	reg_val = SPI_RW(0); // ��ȡ�ӵ�ַ���ص�����
	Hign_24L01_CSN; // �ø�CSN,����SPI��ͨ��
	return(reg_val); // ���ؽ��ܵ�����
}
/************************************************************************************************
������void SPI_WRITE(uchar reg, uchar value)
���ܣ�NRF24L01��SPIдʱ��
*************************************************************************************************/
void SPI_WRITE(uchar reg, uchar value)
{
	Low_24L01_CSN; // ����CSN,��ʼSPI��ͨ��
	SPI_RW(reg); // ����Ҫд�����ݵĵ�ַ
	SPI_RW(value); // д������
	Hign_24L01_CSN; // �ø�CSN,����SPI��ͨ��
}
/************************************************************************************************
������uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
����: ���ڶ����ݣ�reg��Ϊ�Ĵ�����ַ��pBuf��Ϊ���������ݵ�ַ��uchars���������ݵĸ���
************************************************************************************************/
void SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
{
	int uchar_ctr;
	Low_24L01_CSN; // ����CSN,��ʼSPI��ͨ��
	SPI_RW(reg); // ����Ҫ�������ݵĵ�ַ
	for(uchar_ctr=0;uchar_ctr<uchars;uchar_ctr++)
		pBuf[uchar_ctr] = SPI_RW(0); // ��ȡ����
	Hign_24L01_CSN; // �ø�CSN,����SPI��ͨ��
}
/************************************************************************************************
������uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
����: ����д���ݣ�regΪ�Ĵ�����ַ��pBuf��Ϊ��д�����ݵ�ַ��uchars��д�����ݵĸ���
*************************************************************************************************/
void SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
{
	uchar uchar_ctr;
	Low_24L01_CSN; // ����CSN,��ʼSPI��ͨ��
	SPI_RW(reg); // ����Ҫд�����ݵĵ�ַ
	for(uchar_ctr=0; uchar_ctr<uchars; uchar_ctr++)
		SPI_RW(*pBuf++); // д������
	Hign_24L01_CSN; // �ø�CSN,����SPI��ͨ��
}
/************************************************************************************************
������void SetRX_Mode(void)
���ܣ����ݽ�������
************************************************************************************************/
void SetRX_Mode(void)
{
	Low_24L01_CE; // ����CE�����빤��ģʽ��ѡ��
	//SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);
	SPI_WRITE(WRITE_REG + RF_CH, station_R);//���ý���Ƶ��
	SPI_WRITE(WRITE_REG + CONFIG, 0x3f); // ���ã�IRQ�շ�����ж���Ӧ��16λCRC ������ģʽ

	Hign_24L01_CE; // �ø�CE���л�����ģʽ
	_delay_us(500); // �л�ģʽ������130us,�������ֲ�
}
/***********************************************************************************************
������void POWER_DOWN(void)
���ܣ����ݵ���ģʽ����
************************************************************************************************/
void POWER_DOWN(void)
{
	Low_24L01_CE; // ����CE�����빤��ģʽ��ѡ��
	SPI_WRITE(WRITE_REG + CONFIG, 0x74); // ���ã�IRQ�շ�����ж���Ӧ��16λCRC ������ģʽ
	_delay_us(150); // �����ø�CE���л�ģʽ������130us,�������ֲ�
}
/************************************************************************************************
������uchar nRF24L01_RxPacket(uchar* rx_buf)
���ܣ����ݶ�ȡ�����rx_buf���ջ�������
*************************************************************************************************/
uchar nRF24L01_RxPacket(uchar* rx_buf)
{
	uchar revale=0;
	sta=SPI_Read(STATUS); // ��ȡ״̬�Ĵ������ܱ�־
	if(sta&0x40) // �ж��Ƿ���յ�����
	{
		Low_24L01_CE; // NRF24C01ʹ��
		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH); //��������
		revale =1; //��ȡ������ɱ�־
	}
	SPI_WRITE(FLUSH_RX,0); //��ս������ݻ�����
	SPI_WRITE(WRITE_REG+STATUS,sta); //���յ����ݺ�ͨ��д1������жϱ�־

	Hign_24L01_CE; // �ø�CE���л�����ģʽ
	return revale;
}
/************************************************************************************************
������void nRF24L01_TxPacket(uchar * tx_buf)
���ܣ����� tx_buf������
*************************************************************************************************/
void nRF24L01_TxPacket(uchar * tx_buf)
{
	Low_24L01_CE; // NRF24C01ʹ��
	SPI_WRITE(WRITE_REG + RF_CH, station_T);//���÷���Ƶ��
	SPI_WRITE(FLUSH_TX,0); //��շ������ݻ�����
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); // װ������
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);
	SPI_WRITE(WRITE_REG + CONFIG, 0x7e); // ���ã�IRQ�շ�����ж���Ӧ��16λCRC������ģʽ
	Hign_24L01_CE; // �ø�CE�����ݿ�ʼ����
	_delay_us(15); // �ȴ����ݷ������
	//SPI_WRITE(WRITE_REG+STATUS,0x70); //���յ����ݺ�ͨ��д1������жϱ�־
	//_delay_ms(1);
	SetRX_Mode(); // ������ɣ��������ģʽ
}
/****************************************************************************************
//NRF24L01��ʼ��
***************************************************************************************/
void init_NRF24L01(uchar station)
{
	DDRB|=0x2e;
	station_T=station;
	station_R=(station+5)%125;//����Ƶ��
	Low_24L01_CE; // ����CE,NRF24C01ʹ��
	Hign_24L01_CSN; // SPI ֹͣͨѶ
	Low_24L01_SCK; // ʱ�ӳ�ʼ״̬
	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH); // д���ص�ַ
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // װ��ͨ��0�ĵ�ַ������ACK
//*********************************����NRF24C01**************************************
	SPI_WRITE(WRITE_REG + EN_AA, 0x00); // �ر��Զ� ACKӦ��
	SPI_WRITE(WRITE_REG + EN_RXADDR, 0x01); // ������յ�ַֻ��Ƶ��0
	SPI_WRITE(WRITE_REG + SETUP_RETR, 0x00); //�����Զ��ط�����
	SPI_WRITE(WRITE_REG + RF_CH, station); // �����ŵ�����Ϊ2.4GHZ���շ�����һ�� ����Ƶ��
	SPI_WRITE(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //���ý������ݳ���:9�ֽ�
	SPI_WRITE(WRITE_REG + RF_SETUP, 0x07); //���÷�������Ϊ1MHZ�����书��Ϊ���ֵ0dB
	SPI_WRITE(WRITE_REG + CONFIG, 0x3f); // IRQ�շ�����ж���Ӧ��16λCRC 
	Hign_24L01_CE;
	_delay_ms(200); //��ʼ�����
}
#endif


