#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#define uchar unsigned char
#define uint unsigned int
/***************************************************/
#define RES_O DDRB|=(1<<PB0)
#define RES_L PORTB&=~(1<<PB0)
#define RES_H PORTB|=(1<<PB0)

#define SDA_O DDRC|=(1<<PC4)
#define SDA_I DDRC&=~(1<<PC4)
#define SDA_L {DDRC|=(1<<PC4);PORTC&=~(1<<PC4);}
#define SDA_H {DDRC&=~(1<<PC4);PORTC|=(1<<PC4);}

#define SCL_O DDRC|=(1<<PC5)
#define SCL_I DDRC&=~(1<<PC5)
#define SCL_L {DDRC|=(1<<PC5);PORTC&=~(1<<PC5);}
#define SCL_H {DDRC&=~(1<<PC5);PORTC|=(1<<PC5);}

/***************************************************/


const prog_uchar tab_16_8[21][16]={
{0x00,0x00,0xE0,0x0F,0x10,0x10,0x08,0x20,0x08,0x20,0x10,0x10,0xE0,0x0F,0x00,0x00},/*"0",0*/
{0x00,0x00,0x10,0x20,0x10,0x20,0xF8,0x3F,0x00,0x20,0x00,0x20,0x00,0x00,0x00,0x00},/*"1",1*/
{0x00,0x00,0x70,0x30,0x08,0x28,0x08,0x24,0x08,0x22,0x88,0x21,0x70,0x30,0x00,0x00},/*"2",2*/
{0x00,0x00,0x30,0x18,0x08,0x20,0x88,0x20,0x88,0x20,0x48,0x11,0x30,0x0E,0x00,0x00},/*"3",3*/
{0x00,0x00,0x00,0x07,0xC0,0x04,0x20,0x24,0x10,0x24,0xF8,0x3F,0x00,0x24,0x00,0x00},/*"4",4*/
{0x00,0x00,0xF8,0x19,0x08,0x21,0x88,0x20,0x88,0x20,0x08,0x11,0x08,0x0E,0x00,0x00},/*"5",5*/
{0x00,0x00,0xE0,0x0F,0x10,0x11,0x88,0x20,0x88,0x20,0x18,0x11,0x00,0x0E,0x00,0x00},/*"6",6*/
{0x00,0x00,0x38,0x00,0x08,0x00,0x08,0x3F,0xC8,0x00,0x38,0x00,0x08,0x00,0x00,0x00},/*"7",7*/
{0x00,0x00,0x70,0x1C,0x88,0x22,0x08,0x21,0x08,0x21,0x88,0x22,0x70,0x1C,0x00,0x00},/*"8",8*/
{0x00,0x00,0xE0,0x00,0x10,0x31,0x08,0x22,0x08,0x22,0x10,0x11,0xE0,0x0F,0x00,0x00},/*"9",9*/
{0x00,0x20,0x00,0x3C,0xC0,0x23,0x38,0x02,0xE0,0x02,0x00,0x27,0x00,0x38,0x00,0x20},/*"A",10*/
{0x08,0x20,0xF8,0x3F,0x88,0x20,0x88,0x20,0x88,0x20,0x70,0x11,0x00,0x0E,0x00,0x00},/*"B",11*/
{0xC0,0x07,0x30,0x18,0x08,0x20,0x08,0x20,0x08,0x20,0x08,0x10,0x38,0x08,0x00,0x00},/*"C",12*/
{0x08,0x20,0xF8,0x3F,0x08,0x20,0x08,0x20,0x08,0x20,0x10,0x10,0xE0,0x0F,0x00,0x00},/*"D",13*/
{0x08,0x20,0xF8,0x3F,0x88,0x20,0x88,0x20,0xE8,0x23,0x08,0x20,0x10,0x18,0x00,0x00},/*"E",14*/
{0x08,0x20,0xF8,0x3F,0x88,0x20,0x88,0x00,0xE8,0x03,0x08,0x00,0x10,0x00,0x00,0x00},/*"F",15*/

{0x00,0x00,0x00,0x00,0x00,0x36,0x00,0x36,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"：",16*/
{0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x33,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"！",17*/
{0x00,0x00,0x00,0x30,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*".",18*/
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*" ",19*/
{0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80},/*"_",20*/
};

 /*编(0) 号(1) 学(2) 答(3) 案(4) 基(5) 站(6) 发(7) 送(8) 中(9) 完(10) 成(11)*/
const prog_uchar tab_16_16[21][32]={
{0x20,0x22,0x30,0x67,0xAC,0x22,0x63,0x12,0x30,0x52,0x00,0x38,0xFC,0x07,0x24,0xFF,
0x25,0x09,0x26,0x7F,0x24,0x09,0x24,0x3F,0x24,0x89,0x3C,0xFF,0x00,0x00,0x00,0x00},/*"编",0*/
{0x80,0x00,0x80,0x00,0x80,0x00,0xBE,0x06,0xA2,0x05,0xA2,0x04,0xA2,0x04,0xA2,0x04,
0xA2,0x44,0xA2,0x84,0xA2,0x44,0xBE,0x3C,0x80,0x00,0x80,0x00,0x80,0x00,0x00,0x00},/*"号",1*/
{0x40,0x04,0x30,0x04,0x11,0x04,0x96,0x04,0x90,0x04,0x90,0x44,0x91,0x84,0x96,0x7E,
0x90,0x06,0x90,0x05,0x98,0x04,0x14,0x04,0x13,0x04,0x50,0x04,0x30,0x04,0x00,0x00},/*"学",2*/
{0x20,0x01,0x10,0x01,0x8C,0x00,0x87,0xFC,0x4C,0x45,0x54,0x45,0x24,0x45,0x14,0x45,
0x28,0x45,0x47,0x45,0x44,0x45,0x8C,0xFC,0x94,0x00,0x04,0x01,0x04,0x01,0x00,0x00},/*"答",3*/
{0x10,0x44,0x16,0x44,0x12,0x25,0x52,0x25,0x72,0x15,0x5A,0x0D,0xD2,0x04,0x93,0xFE,
0x92,0x04,0xD2,0x0C,0x32,0x15,0x12,0x25,0x12,0x25,0x16,0x44,0x10,0x44,0x00,0x00},/*"案",4*/
{0x00,0x11,0x04,0x11,0x04,0x89,0x04,0x85,0xFF,0x93,0x54,0x91,0x54,0x91,0x54,0xFD,
0x54,0x91,0x54,0x91,0xFF,0x93,0x04,0x85,0x04,0x89,0x04,0x11,0x00,0x11,0x00,0x00},/*"基",5*/
{0x00,0x10,0xC8,0x30,0x08,0x17,0x09,0x10,0x0E,0x0E,0xE8,0x09,0x08,0x08,0x00,0xFF,
0x00,0x41,0x00,0x41,0xFF,0x41,0x10,0x41,0x10,0x41,0x10,0xFF,0x10,0x00,0x00,0x00},/*"站",6*/
{0x00,0x00,0x00,0x20,0x18,0x10,0x16,0x8C,0x10,0x83,0xD0,0x80,0xB8,0x41,0x97,0x46,
0x90,0x28,0x90,0x10,0x90,0x28,0x92,0x44,0x94,0x43,0x10,0x80,0x00,0x80,0x00,0x00},/*"发",7*/
{0x40,0x00,0x40,0x40,0x42,0x20,0xCC,0x1F,0x00,0x20,0x88,0x40,0x89,0x50,0x8E,0x48,
0x88,0x46,0xF8,0x41,0x88,0x42,0x8C,0x44,0x8B,0x58,0x88,0x40,0x80,0x40,0x00,0x00},/*"送",8*/
{0x00,0x00,0x00,0x00,0xF0,0x0F,0x10,0x04,0x10,0x04,0x10,0x04,0x10,0x04,0xFF,0xFF,
0x10,0x04,0x10,0x04,0x10,0x04,0x10,0x04,0xF0,0x0F,0x00,0x00,0x00,0x00,0x00,0x00},/*"中",9*/

{0x00,0x80,0x00,0x60,0xF8,0x1F,0x88,0x00,0x88,0x10,0x88,0x20,0x88,0x1F,0x08,0x80,
0x08,0x40,0xFF,0x21,0x08,0x16,0x09,0x18,0x0A,0x26,0xC8,0x41,0x08,0xF8,0x00,0x00},/*"成",0*/

{0x08,0x10,0x08,0x30,0x08,0x10,0xF8,0x1F,0x08,0x08,0x08,0x88,0x08,0x48,0x10,0x30,
0x10,0x0E,0xFF,0x01,0x10,0x40,0x10,0x80,0x10,0x40,0xF0,0x3F,0x00,0x00,0x00,0x00},/*"功",11*/

{0x00,0x81,0x40,0x81,0x30,0x41,0x1E,0x21,0x10,0x11,0x10,0x0D,0x10,0x03,0xFF,0x01,
0x10,0x03,0x10,0x0D,0x10,0x11,0x10,0x21,0x10,0x41,0x00,0x81,0x00,0x81,0x00,0x00},/*"失",12*/

{0x00,0x80,0xFE,0x47,0x02,0x30,0xFA,0x0F,0x02,0x10,0xFE,0x67,0x40,0x80,0x20,0x40,
0xD8,0x21,0x17,0x16,0x10,0x08,0x10,0x16,0xF0,0x21,0x10,0x40,0x10,0x80,0x00,0x00},/*"败",13*/
{0x80,0x20,0x80,0x20,0x88,0x10,0x88,0x08,0x88,0x04,0x88,0x02,0x88,0x01,0xFF,0xFF,
0x88,0x01,0x88,0x02,0x88,0x04,0x88,0x08,0x88,0x10,0x80,0x20,0x80,0x20,0x00,0x00},/*"未",14*/

{0x10,0x04,0x60,0x04,0x02,0x7E,0x8C,0x01,0x00,0x40,0x08,0x40,0x08,0x41,0x08,0x41,
0x09,0x41,0xFA,0x7F,0x08,0x41,0x08,0x41,0x08,0x41,0x08,0x41,0x00,0x40,0x00,0x00},/*"注",15*/

{0x80,0x00,0x80,0x80,0x80,0x40,0xFE,0x3F,0x82,0x00,0x82,0x40,0xFE,0x7F,0x80,0x80,
0x80,0x60,0xFE,0x1F,0x82,0x40,0x82,0x80,0xFE,0x7F,0x80,0x00,0x80,0x00,0x00,0x00},/*"册",16*/

{0x04,0x10,0x24,0x08,0x44,0x06,0x84,0x01,0x64,0x82,0x9C,0x4C,0x40,0x20,0x30,0x18,
0x0F,0x06,0xC8,0x01,0x08,0x06,0x08,0x18,0x28,0x20,0x18,0x40,0x00,0x80,0x00,0x00},/*"欢",17*/

{0x40,0x00,0x40,0x40,0x42,0x20,0xCC,0x1F,0x00,0x20,0x00,0x40,0xFC,0x4F,0x04,0x44,
0x02,0x42,0x00,0x40,0xFC,0x7F,0x04,0x42,0x04,0x44,0xFC,0x43,0x00,0x40,0x00,0x00},/*"迎",18*/

{0x80,0x00,0x60,0x00,0xF8,0xFF,0x07,0x00,0x04,0x80,0xE4,0x81,0x24,0x45,0x24,0x29,
0x24,0x11,0xFF,0x2F,0x24,0x41,0x24,0x41,0x24,0x81,0xE4,0x81,0x04,0x80,0x00,0x00},/*"使",19*/

{0x00,0x80,0x00,0x60,0xFE,0x1F,0x22,0x02,0x22,0x02,0x22,0x02,0x22,0x02,0xFE,0x7F,
0x22,0x02,0x22,0x02,0x22,0x42,0x22,0x82,0xFE,0x7F,0x00,0x00,0x00,0x00,0x00,0x00},/*"用",20*/
};
const prog_uchar tab_power[9][64]={
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xC0,0x03,0xC0,0x03,0xF8,0x1F,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,
0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,
0x08,0x10,0x08,0x10,0x08,0x10,0xF8,0x1F,0xF8,0x1F,0x00,0x00,0x00,0x00,0x00,0x00},/*1*/

{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xC0,0x03,0xC0,0x03,0xF8,0x1F,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,
0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,
0x08,0x10,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0x00,0x00,0x00,0x00,0x00,0x00},/*2*/

{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xC0,0x03,0xC0,0x03,0xF8,0x1F,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,
0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0xF8,0x1F,
0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0x00,0x00,0x00,0x00,0x00,0x00},/*3*/

{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xC0,0x03,0xC0,0x03,0xF8,0x1F,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,
0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,
0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0x00,0x00,0x00,0x00,0x00,0x00},/*4*/

{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xC0,0x03,0xC0,0x03,0xF8,0x1F,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,
0x08,0x10,0x08,0x10,0x08,0x10,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,
0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0x00,0x00,0x00,0x00,0x00,0x00},/*5*/

{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xC0,0x03,0xC0,0x03,0xF8,0x1F,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,
0x08,0x10,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,
0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0x00,0x00,0x00,0x00,0x00,0x00},/*6*/

{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xC0,0x03,0xC0,0x03,0xF8,0x1F,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0xF8,0x1F,
0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,
0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0x00,0x00,0x00,0x00,0x00,0x00},/*7*/

{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xC0,0x03,0xC0,0x03,0xF8,0x1F,0x08,0x10,0x08,0x10,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,
0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,
0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0x00,0x00,0x00,0x00,0x00,0x00},/*8*/

{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xC0,0x03,0xC0,0x03,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,
0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,
0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0xF8,0x1F,0x00,0x00,0x00,0x00,0x00,0x00},/*9*/
};
const prog_uchar tab_picture1[1024]={
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x7F,0x98,0xF9,0xFF,0xE7,0xFF,0x07,
0xFE,0x7F,0x98,0xF9,0xFF,0xE7,0xFF,0x07,0x06,0x60,0x9E,0x7F,0x1E,0x60,0x00,0x06,
0x06,0x60,0x9E,0x7F,0x1E,0x60,0x00,0x06,0xE6,0x67,0xE6,0x07,0x9E,0x67,0x7E,0x06,
0xE6,0x67,0xE6,0x07,0x9E,0x67,0x7E,0x06,0xE6,0x67,0x1E,0x18,0x18,0x66,0x7E,0x06,
0xE6,0x67,0x1E,0x18,0x18,0x66,0x7E,0x06,0xE6,0x67,0x7E,0x80,0x61,0x60,0x7E,0x06,
0xE6,0x67,0x7E,0x80,0x61,0x60,0x7E,0x06,0x06,0x60,0xE6,0x19,0x86,0x67,0x00,0x06,
0x06,0x60,0xE6,0x19,0x86,0x67,0x00,0x06,0xFE,0x7F,0x66,0x66,0x66,0xE6,0xFF,0x07,
0xFE,0x7F,0x66,0x66,0x66,0xE6,0xFF,0x07,0x00,0x00,0x9E,0x7F,0xE6,0x07,0x00,0x00,
0x00,0x00,0x9E,0x7F,0xE6,0x07,0x00,0x00,0x06,0x7E,0xE0,0xE7,0xF9,0xFF,0xFF,0x01,
0x06,0x7E,0xE0,0xE7,0xF9,0xFF,0xFF,0x01,0x06,0x18,0x1E,0x78,0x98,0x81,0xF9,0x07,
0x06,0x18,0x1E,0x78,0x98,0x81,0xF9,0x07,0x18,0xF8,0x79,0x86,0x19,0x98,0x07,0x00,
0x18,0xF8,0x79,0x86,0x19,0x98,0x07,0x00,0x66,0x06,0x18,0x7E,0x66,0x1E,0xE6,0x07,
0x66,0x06,0x18,0x7E,0x66,0x1E,0xE6,0x07,0x66,0xE0,0x67,0x1E,0xF8,0x7F,0xE6,0x07,
0x66,0xE0,0x67,0x1E,0xF8,0x7F,0xE6,0x07,0x7E,0x06,0x18,0x80,0x87,0x9F,0x87,0x07,
0x7E,0x06,0x18,0x80,0x87,0x9F,0x87,0x07,0xE6,0xE1,0xE7,0x81,0xF9,0x07,0x1E,0x06,
0xE6,0xE1,0xE7,0x81,0xF9,0x07,0x1E,0x06,0x7E,0x86,0xFF,0x7F,0xFE,0xE7,0x1F,0x00,
0x7E,0x86,0xFF,0x7F,0xFE,0xE7,0x1F,0x00,0x98,0xE7,0xFF,0x61,0x66,0xF8,0x99,0x07,
0x98,0xE7,0xFF,0x61,0x66,0xF8,0x99,0x07,0x9E,0x01,0x60,0xFE,0x01,0x06,0x60,0x06,
0x9E,0x01,0x60,0xFE,0x01,0x06,0x60,0x06,0x66,0x7E,0x06,0x98,0x19,0x60,0x06,0x00,
0x66,0x7E,0x06,0x98,0x19,0x60,0x06,0x00,0xF8,0x87,0x01,0x98,0xE1,0xFF,0xF9,0x01,
0xF8,0x87,0x01,0x98,0xE1,0xFF,0xF9,0x01,0x66,0xF8,0x67,0xFE,0x99,0xFF,0x1F,0x00,
0x66,0xF8,0x67,0xFE,0x99,0xFF,0x1F,0x00,0x00,0x00,0x18,0x9E,0x19,0x06,0x86,0x01,
0x00,0x00,0x18,0x9E,0x19,0x06,0x86,0x01,0xFE,0x7F,0xE0,0xF9,0xE7,0x67,0x66,0x00,
0xFE,0x7F,0xE0,0xF9,0xE7,0x67,0x66,0x00,0x06,0x60,0xE6,0x01,0xE0,0x07,0x1E,0x00,
0x06,0x60,0xE6,0x01,0xE0,0x07,0x1E,0x00,0xE6,0x67,0x1E,0x18,0x78,0xFE,0xE7,0x07,
0xE6,0x67,0x1E,0x18,0x78,0xFE,0xE7,0x07,0xE6,0x67,0x98,0x99,0xE7,0x99,0xFF,0x01,
0xE6,0x67,0x98,0x99,0xE7,0x99,0xFF,0x01,0xE6,0x67,0x78,0x06,0x66,0x06,0x7E,0x06,
0xE6,0x67,0x78,0x06,0x66,0x06,0x7E,0x06,0x06,0x60,0x80,0xFF,0x99,0x01,0xE6,0x01,
0x06,0x60,0x80,0xFF,0x99,0x01,0xE6,0x01,0xFE,0x7F,0x1E,0x1E,0x98,0x19,0x06,0x00,
0xFE,0x7F,0x1E,0x1E,0x98,0x19,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};
void IIC_start()
{
	SCL_H;_delay_us(1);
	SDA_H;_delay_us(1);
	SDA_L;_delay_us(1);
	SCL_L;_delay_us(1);
}
void IIC_stop()
{
	SCL_L;_delay_us(1);
	SDA_L;_delay_us(1);
	SCL_H;_delay_us(1);
	SDA_H;_delay_us(1);
}
void write_IIC_byte(uchar IIC_byte)
{
	uchar i;
	for(i=0;i<8;i++)
	{
		if(IIC_byte&0x80){SDA_H;}
		else SDA_L;_delay_us(1);
		SCL_H;_delay_us(1);
		SCL_L;_delay_us(1);
		IIC_byte<<=1;
	}
	SDA_H;_delay_us(1);
	SCL_H;_delay_us(1);
	SCL_L;_delay_us(1);
}
void write_IIC_command(uchar command)
{
	IIC_start();
	write_IIC_byte(0x78);
	write_IIC_byte(0x00);
	write_IIC_byte(command);
	IIC_stop();
}
void write_IIC_data(uchar data)
{
	IIC_start();
	write_IIC_byte(0x78);
	write_IIC_byte(0x40);
	write_IIC_byte(data);
	IIC_stop();
}
void LY096BG30_init(void)
{

	write_IIC_command(0xae);                        //display off
	write_IIC_command(0xd5);write_IIC_command(0x80);//set display clock divide ratio/oscillator frequency
	write_IIC_command(0xa8);write_IIC_command(0x3f);//set multiplex ratio
	write_IIC_command(0xd3);write_IIC_command(0x00);//set display offset
	write_IIC_command(0x40);						//set display start line
	write_IIC_command(0x8d);write_IIC_command(0x14);//set charge pump
	write_IIC_command(0xd8);write_IIC_command(0x05);//set low power diaplay mode
	write_IIC_command(0xa1);						//set segment re-map
	write_IIC_command(0xc8);						//set COM output scan direction
	write_IIC_command(0xda);write_IIC_command(0x12);//set COM pins hardware configuration
	write_IIC_command(0x81);write_IIC_command(0xcf);//设置对比度
	write_IIC_command(0xd9);write_IIC_command(0xf1);//set pre-charge period
	write_IIC_command(0xdb);write_IIC_command(0x40);//set VCOMH deselect level
	write_IIC_command(0xa4);						//set entire display on/off
	write_IIC_command(0xa6);						//set normal/inverse display
}
void display_clear(void)
{
	uchar x,y;
	for(y=0;y<8;y++)
    {
		write_IIC_command(0xb0+y);
		write_IIC_command(0x00);
		write_IIC_command(0x10);
		for(x=0;x<128;x++)
		{
			write_IIC_data(0x00);
		}
    }
}
void display_char(uchar x,uchar y,uchar c)//x:0~127 y:0~6 c:0~15
{
	uchar i;
	write_IIC_command(0xb0+y);
	write_IIC_command(x&0x0f);
	write_IIC_command(0x10+(x>>4));
	for(i=0;i<8;i++)
	{
		write_IIC_data(pgm_read_byte(&tab_16_8[c][i<<1]));
	}
	write_IIC_command(0xb1+y);
	write_IIC_command(x&0x0f);
	write_IIC_command(0x10+(x>>4));
	for(i=0;i<8;i++)
	{
		write_IIC_data(pgm_read_byte(&tab_16_8[c][(i<<1)+1]));
	}
}
void display_chinese_char(uchar x,uchar y,uchar c)//x:0~127 y:0~6 c:0~15
{
	uchar i;
	write_IIC_command(0xb0+y);
	write_IIC_command(x&0x0f);
	write_IIC_command(0x10+(x>>4));
	for(i=0;i<16;i++)
	{
		write_IIC_data(pgm_read_byte(&tab_16_16[c][i<<1]));
	}
	write_IIC_command(0xb1+y);
	write_IIC_command(x&0x0f);
	write_IIC_command(0x10+(x>>4));
	for(i=0;i<16;i++)
	{
		write_IIC_data(pgm_read_byte(&tab_16_16[c][(i<<1)+1]));
	}
}
void display_power(uchar power)
{
	uchar i;
	uchar x=96;
	write_IIC_command(0xb0);
	write_IIC_command(x&0x0f);
	write_IIC_command(0x10+(x>>4));
	for(i=0;i<32;i++)
	{
		write_IIC_data(pgm_read_byte(&tab_power[power][i<<1]));
	}
	write_IIC_command(0xb1);
	write_IIC_command(x&0x0f);
	write_IIC_command(0x10+(x>>4));
	for(i=0;i<32;i++)
	{
		write_IIC_data(pgm_read_byte(&tab_power[power][(i<<1)+1]));
	}
}
void display_picture(void)
{
	uint i,j;
	for(i=0;i<8;i++)
	{
		write_IIC_command(0xb0+i);
		write_IIC_command(0x00);
		write_IIC_command(0x10);
		for(j=0;j<128;j++)
		{
			write_IIC_data(pgm_read_byte(&tab_picture1[(j<<3)+i]));
		}
	}
}
void OLED_init(void)
{
	RES_O;
	RES_L;
	_delay_ms(10);
	RES_H;
	_delay_ms(10);
	write_IIC_command(0xAE);   //display off

	LY096BG30_init();
	_delay_ms(5);

	display_clear();
	_delay_ms(100);
//	write_IIC_command(0xaf);
}
