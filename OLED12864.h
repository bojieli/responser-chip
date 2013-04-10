#ifndef _OLED12864_H
#define _OLED12864_H

#define uchar unsigned char
#define display_off write_IIC_command(0xae);
#define display_on write_IIC_command(0xaf);

void write_IIC_command(uchar command);
void display_clear(void);
void display_char(uchar x,uchar y,uchar c);//x:0~127 y:0~6 c:0~15
void display_chinese_char(uchar x,uchar y,uchar c);//x:0~127 y:0~6 c:0~15
void display_power(uchar power);//0~8
void display_sending(void);
void display_picture(void);
void OLED_init(void);


#endif
