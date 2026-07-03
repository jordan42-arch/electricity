#ifndef __LCD_INIT_H
#define __LCD_INIT_H


#define USE_HORIZONTAL 1  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏 Set the screen to horizontal or vertical. 0 or 1 for vertical screen, 2 or 3 for horizontal screen.


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1

#define LCD_W 80
#define LCD_H 160

#else
#define LCD_W 160
#define LCD_H 80
#endif



//-----------------LCD端口定义 LCD Port Definition---------------- 

//#define LCD_SCLK_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_0)//SCL=SCLK
//#define LCD_SCLK_Set() GPIO_SetBits(GPIOA,GPIO_Pin_0)

//#define LCD_MOSI_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_1)//SDA=MOSI
//#define LCD_MOSI_Set() GPIO_SetBits(GPIOA,GPIO_Pin_1)

//#define LCD_RES_Clr()  GPIO_ResetBits(GPIOA,GPIO_Pin_2)//RES
//#define LCD_RES_Set()  GPIO_SetBits(GPIOA,GPIO_Pin_2)

//#define LCD_DC_Clr()   GPIO_ResetBits(GPIOA,GPIO_Pin_3)//DC
//#define LCD_DC_Set()   GPIO_SetBits(GPIOA,GPIO_Pin_3)
// 		     
//#define LCD_CS_Clr()   GPIO_ResetBits(GPIOA,GPIO_Pin_4)//CS
//#define LCD_CS_Set()   GPIO_SetBits(GPIOA,GPIO_Pin_4)

//#define LCD_BLK_Clr()  GPIO_ResetBits(GPIOA,GPIO_Pin_5)//BLK
//#define LCD_BLK_Set()  GPIO_SetBits(GPIOA,GPIO_Pin_5)


#define LCD_SCLK_Clr() DL_GPIO_clearPins(SCLK_PORT,SCLK_PIN_31_PIN)//SCL=SCLK
#define LCD_SCLK_Set() DL_GPIO_setPins(SCLK_PORT,SCLK_PIN_31_PIN)

#define LCD_MOSI_Clr() DL_GPIO_clearPins(MOSI_PORT,MOSI_PIN_28_PIN)//SDA=MOSI
#define LCD_MOSI_Set() DL_GPIO_setPins(MOSI_PORT,MOSI_PIN_28_PIN)

#define LCD_RES_Clr()  DL_GPIO_clearPins(RES_PORT,RES_PIN_20_PIN)//RES
#define LCD_RES_Set()  DL_GPIO_setPins(RES_PORT,RES_PIN_20_PIN)

#define LCD_DC_Clr()   DL_GPIO_clearPins(DC_PORT,DC_PIN_24_PIN)//DC
#define LCD_DC_Set()   DL_GPIO_setPins(DC_PORT,DC_PIN_24_PIN)
 		     
#define LCD_CS_Clr()   DL_GPIO_clearPins(CS_PORT,CS_PIN_23_PIN)//CS
#define LCD_CS_Set()  DL_GPIO_clearPins(CS_PORT,CS_PIN_23_PIN) //DL_GPIO_setPins(CS_PORT,CS_PIN_23_PIN)

#define LCD_BLK_Clr()  DL_GPIO_clearPins(BLK_PORT,BLK_PIN_19_PIN)//BLK
#define LCD_BLK_Set()  DL_GPIO_setPins(BLK_PORT,BLK_PIN_19_PIN)

#include <stdint.h>
#include "lcd.h"

void LCD_GPIO_Init(void);//初始化GPIO Initialize GPIO
void LCD_Writ_Bus(uint8_t dat);//模拟SPI时序 Simulate SPI timing
void LCD_WR_DATA8(uint8_t dat);//写入一个字节 Write a byte
void LCD_WR_DATA(uint16_t dat);//写入两个字节 Write two bytes
void LCD_WR_REG(uint8_t dat);//写入一个指令 Write a command
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);//设置坐标函数 Set coordinate function
void LCD_Init(void);//LCD初始化 LCD initialization
#endif




