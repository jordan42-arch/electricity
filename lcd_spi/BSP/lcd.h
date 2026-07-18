#ifndef __LCD_H
#define __LCD_H		
#include <stdint.h>


void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);//指定区域填充颜色 Specify area fill color
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color);//在指定位置画一个点 Draw a point at the specified location
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color);//在指定位置画一条线 Draw a line at the specified position
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);//在指定位置画一个矩形 Draw a rectangle at the specified location
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color);//在指定位置画一个圆 Draw a circle at the specified location

void LCD_ShowChinese(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示汉字串 Display Chinese character string
void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示单个12x12汉字 Display a single 12x12 Chinese character
void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示单个16x16汉字 Display a single 16x16 Chinese character
void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示单个24x24汉字 Display a single 24x24 Chinese character
void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示单个32x32汉字 Display a single 32x32 Chinese character

void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示一个字符 Display a character
void LCD_ShowString(uint16_t x,uint16_t y,const char *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示字符串 Display String
uint32_t mypow(uint8_t m,uint8_t n);//求幂 Power
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey);//显示整数变量 Display integer variables
void LCD_ShowFloatNum1(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey);//显示两位小数变量 Display variables with two decimal places

void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[]);//显示图片 Show image


//画笔颜色 Brush Color
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //棕色  Brown
#define BRRED 			 0XFC07 //棕红色 Brown red
#define GRAY  			 0X8430 //灰色 Gray
#define DARKBLUE      	 0X01CF	//深蓝色 Dark blue
#define LIGHTBLUE      	 0X7D7C	//浅蓝色 Light blue
#define GRAYBLUE       	 0X5458 //灰蓝色 Gray blue
#define LIGHTGREEN     	 0X841F //浅绿色 Light green
#define LGRAY 			 0XC618 //浅灰色(PANNEL),窗体背景色 Light gray (PANNEL), window background color
#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色) Light gray blue (middle layer color)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色) Light brown blue (inverted color of selected item)

#endif





