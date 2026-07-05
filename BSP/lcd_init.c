#include "lcd_init.h"
#include "board.h"

void LCD_GPIO_Init(void)
{
//	GPIO_InitTypeDef  GPIO_InitStructure;
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能A端口时钟 Enable port A clock
//	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6|GPIO_Pin_7;	 
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出 Push-pull output
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz Speed ??50MHz
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);	  //初始化GPIOA Initialize GPIOA
// 	GPIO_SetBits(GPIOA,GPIO_Pin_6|GPIO_Pin_7);
//	 
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能A端口时钟 Enable port A clock
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_10|GPIO_Pin_11;	 
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出 Push-pull output
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz Speed ??50MHz
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);	  //初始化GPIOA Initialize GPIOA
// 	GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_10|GPIO_Pin_11);
}


/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
	  Function description: LCD serial data writing function
	  Input data: dat serial data to be written
	  Return value: None
******************************************************************************/
void LCD_Writ_Bus(uint8_t dat) 
{	
	uint8_t i;
	LCD_CS_Clr();
	for(i=0;i<8;i++)
	{			  
		LCD_SCLK_Clr();
		if(dat&0x80)
		{
		   LCD_MOSI_Set();
		}
		else
		{
		   LCD_MOSI_Clr();
		}
		LCD_SCLK_Set();
		dat<<=1;
	}	
  LCD_CS_Set();	
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
	  Function description: LCD writes data
	  Input data: dat written data
	  Return value: None
******************************************************************************/
void LCD_WR_DATA8(uint8_t dat)
{
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
	  Function description: LCD writes data
	  Input data: dat written data
	  Return value: None
******************************************************************************/
void LCD_WR_DATA(uint16_t dat)
{
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
	  Function description: LCD write command
	  Input data: dat write command
	  Return value: None
******************************************************************************/
void LCD_WR_REG(uint8_t dat)
{
	LCD_DC_Clr();//写命令 Write Command
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//写数据 Writing Data
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
	  Function description: Set the start and end address
	  Input data: x1, x2 Set the start and end address of the column
				  y1, y2 Set the start and end address of the row
	  Return value: None
******************************************************************************/
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);//列地址设置 Column address setting
		LCD_WR_DATA(x1+26);
		LCD_WR_DATA(x2+26);
		LCD_WR_REG(0x2b);//行地址设置 Row address setting
		LCD_WR_DATA(y1+1);
		LCD_WR_DATA(y2+1);
		LCD_WR_REG(0x2c);//储存器写 Memory Write
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);//列地址设置 Column address setting
		LCD_WR_DATA(x1+26);
		LCD_WR_DATA(x2+26);
		LCD_WR_REG(0x2b);//行地址设置 Row address setting
		LCD_WR_DATA(y1+1);
		LCD_WR_DATA(y2+1);
		LCD_WR_REG(0x2c);//储存器写 Memory Write
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);//列地址设置 Column address setting
		LCD_WR_DATA(x1+1);
		LCD_WR_DATA(x2+1);
		LCD_WR_REG(0x2b);//行地址设置 Row address setting
		LCD_WR_DATA(y1+26);
		LCD_WR_DATA(y2+26);
		LCD_WR_REG(0x2c);//储存器写 Memory Write
	}
	else
	{
		LCD_WR_REG(0x2a);//列地址设置 Column address setting
		LCD_WR_DATA(x1+1);
		LCD_WR_DATA(x2+1);
		LCD_WR_REG(0x2b);//行地址设置 Row address setting
		LCD_WR_DATA(y1+26);
		LCD_WR_DATA(y2+26);
		LCD_WR_REG(0x2c);//储存器写 Memory Write
	}
}

void LCD_Init(void)
{
	LCD_GPIO_Init();//初始化GPIO	 Initialize GPIO
	LCD_RES_Clr();//复位 Reset
	delay_ms(100);
	LCD_RES_Set();
	delay_ms(100);	
//	LCD_BLK_Set();//打开背光 Turn on backlight
	LCD_BLK_Clr();

	delay_ms(100);	
	LCD_WR_REG(0x11);     //Sleep out
	delay_ms(120);                //Delay 120ms
	LCD_WR_REG(0xB1);     //Normal mode
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_REG(0xB2);     //Idle mode
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_REG(0xB3);     //Partial mode
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_REG(0xB4);     //Dot inversion
	LCD_WR_DATA8(0x03);   
	LCD_WR_REG(0xC0);     //AVDD GVDD
	LCD_WR_DATA8(0xAB);   
	LCD_WR_DATA8(0x0B);   
	LCD_WR_DATA8(0x04);   
	LCD_WR_REG(0xC1);     //VGH VGL
	LCD_WR_DATA8(0xC5);   //C0
	LCD_WR_REG(0xC2);     //Normal Mode
	LCD_WR_DATA8(0x0D);   
	LCD_WR_DATA8(0x00);   
	LCD_WR_REG(0xC3);     //Idle
	LCD_WR_DATA8(0x8D);   
	LCD_WR_DATA8(0x6A);   
	LCD_WR_REG(0xC4);     //Partial+Full
	LCD_WR_DATA8(0x8D);   
	LCD_WR_DATA8(0xEE);   
	LCD_WR_REG(0xC5);     //VCOM
	LCD_WR_DATA8(0x0F);   
	LCD_WR_REG(0xE0);     //positive gamma
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x10);   
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x02);   
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x0F);   
	LCD_WR_DATA8(0x25);   
	LCD_WR_DATA8(0x36);   
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x04);   
	LCD_WR_DATA8(0x10);   
	LCD_WR_REG(0xE1);     //negative gamma
	LCD_WR_DATA8(0x0A);   
	LCD_WR_DATA8(0x0D);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x0F);   
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x02);   
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x0F);   
	LCD_WR_DATA8(0x25);   
	LCD_WR_DATA8(0x35);   
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x04);   
	LCD_WR_DATA8(0x10);		 
	LCD_WR_REG(0xFC);    
	LCD_WR_DATA8(0x80);  		
	LCD_WR_REG(0x3A);     
	LCD_WR_DATA8(0x05);   
	LCD_WR_REG(0x36);
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x08);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC8);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x78);
	else LCD_WR_DATA8(0xA8);   
	LCD_WR_REG(0x21);     //Display inversion
	LCD_WR_REG(0x29);     //Display on
	LCD_WR_REG(0x2A);     //Set Column Address
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0x1A);  //26  
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0x69);   //105 
	LCD_WR_REG(0x2B);     //Set Page Address
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0x01);    //1
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0xA0);    //160
	LCD_WR_REG(0x2C); 
	
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);	//初始化成白屏 Initialize to white screen
	

}








