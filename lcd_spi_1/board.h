#ifndef	__BOARD_H__
#define __BOARD_H__

#include "ti_msp_dl_config.h"
#include "imu.h"
#include "k210_use.h"

void board_init(void);
void Start_Alarm(void);

void delay_us(unsigned long __us);
void delay_ms(unsigned long ms);
void delay_1us(unsigned long __us);
void delay_1ms(unsigned long ms);

void uart0_send_char(char ch);
void uart0_send_string(char* str);
void uart0_send_int(int num);      // 发送整数到串口
char *reverse(char *s);            // 反转字符串
char *my_itoa(int n);              // int转char*字符串

uint8_t HAL_UART_Transmit( uint8_t *pData, uint16_t Size, uint32_t Timeout);

void uart3_send_char(char ch);
void usart3_send_bytes(unsigned char *buf, int len);
void usart3_send_byte(unsigned char byte);
void send_imu_data(void);

/*声光提示-灯和喇叭*/
#define LED1_ON()		DL_GPIO_setPins(LED_PORT, LED_LED1_PIN)
#define LED1_OFF()		DL_GPIO_clearPins(LED_PORT, LED_LED1_PIN)
#define LED1_TOGGLE()	DL_GPIO_togglePins(LED_PORT, LED_LED1_PIN)

#define BEEP_ON()		DL_GPIO_setPins(BEEP_PORT,BEEP_PIN1_PIN)
#define BEEP_OFF()		DL_GPIO_clearPins(BEEP_PORT,BEEP_PIN1_PIN)
#define BEEP_Toggle()	DL_GPIO_togglePins(BEEP_PORT,BEEP_PIN1_PIN)


/*------------------------------------------------
                将config.h的内容迁移进来
--------------------------------------------------*/
#include "stdbool.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

// 定义状态枚举
typedef enum {
        STATE_A,    // A点状态
        STATE_B,    // B点状态
        STATE_C,    // C点状态
        STATE_D,    // D点状态
        STATE_A_END // 回到A点结束状态
} RunModeState;

extern bool alarm_enabled;

#define u8  unsigned char
#define u16 unsigned int
#define u32 unsigned long

extern float Basic_Speed;
extern uint8_t OLED_View_Select;
extern uint8_t Car_Mode;
extern uint8_t page_flag;

#define Run_Mode				0		//比赛模式
#define Speed_Mode 			1		//速度环
#define Turn_Mode				2		//转向环 灰度
#define Distance_Mode		3		//距离环
#define Gyro_Mode				4		//角速度环
#define Angle_Mode			5		//角度环


#endif
