#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "board.h"
#include "lcd_init.h"
#include "lcd.h"
#include "imu.h"
#include "key.h"
#include "grey.h"
#include "motor.h"
#include "k210_use.h"
#include "car_control.h"
#include "test.h"
#include "task3.h"
//#include <cstdint>
#include <stdint.h>
#include <stdio.h>


void task3(void){
    LCD_Fill(0, 20, 120, 40, WHITE); // 清除上一次显示区域
    LCD_ShowString(10,20,"task3",BLACK,WHITE,16,0);
    //变量

    while(1)
    {
        



        //imu显示
        char buf_imu[32];
        // sprintf(buf_imu, "CW %f\r\n", color_width); // 将偏移值转换为字符串
        // LCD_ShowString(10, 40, buf_imu, BLACK, WHITE, 16, 0); // 显示字符串
        
        
        // 检测按键是否切换任务
        KeyState key = get_key_state();
        if (key == KEY_K2) {
            page_flag = 1;
            SET_MOTORS_SPEED(0,0);
            break; // 如果 K2 ，则退出 test 模式
        }
    }
}