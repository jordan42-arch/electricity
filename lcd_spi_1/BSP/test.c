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
#include "encoder.h"
#include "test.h"
//#include <cstdint>
#include <stdint.h>
#include <stdio.h>

// 封装显示灰度传感器信息的函数
void display_grey_info(int y_position) {
    char buf_grey_error[32];
    char buf_grey_flag[32];
    
    sprintf(buf_grey_error, "g_e: %d\r\n", Huidu_Error);
    
    switch (Huidu_Flag) {
        case 0:
            strcpy(buf_grey_flag, "none");
            break;
        case 1:
            strcpy(buf_grey_flag, "line");
            break;
        case 2:
            strcpy(buf_grey_flag, "R_T");
            break;
        case 3:
            strcpy(buf_grey_flag, "L_T");
            break;
        default:
            buf_grey_flag[0] = '\0'; // 空字符串
            break;
    }
    
    LCD_ShowString(10, y_position, buf_grey_error, BLACK, WHITE, 16, 0);
    LCD_ShowString(10, y_position+20, buf_grey_flag, BLACK, WHITE, 16, 0);
}

// 封装显示IMU信息的函数
void display_imu_info(int y_position) {
    char buf_imu[32];
    sprintf(buf_imu, "yaw %f\r\n", imu_data.yaw);
    LCD_ShowString(10, y_position, buf_imu, BLACK, WHITE, 16, 0);
}

// 封装显示红灯状态的函数
void display_red_state(void) {
    get_red_state();
    if (red_flag) {
        LCD_ShowString(10, 100, "drug_get   ", BLACK, WHITE, 16, 0);
    } else {
        LCD_ShowString(10, 100, "drug_remove", BLACK, WHITE, 16, 0);
    }
}


void test_1(void) {

    while (1) {
        LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
        LCD_ShowString(10, 10, "gogogo", BLACK, WHITE, 16, 0);
        
        // 显示灰度传感器信息
        display_grey_info(30);
        // 显示IMU信息
        display_imu_info(70);
        //显示路程
        char buf_imu[32];
        sprintf(buf_imu, "x %f\r\n", Measure_Distance);
        LCD_ShowString(10, 90, buf_imu, BLACK, WHITE, 16, 0);
        sprintf(buf_imu, "x1 %f\r\n", Motor1_Lucheng);
        LCD_ShowString(10, 110, buf_imu, BLACK, WHITE, 16, 0);
        sprintf(buf_imu, "x1 %f\r\n", Motor2_Lucheng);
        LCD_ShowString(10, 130, buf_imu, BLACK, WHITE, 16, 0);
        // 显示红灯状态
       // display_red_state();
        
        // 检查是否需要退出
         //检测按键是否切换任务
            KeyState key = get_key_state();
            if (key == KEY_K2) {
                page_flag = 1;
                break; // 如果 K2 ，则退出 test 模式
            }
    }
}

// 封装PID参数调整处理函数
void handle_pid_adjustment(void) {
    while (1) {
        KeyState key_1 = get_key_state();
        if (key_1 == KEY_K1) { // 增加参数
            AdjustPIDParam(0);
            DisplayPIDInfo();
            delay_ms(200);
        }
        if (key_1 == KEY_K2) { // 减少参数
            AdjustPIDParam(1);
            DisplayPIDInfo();
            delay_ms(200);
        }
        if (key_1 == KEY_K3) { // 确认修改
            if (current_param == 3) {
                pid_page = 1;
            }
            if (current_param != 3) {
                pid_page = 0;
            }
            DisplayPIDInfo();
            delay_cycles(200);
            break;
        }
    }
}

// 优化后的 test_2 函数
void test_2(void) {
    DisplayPIDInfo(); // 初始化显示
    
    while (1) {
        // 检测按键是否切换任务
        KeyState key = get_key_state();
        if (key == KEY_K1) { // 切换 PID
            SelectNextPID();
            DisplayPIDInfo();
            delay_ms(200);
        }
        if (key == KEY_K3 && pid_page == 0) { // 切换参数项，解锁
            pid_page = 1; // pid参数调节页
            SelectNextParam();
            DisplayPIDInfo();
            handle_pid_adjustment();
        }
        if (key == KEY_K2) { // 结束调参
            current_param = 3; // 参数锁定
            page_flag = 1;
            delay_ms(1000);
            break;
        }
    }
}



