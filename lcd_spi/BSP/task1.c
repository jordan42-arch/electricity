#include "encoder.h"
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
// #include "bsp_sg90.h"
#include "test.h"
#include "task1.h"
//#include <cstdint>
#include <stdint.h>
#include <stdio.h>

void task1(void){
    LCD_Fill(0, 20, 120, 40, WHITE); // 清除上一次显示区域
    LCD_ShowString(10,20,"task1",BLACK,WHITE,16,0);
    int circle_num = 1;//需求转弯数
    int circle_count = 0;
    int sure = 0;//确认运动键
    int turn_f = 0;//转弯阶段判断
    int turn_direction = 0;//-1:左转，1:右转，0:未识别


    //Trc_begin(50, 400);
    
    while(1)
    {
         if (sure == 1) {//确认运动
           
            if (turn_f == 0) {//直线阶段，加速
                Car_Tracing(45);
                if (Measure_Distance >= 45) {
                    Reset_Distance();
                    turn_f = 1;
                }
            }
            else if (turn_f == 1) {//转弯前减速
                Car_Tracing(40);
                if (Huidu_Flag == 3) {
                    turn_direction = -1;
                    Reset_Distance();
                    turn_f = 2;
                }
                else if (Huidu_Flag == 2) {
                    turn_direction = 1;
                    Reset_Distance();
                    turn_f = 2;
                }
            }
            else if (turn_f == 2) {//稳定转弯通过后，切换为直线阶段
                if (turn_direction < 0) {
                    int left_turn_speed =
                        Measure_Distance < 2 ? 50 : 40;
                    SET_MOTORS_SPEED(0, left_turn_speed);
                }
                else if (turn_direction > 0) {
                    int right_turn_speed =
                        Measure_Distance < 2 ? 75 : 55;
                    SET_MOTORS_SPEED(right_turn_speed, 0);
                }
                else {
                    SET_MOTORS_SPEED(0, 0);
                }
                if ((Measure_Distance >= 3 &&
                     Huidu_Flag == 1 &&
                     Huidu_Error >= -1 &&
                     Huidu_Error <= 1) ||
                    Measure_Distance >= 20) {
                    Reset_Distance();
                    turn_f = 0;
                    turn_direction = 0;
                    circle_count++;//每次转弯后圈数计数加一
                }
            }
        }
        else {
            SET_MOTORS_SPEED(0, 0);
        }
       if (circle_count == 4*circle_num) {
        sure = 0;
        circle_count = 0;
       }

        char buf_imu[32];
        sprintf(buf_imu, "cn %i su %i\r\n", circle_num , sure); // 将偏移值转换为字符串
        LCD_ShowString(10, 40, buf_imu, BLACK, WHITE, 16, 0); // 显示字符串
        //检测按键是否切换任务
        KeyState key = get_key_state();
        if (key == KEY_K1) {
            circle_num += 1;//设置圈数
            if (circle_num > 5) {
                circle_num = 1;//循环圈数
            }
        }
        else if (key == KEY_K3) {
            sure += 1;//可以跑了
            if (sure > 1) {
                sure = 0;
            }
        }
        else 
        if (key == KEY_K2) {
            page_flag = 1;
            sure = 0;
            SET_MOTORS_SPEED(0,0);
            break; // 如果 K2 ，则退出 test 模式
        }
    }
}
