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
// #include "bsp_sg90.h"
#include "car_control.h"
#include "test.h"
#include "task1.h"
#include "task2.h"
#include "task3.h"
#include "task4.h"
#include "encoder.h"
#include <stdint.h>
#include <stdio.h>

int main(void)
{
	 
	//初始化
    SYSCFG_DL_init();
    LED1_ON();
	// Development board initialization
	board_init();
    // 初始化电机
    motor_init();
    //编码器初始化
    encoder_init();
    //imu初始化
    calibrate_z_axis_zero();
    // LCD Initializatio
    delay_ms(300);   // ⭐关键：等系统完全稳定

// ⭐关键：强制LCD复位（必须）
    DL_GPIO_clearPins(RES_PORT, RES_PIN_20_PIN);
    delay_ms(50);

    DL_GPIO_setPins(RES_PORT, RES_PIN_20_PIN);
    delay_ms(50);

// ⭐关键：再次延时（防止SPI时序错位）
    delay_ms(100);
	LCD_Init();
    LCD_Fill(0,0,LCD_W,LCD_H,WHITE);	
    LCD_ShowString(10,20,"Gogogo",BLACK,WHITE,16,0);
     //任务列表，任务计数器
    const uint8_t task_count = 6;
    void (*tasks[])(void) = {test_1, test_2, task1, task2, task3, task4};
    uint8_t current_task_index = 0;
    uint8_t last_current_task_index = 0;

	char buf_index[32];
            sprintf(buf_index, "test: %d\r\n", current_task_index+1); // 将值转换为字符串
            LCD_Fill(0,0,LCD_W,LCD_H,WHITE);	
            LCD_ShowString(10, 30, buf_index, BLACK, WHITE, 16, 0); // 显示字符串

	while (1)
	{

        KeyState key = get_key_state();
        last_current_task_index = current_task_index;
        switch (key)
        {
            case KEY_K1: // 上一个
                page_flag = 0;
                current_task_index = (current_task_index + task_count - 1) % task_count;
                break;

            case KEY_K3: // 下一个
                page_flag = 0;
                current_task_index = (current_task_index + 1) % task_count;
                break;

            case KEY_K2: // 确定
                if(page_flag == 0)
                tasks[current_task_index]();
                break;
                
            default:
                break;

        }
        if(current_task_index != last_current_task_index)
        {
            switch (current_task_index) {
                case 0:
                case 1:
                    sprintf(buf_index, "test: %d\r\n", current_task_index + 1);
                    break;
                default:
                    sprintf(buf_index, "task: %d\r\n", current_task_index - 1);
                    break;
            }
                        
            LCD_Fill(0,0,LCD_W,LCD_H,WHITE);	
            LCD_ShowString(10, 30, buf_index, BLACK, WHITE, 16, 0); // 显示字符串
        }
    }     
    
}

void TIMER_0_INST_IRQHandler(void) {
    static uint16_t count_10ms = 0;
    static uint16_t count_100ms = 0;
    static uint16_t alarm_timer_count = 0;
    switch (DL_TimerA_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMERA_IIDX_LOAD:
        //循迹计时
            if (tracing_enabled) {
                tracing_timer_count++;
                if (tracing_timer_count >= tracing_duration_ms) {
                    tracing_enabled = 0;
                    SET_MOTORS_SPEED(0, 0);
                }
            }
        //声光计时器
            if (alarm_enabled) {
                if (++alarm_timer_count >= 500) //0.5s
                {
                    alarm_timer_count = 0;
                    alarm_enabled = 0;
                    LED1_OFF();
                    BEEP_OFF();
                }
            }
            
        // 10ms任务
            if (++count_10ms >= 10) {
                count_10ms = 0;
                //灰度数据获取
                Get_Huidu();


                //开启沿Target_angle直行
                //启动示例
                /*
                *Target_angle = 90;//修改目标角度，（不是所有调用都要修改的）
                *GoS_begin(speed,distance_limit)
                */
                //关闭示例
                //超出距离限制自动关闭并清零相关数据,也可以在外部根据条件关闭
                /*
                *GoS_stop();
                */
                if (Car_GoStraight_flag) {
                    if (Measure_Distance <= Distance_limit ) 
                    {
                        Car_GoStraight(GoStraight_speed);
                    }
                    else {
                        GoS_stop();
                    }
                }
                
                //开启循迹
                /*
                *Trc_begin(speed, distance);
                *Trc_stop();
                */
                if (Car_Tracing_flag) {
                    if (Measure_Distance <= Distance_limit ) 
                    {
                         Car_Tracing(Tracing_speed);
                    }
                    else {
                        Trc_stop();
                    }
                }
                
                
            }

        //100ms任务
            if (++count_100ms >= 100) {
                count_100ms = 0;
                MEASURE_MOTORS_SPEED();
            }
            break;
        default: break;
    }
}
