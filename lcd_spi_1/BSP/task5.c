#include "board.h"
#include "key.h"
#include "lcd.h"
#include "lcd_init.h"
#include "motor.h"
#include "task5.h"
#include <stdio.h>

static void task5_apply_beep(bool beep_on)
{
    if (beep_on) {
        LED1_ON();
        BEEP_ON();
    }
    else {
        LED1_OFF();
        BEEP_OFF();
    }
}

static void task5_show(bool beep_on, uint16_t alarm_count)
{
    char line[24];

    LCD_ShowString(0, 0, "                ", BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 20, "                ", BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 40, "                ", BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 60, "                ", BLACK, WHITE, 16, 0);

    LCD_ShowString(0, 0, "TASK5 ALARM", BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 20, "PB18+PA2", BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 40, "AUTO 0.5s", BLACK, WHITE, 16, 0);

    sprintf(line, "%s C:%u", beep_on ? "BEEP ON " : "BEEP OFF", alarm_count);
    LCD_ShowString(0, 60, line, BLACK, WHITE, 16, 0);
}

void task5(void)
{
    bool beep_on = false;
    uint16_t tick_count = 0;
    uint16_t alarm_count = 0;

    SET_MOTORS_SPEED(0, 0);
    alarm_enabled = 0;
    alarm_timer_count = 0;
    task5_apply_beep(beep_on);
    task5_show(beep_on, alarm_count);

    while (1) {
        KeyState key = get_key_state();

        if (key == KEY_K2) {
            alarm_enabled = 0;
            alarm_timer_count = 0;
            LED1_OFF();
            BEEP_OFF();
            SET_MOTORS_SPEED(0, 0);
            page_flag = 1;
            break;
        }

        if (key == KEY_K3) {
            alarm_count++;
            Start_Alarm();
            task5_show(true, alarm_count);
        }
        else if (key == KEY_K1) {
            alarm_enabled = 0;
            alarm_timer_count = 0;
            beep_on = !beep_on;
            task5_apply_beep(beep_on);
            task5_show(beep_on, alarm_count);
            tick_count = 0;
        }

        if (++tick_count >= 50U) {
            tick_count = 0;
            alarm_enabled = 0;
            alarm_timer_count = 0;
            beep_on = !beep_on;
            task5_apply_beep(beep_on);
            task5_show(beep_on, alarm_count);
        }

        delay_ms(10);
    }
}
