#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "board.h"
#include "car_control.h"
#include "encoder.h"
#include "grey.h"
#include "key.h"
#include "lcd.h"
#include "lcd_init.h"
#include "motor.h"
#include "task3.h"
#include "task4.h"
#include <stdbool.h>
#include <stdint.h>

#define TASK3_STRAIGHT_MAX_CM      120.0f
#define TASK3_LINE_MAX_CM          200.0f
#define TASK3_ARC_EXIT_DISTANCE_CM  16.0f

#define TASK3_STRAIGHT_LEFT_SPEED   56
#define TASK3_STRAIGHT_RIGHT_SPEED  52
#define TASK3_CURVE_SPEED           50
#define TASK3_CURVE_STEER_SCALE      1.50f
#define TASK3_CURVE_HOLD_FILTER      0.10f
#define TASK3_ENCODER_DISTANCE_KP    4.0f
#define TASK3_ENCODER_SPEED_KP       0.35f
#define TASK3_MAX_CORRECTION        25.0f
#define TASK3_STRAIGHT_SETTLE_TICKS  3U
#define TASK3_LINE_LOST_CONFIRM_TICKS 3U

typedef enum {
    TASK3_WAIT = 0,
    TASK3_STRAIGHT,
    TASK3_FOLLOW_LINE,
    TASK3_ARC_EXIT_ALIGN,
    TASK3_FINISHED,
    TASK3_ERROR
} Task3State;

static int task3_limit_speed(int speed)
{
    if (speed > 100) {
        return 100;
    }
    if (speed < 0) {
        return 0;
    }
    return speed;
}

static void task3_reset_encoder_control(void)
{
    gEncoderCount_L = 0;
    gEncoderCount_R = 0;
    Motor1_Speed = 0.0f;
    Motor2_Speed = 0.0f;
    Reset_Distance();
}

static void task3_encoder_straight(int left_base,
                                   int right_base,
                                   uint8_t *settle_ticks)
{
    float distance_error = Motor2_Lucheng - Motor1_Lucheng;
    float speed_error = 0.0f;

    if (*settle_ticks > 0U) {
        (*settle_ticks)--;
    }
    else {
        speed_error = Motor2_Speed - Motor1_Speed;
    }

    float correction =
        TASK3_ENCODER_DISTANCE_KP * distance_error +
        TASK3_ENCODER_SPEED_KP * speed_error;

    if (correction > TASK3_MAX_CORRECTION) {
        correction = TASK3_MAX_CORRECTION;
    }
    else if (correction < -TASK3_MAX_CORRECTION) {
        correction = -TASK3_MAX_CORRECTION;
    }

    int left_speed =
        task3_limit_speed(left_base + (int)correction);
    int right_speed =
        task3_limit_speed(right_base - (int)correction);

    SET_MOTORS_SPEED(left_speed, right_speed);
}

static void task3_curve_drive(int base_speed, float turn_speed)
{
    SET_MOTORS_SPEED(
        task3_limit_speed(base_speed + (int)turn_speed),
        task3_limit_speed(base_speed - (int)turn_speed));
}

static float task3_curve_tracing(int base_speed)
{
    float turn_speed =
        PID_Calculate(&pid_Turn, Huidu_Error, Huidu_Target) *
        TASK3_CURVE_STEER_SCALE;

    task3_curve_drive(base_speed, turn_speed);
    return turn_speed;
}

static void task3_show_state(const char *state_name)
{
    LCD_ShowString(0, 20, "          ", BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 40, "          ", BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 20, "TASK3 LAP", BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 40, state_name, BLACK, WHITE, 16, 0);
}

static void task3_start_straight(Task3State *state,
                                 bool *straight_armed,
                                 uint8_t *straight_settle_ticks,
                                 const char *state_name)
{
    SET_MOTORS_SPEED(0, 0);
    task3_reset_encoder_control();
    *straight_armed = false;
    *straight_settle_ticks = TASK3_STRAIGHT_SETTLE_TICKS;
    *state = TASK3_STRAIGHT;
    task3_show_state(state_name);
}

static void task3_start_line(Task3State *state,
                             float *last_black_distance,
                             float *held_turn_speed,
                             uint8_t *line_lost_ticks,
                             const char *state_name)
{
    SET_MOTORS_SPEED(0, 0);
    task3_reset_encoder_control();
    *last_black_distance = 0.0f;
    *held_turn_speed = 0.0f;
    *line_lost_ticks = 0U;
    *state = TASK3_FOLLOW_LINE;
    task3_show_state(state_name);
}

static void task3_start_arc_exit_align(Task3State *state,
                                       uint8_t *line_lost_ticks)
{
    task3_reset_encoder_control();
    *line_lost_ticks = 0U;
    *state = TASK3_ARC_EXIT_ALIGN;
    task3_show_state("ALIGN");
}

static void task3_stop(Task3State *state, const char *state_name)
{
    SET_MOTORS_SPEED(0, 0);
    *state = TASK3_ERROR;
    task3_show_state(state_name);
    Start_Alarm();
}

void task3(void)
{
    Task3State state = TASK3_WAIT;
    uint8_t endpoint_count = 0;
    bool straight_armed = false;
    uint8_t straight_settle_ticks = 0U;
    uint8_t line_lost_ticks = 0U;
    float last_black_distance = 0.0f;
    float held_turn_speed = 0.0f;
    int straight_left_speed = TASK3_STRAIGHT_LEFT_SPEED;
    int straight_right_speed = TASK3_STRAIGHT_RIGHT_SPEED;

    task4_load_straight_calibration(
        &straight_left_speed, &straight_right_speed);

    SET_MOTORS_SPEED(0, 0);
    Reset_Distance();
    task3_show_state("K3 START");

    while (1) {
        KeyState key = get_key_state();

        if (key == KEY_K2) {
            SET_MOTORS_SPEED(0, 0);
            page_flag = 1;
            break;
        }

        if (state == TASK3_WAIT ||
            state == TASK3_FINISHED ||
            state == TASK3_ERROR) {
            SET_MOTORS_SPEED(0, 0);
            if (key == KEY_K3) {
                endpoint_count = 0;
                if (Huidu_Pattern != 0) {
                    task3_start_line(
                        &state, &last_black_distance,
                        &held_turn_speed, &line_lost_ticks, "LINE");
                }
                else {
                    task3_start_straight(
                        &state, &straight_armed,
                        &straight_settle_ticks, "STRAIGHT");
                }
            }
            continue;
        }

        if (state == TASK3_STRAIGHT) {
            task3_encoder_straight(
                straight_left_speed, straight_right_speed,
                &straight_settle_ticks);

            if (Huidu_Pattern == 0) {
                straight_armed = true;
            }

            if (straight_armed &&
                Huidu_Pattern != 0) {
                endpoint_count++;
                Start_Alarm();

                if (endpoint_count >= 4) {
                    SET_MOTORS_SPEED(0, 0);
                    state = TASK3_FINISHED;
                    task3_show_state("FINISH");
                }
                else {
                    task3_start_line(
                        &state, &last_black_distance,
                        &held_turn_speed, &line_lost_ticks, "LINE");
                }
            }
            else if (Measure_Distance >= TASK3_STRAIGHT_MAX_CM) {
                task3_stop(&state, "LINE LOST");
            }
        }
        else if (state == TASK3_FOLLOW_LINE) {
            if (Huidu_Pattern != 0) {
                float current_turn =
                    task3_curve_tracing(TASK3_CURVE_SPEED);

                if (Huidu_Flag == 1U) {
                    held_turn_speed += TASK3_CURVE_HOLD_FILTER *
                        (current_turn - held_turn_speed);
                }
                last_black_distance = Measure_Distance;
                line_lost_ticks = 0U;
            }
            else {
                task3_curve_drive(
                    TASK3_CURVE_SPEED, held_turn_speed);
                if (line_lost_ticks < TASK3_LINE_LOST_CONFIRM_TICKS) {
                    line_lost_ticks++;
                }
            }

            if (line_lost_ticks >= TASK3_LINE_LOST_CONFIRM_TICKS) {
                endpoint_count++;
                Start_Alarm();

                if (endpoint_count >= 4) {
                    SET_MOTORS_SPEED(0, 0);
                    state = TASK3_FINISHED;
                    task3_show_state("FINISH");
                }
                else {
                    task3_start_arc_exit_align(
                        &state, &line_lost_ticks);
                }
            }
            else if (Measure_Distance >= TASK3_LINE_MAX_CM) {
                task3_stop(&state, "END LOST");
            }
        }
        else if (state == TASK3_ARC_EXIT_ALIGN) {
            task3_curve_drive(TASK3_CURVE_SPEED, held_turn_speed);

            if (Measure_Distance >= TASK3_ARC_EXIT_DISTANCE_CM) {
                task3_start_straight(
                    &state, &straight_armed,
                    &straight_settle_ticks, "STRAIGHT");
            }
        }

        delay_ms(10);
    }
}
