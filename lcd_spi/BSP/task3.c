#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "board.h"
#include "car_control.h"
#include "encoder.h"
#include "grey.h"
#include "imu.h"
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
#define TASK3_LINE_LOST_CONFIRM_TICKS 3U
#define TASK3_LINE_LOST_CONFIRM_CM    1.0f
#define TASK3_LINE_ENTRY_CONFIRM_TICKS 3U
#define TASK3_LINE_ENTRY_CONFIRM_CM   1.5f
#define TASK3_LINE_EXIT_LOCK_CM       2.0f

/*
 * The heading controller is only active on the white straight sections.
 * Positive correction slows the left wheel and speeds up the right wheel.
 * If the IMU is mounted with its Z axis reversed, change the sign below.
 */
#define TASK3_HEADING_TURN_SIGN       1.0f
#define TASK3_HEADING_KP               3.0f
#define TASK3_HEADING_RATE_KP          0.55f
#define TASK3_HEADING_MAX_RATE_DPS    70.0f
#define TASK3_HEADING_MAX_CORRECTION  20.0f

/* Keep the same arc curvature while the wheel axle catches up to the endpoint. */
#define TASK3_ARC_EXIT_RATE_KP         0.10f
#define TASK3_ARC_EXIT_MAX_CORRECTION 12.0f

#define TASK3_FINAL_ALIGN_TOLERANCE_DEG       3.0f
#define TASK3_FINAL_ALIGN_RATE_TOLERANCE_DPS  8.0f
#define TASK3_FINAL_ALIGN_STABLE_TICKS        5U
#define TASK3_FINAL_ALIGN_MIN_TURN_SPEED     18
#define TASK3_FINAL_ALIGN_MAX_TURN_SPEED     35
#define TASK3_FINAL_ALIGN_TIMEOUT_TICKS     300U
#define TASK3_RETURN_LINE_MIN_DISTANCE_CM    2.0f
#define TASK3_RETURN_LINE_HEADING_TOLERANCE_DEG 6.0f

typedef enum {
    TASK3_WAIT = 0,
    TASK3_STRAIGHT,
    TASK3_FOLLOW_LINE,
    TASK3_ARC_EXIT_ALIGN,
    TASK3_RETURN_START_LINE,
    TASK3_FINAL_ALIGN,
    TASK3_FINISHED,
    TASK3_ERROR
} Task3State;

typedef struct {
    uint8_t ticks;
    bool center_seen;
    float start_distance;
} Task3LineEntryCandidate;

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

static int task3_round_to_int(float value)
{
    return value >= 0.0f ? (int) (value + 0.5f) :
                           (int) (value - 0.5f);
}

static float task3_limit_float(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float task3_abs_float(float value)
{
    return value < 0.0f ? -value : value;
}

static float task3_wrap_angle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle <= -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float task3_current_heading(float heading_zero)
{
    return task3_wrap_angle(imu_data.yaw - heading_zero);
}

static void task3_reset_encoder_control(void)
{
    gEncoderCount_L = 0;
    gEncoderCount_R = 0;
    Motor1_Speed = 0.0f;
    Motor2_Speed = 0.0f;
    Reset_Distance();
}

static float task3_heading_correction(float target_heading,
                                      float heading_zero)
{
    float heading_error = task3_wrap_angle(
        target_heading - task3_current_heading(heading_zero));
    float target_yaw_rate = task3_limit_float(
        TASK3_HEADING_KP * heading_error,
        TASK3_HEADING_MAX_RATE_DPS);
    float rate_error = target_yaw_rate - imu_data.angular_velocity_z;

    return task3_limit_float(
        TASK3_HEADING_TURN_SIGN * TASK3_HEADING_RATE_KP * rate_error,
        TASK3_HEADING_MAX_CORRECTION);
}

static void task3_heading_straight(int left_base,
                                   int right_base,
                                   float target_heading,
                                   float heading_zero)
{
    int correction = task3_round_to_int(
        task3_heading_correction(target_heading, heading_zero));

    SET_MOTORS_SPEED(
        task3_limit_speed(left_base - correction),
        task3_limit_speed(right_base + correction));
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

static void task3_arc_exit_drive(int base_speed,
                                 float held_turn_speed,
                                 float held_yaw_rate)
{
    float rate_error = held_yaw_rate - imu_data.angular_velocity_z;
    int rate_correction = task3_round_to_int(task3_limit_float(
        TASK3_ARC_EXIT_RATE_KP * rate_error,
        TASK3_ARC_EXIT_MAX_CORRECTION));

    /* A positive yaw-rate correction needs left wheel slower, right faster. */
    SET_MOTORS_SPEED(
        task3_limit_speed(base_speed + task3_round_to_int(held_turn_speed) -
                          rate_correction),
        task3_limit_speed(base_speed - task3_round_to_int(held_turn_speed) +
                          rate_correction));
}

static bool task3_heading_is_settled(float target_heading,
                                     float heading_zero)
{
    float heading_error = task3_wrap_angle(
        target_heading - task3_current_heading(heading_zero));

    return task3_abs_float(heading_error) <=
               TASK3_FINAL_ALIGN_TOLERANCE_DEG &&
           task3_abs_float(imu_data.angular_velocity_z) <=
               TASK3_FINAL_ALIGN_RATE_TOLERANCE_DPS;
}

static bool task3_heading_is_near(float target_heading,
                                  float heading_zero,
                                  float tolerance)
{
    float heading_error = task3_wrap_angle(
        target_heading - task3_current_heading(heading_zero));

    return task3_abs_float(heading_error) <= tolerance;
}

static void task3_final_align_drive(float target_heading, float heading_zero)
{
    float heading_error = task3_wrap_angle(
        target_heading - task3_current_heading(heading_zero));
    float correction = task3_heading_correction(
        target_heading, heading_zero);
    int turn_speed;

    if (task3_abs_float(correction) < 0.5f) {
        correction = heading_error >= 0.0f ? 0.5f : -0.5f;
    }

    turn_speed = task3_round_to_int(task3_abs_float(correction));
    if (turn_speed < TASK3_FINAL_ALIGN_MIN_TURN_SPEED) {
        turn_speed = TASK3_FINAL_ALIGN_MIN_TURN_SPEED;
    }
    if (turn_speed > TASK3_FINAL_ALIGN_MAX_TURN_SPEED) {
        turn_speed = TASK3_FINAL_ALIGN_MAX_TURN_SPEED;
    }

    if (correction >= 0.0f) {
        SET_MOTORS_SPEED(-turn_speed, turn_speed);
    }
    else {
        SET_MOTORS_SPEED(turn_speed, -turn_speed);
    }
}

static void task3_show_state(const char *state_name)
{
    LCD_ShowString(0, 20, "          ", BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 40, "          ", BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 20, "TASK3 LAP", BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 40, state_name, BLACK, WHITE, 16, 0);
}

static void task3_reset_line_entry_candidate(
    Task3LineEntryCandidate *candidate)
{
    candidate->ticks = 0U;
    candidate->center_seen = false;
    candidate->start_distance = 0.0f;
}

static void task3_start_straight(Task3State *state,
                                 bool *straight_armed,
                                 Task3LineEntryCandidate *line_candidate,
                                 const char *state_name)
{
    SET_MOTORS_SPEED(0, 0);
    task3_reset_encoder_control();
    *straight_armed = false;
    task3_reset_line_entry_candidate(line_candidate);
    *state = TASK3_STRAIGHT;
    task3_show_state(state_name);
}

static void task3_start_line(Task3State *state,
                             float *held_turn_speed,
                             float *held_yaw_rate,
                             uint8_t *line_lost_ticks,
                             float *line_lost_start_distance,
                             float *line_exit_unlock_distance,
                             float exit_lock_distance,
                             const char *state_name)
{
    SET_MOTORS_SPEED(0, 0);
    task3_reset_encoder_control();
    *held_turn_speed = 0.0f;
    *held_yaw_rate = 0.0f;
    *line_lost_ticks = 0U;
    *line_lost_start_distance = 0.0f;
    *line_exit_unlock_distance = exit_lock_distance;
    PID_Reset(&pid_Turn);
    *state = TASK3_FOLLOW_LINE;
    task3_show_state(state_name);
}

static void task3_start_arc_exit_align(Task3State *state,
                                       uint8_t *line_lost_ticks,
                                       float *line_lost_start_distance)
{
    task3_reset_encoder_control();
    *line_lost_ticks = 0U;
    *line_lost_start_distance = 0.0f;
    *state = TASK3_ARC_EXIT_ALIGN;
    task3_show_state("ALIGN");
}

static void task3_start_final_align(Task3State *state,
                                    uint8_t *stable_ticks,
                                    uint16_t *timeout_ticks)
{
    SET_MOTORS_SPEED(0, 0);
    *stable_ticks = 0U;
    *timeout_ticks = 0U;
    *state = TASK3_FINAL_ALIGN;
    task3_show_state("FINAL");
}

static void task3_start_return_line(Task3State *state,
                                    float *held_turn_speed,
                                    float *held_yaw_rate,
                                    uint8_t *line_lost_ticks,
                                    float *line_lost_start_distance,
                                    float *line_exit_unlock_distance)
{
    task3_start_line(
        state, held_turn_speed, held_yaw_rate, line_lost_ticks,
        line_lost_start_distance, line_exit_unlock_distance, 0.0f,
        "RETURN");
    *state = TASK3_RETURN_START_LINE;
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
    uint8_t line_lost_ticks = 0U;
    float line_lost_start_distance = 0.0f;
    float line_exit_unlock_distance = 0.0f;
    Task3LineEntryCandidate line_candidate = {0U, false, 0.0f};
    float held_turn_speed = 0.0f;
    float held_yaw_rate = 0.0f;
    float heading_zero = 0.0f;
    float straight_target_heading = 0.0f;
    float final_target_heading = 0.0f;
    uint8_t final_align_stable_ticks = 0U;
    uint16_t final_align_timeout_ticks = 0U;
    bool final_after_arc_align = false;
    bool started_on_line = false;
    bool straight_heading_known = false;
    bool capture_heading_after_arc_align = false;
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
                heading_zero = imu_data.yaw;
                straight_target_heading = 0.0f;
                final_after_arc_align = false;
                capture_heading_after_arc_align = false;
                started_on_line = Huidu_Pattern != 0U;
                straight_heading_known = !started_on_line;

                if (started_on_line) {
                    task3_start_line(
                        &state, &held_turn_speed, &held_yaw_rate,
                        &line_lost_ticks, &line_lost_start_distance,
                        &line_exit_unlock_distance, 0.0f, "LINE");
                }
                else {
                    task3_start_straight(
                        &state, &straight_armed, &line_candidate,
                        "STRAIGHT");
                }
            }
            continue;
        }

        if (state == TASK3_STRAIGHT) {
            bool line_confirmed = false;

            task3_heading_straight(
                straight_left_speed, straight_right_speed,
                straight_target_heading, heading_zero);

            if (Huidu_Pattern == 0) {
                straight_armed = true;
                task3_reset_line_entry_candidate(&line_candidate);
            }
            else if (straight_armed) {
                if (line_candidate.ticks == 0U) {
                    line_candidate.start_distance = Measure_Distance;
                }
                if (line_candidate.ticks < 255U) {
                    line_candidate.ticks++;
                }
                if (Huidu_Flag == 1U) {
                    line_candidate.center_seen = true;
                }

                line_confirmed =
                    line_candidate.ticks >= TASK3_LINE_ENTRY_CONFIRM_TICKS &&
                    line_candidate.center_seen &&
                    (Measure_Distance - line_candidate.start_distance) >=
                        TASK3_LINE_ENTRY_CONFIRM_CM;
            }

            if (line_confirmed) {
                endpoint_count++;
                Start_Alarm();

                if (endpoint_count >= 4) {
                    if (started_on_line) {
                        task3_start_return_line(
                            &state, &held_turn_speed, &held_yaw_rate,
                            &line_lost_ticks, &line_lost_start_distance,
                            &line_exit_unlock_distance);
                    }
                    else {
                        final_target_heading = 0.0f;
                        final_after_arc_align = false;
                        task3_start_final_align(
                            &state, &final_align_stable_ticks,
                            &final_align_timeout_ticks);
                    }
                }
                else {
                    task3_start_line(
                        &state, &held_turn_speed, &held_yaw_rate,
                        &line_lost_ticks, &line_lost_start_distance,
                        &line_exit_unlock_distance,
                        TASK3_LINE_EXIT_LOCK_CM, "LINE");
                }
            }
            else if (Measure_Distance >= TASK3_STRAIGHT_MAX_CM) {
                task3_stop(&state, "LINE LOST");
            }
        }
        else if (state == TASK3_FOLLOW_LINE) {
            bool line_exit_locked =
                Measure_Distance < line_exit_unlock_distance;
            bool line_lost_confirmed = false;

            if (Huidu_Pattern != 0) {
                float current_turn =
                    task3_curve_tracing(TASK3_CURVE_SPEED);

                if (Huidu_Flag == 1U) {
                    held_turn_speed += TASK3_CURVE_HOLD_FILTER *
                        (current_turn - held_turn_speed);
                    held_yaw_rate += TASK3_CURVE_HOLD_FILTER *
                        (imu_data.angular_velocity_z - held_yaw_rate);
                }
                line_lost_ticks = 0U;
                line_lost_start_distance = Measure_Distance;
            }
            else {
                task3_curve_drive(
                    TASK3_CURVE_SPEED, held_turn_speed);

                if (!line_exit_locked) {
                    if (line_lost_ticks == 0U) {
                        line_lost_start_distance = Measure_Distance;
                    }
                    if (line_lost_ticks < 255U) {
                        line_lost_ticks++;
                    }
                    line_lost_confirmed =
                        line_lost_ticks >= TASK3_LINE_LOST_CONFIRM_TICKS &&
                        (Measure_Distance - line_lost_start_distance) >=
                            TASK3_LINE_LOST_CONFIRM_CM;
                }
            }

            if (line_lost_confirmed) {
                endpoint_count++;
                Start_Alarm();

                if (straight_heading_known) {
                    straight_target_heading = task3_wrap_angle(
                        straight_target_heading + 180.0f);
                }
                else {
                    capture_heading_after_arc_align = true;
                }
                final_after_arc_align = endpoint_count >= 4;
                task3_start_arc_exit_align(
                    &state, &line_lost_ticks,
                    &line_lost_start_distance);
            }
            else if (Measure_Distance >= TASK3_LINE_MAX_CM) {
                task3_stop(&state, "END LOST");
            }
        }
        else if (state == TASK3_ARC_EXIT_ALIGN) {
            task3_arc_exit_drive(
                TASK3_CURVE_SPEED, held_turn_speed, held_yaw_rate);

            if (Measure_Distance >= TASK3_ARC_EXIT_DISTANCE_CM) {
                if (capture_heading_after_arc_align) {
                    straight_target_heading = task3_current_heading(
                        heading_zero);
                    straight_heading_known = true;
                    capture_heading_after_arc_align = false;
                }

                if (final_after_arc_align) {
                    final_target_heading = 0.0f;
                    final_after_arc_align = false;
                    task3_start_final_align(
                        &state, &final_align_stable_ticks,
                        &final_align_timeout_ticks);
                }
                else {
                    task3_start_straight(
                        &state, &straight_armed, &line_candidate,
                        "STRAIGHT");
                }
            }
        }
        else if (state == TASK3_RETURN_START_LINE) {
            if (Huidu_Pattern == 0U) {
                task3_stop(&state, "RETURN LOST");
            }
            else {
                task3_curve_tracing(TASK3_CURVE_SPEED);

                if (Measure_Distance >= TASK3_RETURN_LINE_MIN_DISTANCE_CM &&
                    task3_heading_is_near(
                        0.0f, heading_zero,
                        TASK3_RETURN_LINE_HEADING_TOLERANCE_DEG)) {
                    final_target_heading = 0.0f;
                    task3_start_final_align(
                        &state, &final_align_stable_ticks,
                        &final_align_timeout_ticks);
                }
                else if (Measure_Distance >= TASK3_LINE_MAX_CM) {
                    task3_stop(&state, "RETURN ERR");
                }
            }
        }
        else if (state == TASK3_FINAL_ALIGN) {
            if (task3_heading_is_settled(
                    final_target_heading, heading_zero)) {
                if (final_align_stable_ticks < 255U) {
                    final_align_stable_ticks++;
                }
            }
            else {
                final_align_stable_ticks = 0U;
            }

            if (final_align_stable_ticks >=
                TASK3_FINAL_ALIGN_STABLE_TICKS) {
                SET_MOTORS_SPEED(0, 0);
                state = TASK3_FINISHED;
                task3_show_state("FINISH");
                Start_Alarm();
            }
            else if (final_align_timeout_ticks >=
                     TASK3_FINAL_ALIGN_TIMEOUT_TICKS) {
                task3_stop(&state, "ALIGN ERR");
            }
            else {
                task3_final_align_drive(
                    final_target_heading, heading_zero);
                final_align_timeout_ticks++;
            }
        }

        delay_ms(10);
    }
}
