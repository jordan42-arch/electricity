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
#include "task4.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define TASK4_CAL_SPEED              55
#define TASK4_WARMUP_DISTANCE_CM     25.0f
#define TASK4_MIN_SAMPLES            50U
#define TASK4_MAX_BASE_CORRECTION     8
#define TASK4_TRACE_SCALE             1.05f

#define TASK4_FLASH_ADDRESS       0x0001FC00U
#define TASK4_FLASH_SECTOR_SIZE   1024U
#define TASK4_CAL_MAGIC           0x43414C31U
#define TASK4_CAL_VERSION         1U
#define TASK4_CAL_CHECK_XOR       0x5A3CC3A5U

/*
 * Reserve the last 1 KB main-flash sector so the linker cannot place code in
 * the sector used for straight-line calibration.
 */
const volatile uint8_t g_task4_flash_reserve[TASK4_FLASH_SECTOR_SIZE]
    __attribute__((noinit, used, location(TASK4_FLASH_ADDRESS)));

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t left_base;
    uint32_t right_base;
    uint32_t sample_count;
    uint32_t reserved;
    uint32_t checksum;
    uint32_t magic_inverse;
} Task4CalibrationRecord;

static int task4_limit_speed(int speed)
{
    if (speed > 100) {
        return 100;
    }
    if (speed < 0) {
        return 0;
    }
    return speed;
}

static uint32_t task4_cal_checksum(const Task4CalibrationRecord *record)
{
    return record->magic ^ record->version ^ record->left_base ^
           record->right_base ^ record->sample_count ^ record->reserved ^
           TASK4_CAL_CHECK_XOR;
}

bool task4_load_straight_calibration(int *left_base, int *right_base)
{
    const Task4CalibrationRecord *record =
        (const Task4CalibrationRecord *) &g_task4_flash_reserve[0];

    if (record->magic != TASK4_CAL_MAGIC ||
        record->version != TASK4_CAL_VERSION ||
        record->magic_inverse != ~TASK4_CAL_MAGIC ||
        record->checksum != task4_cal_checksum(record) ||
        record->left_base < 35U || record->left_base > 75U ||
        record->right_base < 35U || record->right_base > 75U) {
        return false;
    }

    *left_base = (int) record->left_base;
    *right_base = (int) record->right_base;
    return true;
}

static bool task4_save_straight_calibration(
    int left_base, int right_base, uint32_t sample_count)
{
    Task4CalibrationRecord record = {
        .magic = TASK4_CAL_MAGIC,
        .version = TASK4_CAL_VERSION,
        .left_base = (uint32_t) left_base,
        .right_base = (uint32_t) right_base,
        .sample_count = sample_count,
        .reserved = 0U,
        .checksum = 0U,
        .magic_inverse = ~TASK4_CAL_MAGIC
    };
    record.checksum = task4_cal_checksum(&record);

    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();

    DL_FlashCTL_unprotectSector(
        FLASHCTL, TASK4_FLASH_ADDRESS, DL_FLASHCTL_REGION_SELECT_MAIN);
    DL_FLASHCTL_COMMAND_STATUS status = DL_FlashCTL_eraseMemoryFromRAM(
        FLASHCTL, TASK4_FLASH_ADDRESS, DL_FLASHCTL_COMMAND_SIZE_SECTOR);

    if (status == DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        DL_FlashCTL_unprotectSector(
            FLASHCTL, TASK4_FLASH_ADDRESS, DL_FLASHCTL_REGION_SELECT_MAIN);
        status =
            DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
                FLASHCTL,
                TASK4_FLASH_ADDRESS,
                (uint32_t *) &record,
                sizeof(record) / sizeof(uint32_t),
                DL_FLASHCTL_REGION_SELECT_MAIN);
    }

    if (interrupt_state == 0U) {
        __enable_irq();
    }

    return status == DL_FLASHCTL_COMMAND_STATUS_PASSED &&
           task4_load_straight_calibration(&left_base, &right_base);
}

static float task4_trace_once(void)
{
    float turn_speed =
        PID_Calculate(&pid_Turn, Huidu_Error, Huidu_Target) *
        TASK4_TRACE_SCALE;

    SET_MOTORS_SPEED(
        task4_limit_speed(TASK4_CAL_SPEED + (int) turn_speed),
        task4_limit_speed(TASK4_CAL_SPEED - (int) turn_speed));
    return turn_speed;
}

static void task4_show(const char *line1, const char *line2)
{
    LCD_ShowString(0, 20, "               ", BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 40, "               ", BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 20, line1, BLACK, WHITE, 16, 0);
    LCD_ShowString(0, 40, line2, BLACK, WHITE, 16, 0);
}

void task4(void)
{
    bool running = false;
    float correction_sum = 0.0f;
    uint32_t sample_count = 0U;
    uint8_t lost_count = 0U;
    char display_buffer[24];

    SET_MOTORS_SPEED(0, 0);
    task4_show("TASK4 CAL", "K3 START");

    while (1) {
        KeyState key = get_key_state();

        if (key == KEY_K2) {
            SET_MOTORS_SPEED(0, 0);
            page_flag = 1;
            break;
        }

        if (key == KEY_K3) {
            if (!running) {
                correction_sum = 0.0f;
                sample_count = 0U;
                lost_count = 0U;
                Reset_Distance();
                PID_Reset(&pid_Turn);
                running = true;
                task4_show("TASK4 CAL", "RUN K3 SAVE");
            }
            else {
                SET_MOTORS_SPEED(0, 0);
                running = false;

                if (sample_count < TASK4_MIN_SAMPLES) {
                    task4_show("CAL FAILED", "RUN LONGER");
                    continue;
                }

                float average = correction_sum / (float) sample_count;
                int correction = average >= 0.0f
                    ? (int) (average + 0.5f)
                    : (int) (average - 0.5f);

                if (correction > TASK4_MAX_BASE_CORRECTION) {
                    correction = TASK4_MAX_BASE_CORRECTION;
                }
                else if (correction < -TASK4_MAX_BASE_CORRECTION) {
                    correction = -TASK4_MAX_BASE_CORRECTION;
                }

                int left_base = TASK4_CAL_SPEED + correction;
                int right_base = TASK4_CAL_SPEED - correction;

                if (task4_save_straight_calibration(
                        left_base, right_base, sample_count)) {
                    snprintf(display_buffer, sizeof(display_buffer),
                             "SAVED %d/%d", left_base, right_base);
                    task4_show("CAL OK", display_buffer);
                    Start_Alarm();
                }
                else {
                    task4_show("CAL FAILED", "FLASH ERROR");
                }
            }
        }

        if (running) {
            if (Huidu_Pattern == 0U) {
                if (lost_count < 255U) {
                    lost_count++;
                }
                if (lost_count >= 5U) {
                    SET_MOTORS_SPEED(0, 0);
                    running = false;
                    task4_show("CAL FAILED", "LINE LOST");
                }
            }
            else {
                lost_count = 0U;
                float correction = task4_trace_once();

                if (Measure_Distance >= TASK4_WARMUP_DISTANCE_CM &&
                    Huidu_Flag == 1U) {
                    correction_sum += correction;
                    sample_count++;
                }
            }
        }
        else {
            SET_MOTORS_SPEED(0, 0);
        }

        delay_ms(10);
    }
}
