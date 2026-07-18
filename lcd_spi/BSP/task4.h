/**
 * @file task4.h
 * @brief 任务4模块头文件
 * 
 * 该文件包含任务4的函数声明，主要实现智能车路径识别与导航功能
 */

#ifndef TASK4_H
#define TASK4_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 任务4主函数
 * 
 * 实现智能车的完整路径识别与导航功能，包括：
 * - 数字识别与路径记忆
 * - 十字路口判断与转向控制
 * - 返程路径规划与执行
 * - 实时状态显示
 * 
 * 按下K2键可退出任务
 */
void task4(void);

/**
 * @brief Load the straight-line motor bases learned by task4.
 *
 * @return true when a valid calibration record exists in internal flash.
 */
bool task4_load_straight_calibration(int *left_base, int *right_base);

#ifdef __cplusplus
}
#endif

#endif /* TASK4_H */
