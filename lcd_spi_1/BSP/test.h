/**
 * @file test.h
 * @brief 测试功能模块头文件
 * 
 * 该文件包含各种测试功能的函数声明，包括传感器测试、PID调试等功能
 */

#ifndef TEST_H
#define TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 基础传感器测试函数
 * 
 * 该函数用于测试灰度传感器、IMU、红灯检测等基础传感器功能
 * 显示传感器数据并在按键K2按下时退出测试模式
 */
void test_1(void);

/**
 * @brief PID参数调试函数
 * 
 * 该函数用于调试PID参数，支持：
 * - K1键切换PID环
 * - K3键切换具体参数（KP/KI/KD）
 * - K1/K2键调整参数值
 * - K2键确认修改或结束调参
 */
void test_2(void);

/**
 * @brief 显示灰度传感器信息
 * 
 * 在LCD上显示灰度传感器的误差值和标志位状态
 */
void display_grey_info(int y_position);

/**
 * @brief 显示IMU信息
 * 
 * @param y_position LCD上显示的Y坐标位置
 * 在LCD上显示IMU的yaw角度值
 */
void display_imu_info(int y_position);

/**
 * @brief 显示红灯状态
 * 
 * 在LCD上显示红灯检测状态（drug_get/drug_remove）
 */
void display_red_state(void);


/**
 * @brief 处理PID参数调整
 * 
 * 处理PID参数的增加、减少和确认操作
 */
void handle_pid_adjustment(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_H */