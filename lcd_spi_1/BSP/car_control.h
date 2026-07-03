#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdint.h>
#include "stdbool.h"
// 外部变量声明
extern float Target_ChaSu;                      // 目标差速变量
//循迹定时器变量
extern volatile uint32_t tracing_duration_ms;   // 动态设置的循迹时长（单位ms）
extern volatile uint32_t tracing_timer_count;   // 中断计数器
extern volatile uint8_t tracing_enabled;        // 循迹使能标志
extern volatile uint8_t tracing_speed;          // 循迹速度 1：高速 0：低速
//pid计算相关变量
extern float Target_Distance;
extern float Target_Gyro;
extern float Target_Angle ;
//定时器条件控制
extern bool Car_GoStraight_flag ;
extern bool Car_Tracing_flag;
extern uint32_t Distance_limit;
extern uint32_t GoStraight_speed;//占空比
extern uint32_t Tracing_speed;//占空比
// 函数声明
//陀螺仪直行
void Car_GoStraight(int speed);
void GoS_begin(uint32_t speed, uint32_t distance);
void GoS_stop(void);
//灰度循迹函数
void Car_Tracing(int speed);//speed可变，推荐使用
void Trc_begin(uint32_t speed, uint32_t distance);
void Trc_stop(void);
//快速停车
void Fast_stop(void);

//送药小车相关函数
// void Car_Tracing_High(void);//50
// void Car_Tracing_Mid(void);//40
// void Car_Tracing_LOW(void);//30
// void Stop_Tracing(void);//循迹强制停止
// void Start_Tracing(uint32_t duration_ms, int speed);//进入duration_ms的循迹
// float Angle_Range(float angle);
// void Car_Trun_90(uint8_t direction);
// void Car_Back_90(uint8_t direction);

// void dot_stop();//虚线停车
#endif /* CAR_CONTROL_H */




