#ifndef __MOTOR_H__
#define __MOTOR_H__

/*
说明：
1.该模块是左、右轮电机驱动模块。使用Motor(右轮速度，左轮速度)函数来驱动小车电机。
2.速度的输入范围是-100~100，本质上是修改驱动电机的信号的占空比，正、负分别代表小车前进和后退。

使用方式：
1.在主模块中启动电机PWM输出。
2.使用Motor(0,0)初始化速度为0。
3.允许随时使用Motor(右轮速度，左轮速度)来驱动小车和修改小车速度。
*/
#include "board.h"
#include "lcd.h"
#include "lcd_init.h"
/*-------------------------------------------------------------------------------------------*/
/*---------------------------------------PID函数---------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
/*PID结构体部分*/
/*PID三个权重*/
/*目标值、获取值、三个误差*/
/*比例、积分、微分输出、总输出*/
/*输出限幅以及积分限幅*/
typedef struct pid_t
{
    float Kp; // P参数
    float Ki; // I参数
    float Kd; // D参数

    float Target;   // 目标值
    float Measure;  // 当前获取值
    float Error[3]; // 三次误差

    float KpOut;   // 比例输出
    float KiOut;   // 积分输出
    float KdOut;   // 微分输出
    float PID_Out; // PID总输出

    uint32_t PID_Limit_MAX; // 最大输出限幅
    uint32_t Ki_Limit_MAX;  // 最大积分输出限幅
} pid_t;

float PID_Calculate(pid_t *pid, float Measure, float Target);
float PID_Calculate_Angle(pid_t *pid, float Measure, float Target);
void Set_PID_Param(pid_t *pid, float P, float I, float D);
void PID_Reset(pid_t *pid);
/*************************************************************************************************
 *	pid调试参数、调试函数//v6
 *************************************************************************************************/
extern pid_t* current_pid;//指向当前要修改的 PID 环结构体
extern uint8_t current_param;
extern uint8_t pid_page;//0：参数选择页，1：参数调整页
void AdjustPIDParam(uint8_t and_or_sub);//用于根据 current_param 修改对应的 PID 参数
void SelectNextPID(void);//切换当前 PID 指针的函数
void SelectNextParam(void);//函数切换要修改的参数项（Kp/Ki/Kd）
void DisplayPIDInfo(void);//用于在 LCD 屏幕上显示当前选中的 PID 环名称及其 Kp、Ki、Kd 参数
/*************************************************************************************************/

/*-------------------------------------------------------------------------------------------*/
/*------------------------------------电机操作函数-------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
// 初始化函数
void motor_init(void);

// 电机驱动函数，可以同时控制，可以单独控制左右轮子。
void SET_MOTORS_SPEED(int motor_left_speed, int motor_right_speed); // 分别修改右、左轮电机的PWM占空比。用正、负代表轮子前进和轮子后退。
void Set_Motor1_Speed(int Target_Speed);                 // 左
void Set_Motor2_Speed(int Target_Speed);                 // 右

// 定义（简化写法），注意：宏定义后面不加一般代码语句所有的分号;！！！
#define restrict_factor 0.8                                                                                                 // 限制占空比防止过驱动烧坏板子。
#define PWM_A1(speed) DL_TimerA_setCaptureCompareValue(PWM_Motor_INST, speed * 80 * restrict_factor, GPIO_PWM_Motor_C0_IDX) // 右M+
#define PWM_A2(speed) DL_TimerA_setCaptureCompareValue(PWM_Motor_INST, speed * 80 * restrict_factor, GPIO_PWM_Motor_C1_IDX) // 右M-
#define PWM_B1(speed) DL_TimerA_setCaptureCompareValue(PWM_Motor_INST, speed * 80 * restrict_factor, GPIO_PWM_Motor_C2_IDX) // 左M+
#define PWM_B2(speed) DL_TimerA_setCaptureCompareValue(PWM_Motor_INST, speed * 80 * restrict_factor, GPIO_PWM_Motor_C3_IDX) // 左M-

// PID参数
extern pid_t pid_Motor1_Speed;
extern pid_t pid_Motor2_Speed;
extern pid_t pid_Turn;
extern pid_t pid_Distance;
extern pid_t pid_Pirouetee_Gyro;
extern pid_t pid_Pirouetee_Angle;
extern pid_t pid_Gyro;
extern pid_t pid_Angle;
extern pid_t pid_Angle_back;


// PID标志位（可用可不用，待定）
extern uint8_t Turn_PID_Flag;
extern uint8_t Distance_PID_Flag;
extern uint8_t Gyro_PID_Flag;
extern uint8_t Angle_PID_Flag;

#endif