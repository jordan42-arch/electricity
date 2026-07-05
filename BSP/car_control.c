#include "car_control.h"
#include "imu.h"
#include "motor.h"
#include "grey.h"
#include "board.h"
#include "encoder.h"
#include <stdint.h>




//定时器变量
volatile uint32_t tracing_duration_ms = 0;  // 动态设置的循迹时长（单位ms）
volatile uint32_t tracing_timer_count = 0;  // 中断计数器
volatile uint8_t tracing_enabled = 0;       // 循迹使能标志
volatile uint8_t tracing_speed = 0;       // 循迹速度 1：高速 0：低速

//pid变量
float Target_Distance = 0;
float Target_Gyro = 0;
float Target_Angle = 0;

bool Car_GoStraight_flag = false;
bool Car_Tracing_flag = false;
uint32_t Distance_limit =10000;//距离限制，初始值为100米
uint32_t GoStraight_speed = 0;
uint32_t Tracing_speed = 0;
void Car_GoStraight(int speed)
{
    Target_Gyro = PID_Calculate_Angle(&pid_Angle, imu_data.yaw, Target_Angle); // 角度环
    float TurnSpeed = PID_Calculate(&pid_Gyro, Z_Gyro, Target_Gyro);           // 角速度环
    SET_MOTORS_SPEED(speed - TurnSpeed, speed + TurnSpeed);
    // delay_ms(15);
}
void GoS_begin(uint32_t speed, uint32_t distance)
{
    GoStraight_speed = speed;//PWM占空比speed
    Distance_limit = distance; //最多跑distancecm
    Reset_Distance();//距离计数清零
    Car_GoStraight_flag = 1;//启动
}
void GoS_stop(void)
{
    Car_GoStraight_flag = 0;//关闭直行
    GoStraight_speed = 0;//速度归零
    Reset_Distance();
    SET_MOTORS_SPEED(0, 0);
}
void Car_Tracing(int speed)
{
    float Target_ChaSu;  // 目标差速
    Target_ChaSu = PID_Calculate(&pid_Turn, Huidu_Error, Huidu_Target) * (35) * 0.03; // 转向环输出差速
    SET_MOTORS_SPEED(speed + Target_ChaSu, speed - Target_ChaSu);
}
void Trc_begin(uint32_t speed, uint32_t distance)
{
    Tracing_speed = speed;//PWM占空比speed
    Distance_limit = distance; //最多跑distancecm
    Reset_Distance();//距离计数清零
    Car_Tracing_flag = 1;//启动
}
void Trc_stop(void)
{
    Car_Tracing_flag = 0;//关闭循迹
    Tracing_speed = 0;//速度归零
    Reset_Distance();
    SET_MOTORS_SPEED(0, 0);
}
//用于给小车使用快刹车
void Fast_stop(void)
{
    PWM_A1(125);
    PWM_A2(125);
    PWM_B1(125);
    PWM_B2(125);
}

// // 强制停止循迹
// void Stop_Tracing(void) {
//     tracing_enabled = 0;
//     SET_MOTORS_SPEED(0, 0);
// }

// // 启动循迹，时长由参数动态指定（单位ms）
// void Start_Tracing(uint32_t duration_ms,int speed) {
//     tracing_duration_ms = duration_ms;
//     tracing_timer_count = 0;
//     tracing_speed = speed;
//     tracing_enabled = 1;
//     uint32_t timeout = 0;
//     while (tracing_enabled && timeout++ < duration_ms) { // 最多等待2000ms
//         delay_ms(1);
//         if (tracing_speed == 1) {
//                         Car_Tracing_High();
//                     }
//         else {
//                         Car_Tracing_LOW();
//                     }
//     }
//     if (!tracing_enabled) {
//         // 成功完成
//     } else {
//         Stop_Tracing(); // 超时强制停止
//     }                     
// }
// float Angle_Range(float angle) {
//      while (angle > 180.0f) angle -= 360.0f;
//     while (angle < -180.0f) angle += 360.0f;
//     return angle;
// }
// void Car_Trun_90(uint8_t direction)
// {
//     if(direction == 0)//left
//     Target_Angle = Angle_Range(Target_Angle + 90);
//     else if(direction == 1)//right
//     Target_Angle = Angle_Range(Target_Angle - 90);
//     float Tolerant_Angle = 20;//宽容范围，允许误差存在
//     while(1)
//     {
//     Target_Gyro = PID_Calculate_Angle(&pid_Angle,imu_data.yaw,Target_Angle);//角度环
//     float TurnSpeed = PID_Calculate(&pid_Gyro,Z_Gyro,Target_Gyro);//角速度环    
//     float Angle_Diff = Angle_Range(Target_Angle-imu_data.yaw);
//     if(Angle_Diff > Tolerant_Angle && direction == 0)//to left
//         SET_MOTORS_SPEED(30, 40+TurnSpeed);//40
//     else if(Angle_Diff < (-Tolerant_Angle) && direction == 1)//to right
//             SET_MOTORS_SPEED(40-TurnSpeed, 30);//40
//         else//stop
//         {
//             SET_MOTORS_SPEED(0, 0);
//             PID_Reset(&pid_Angle);
//             PID_Reset(&pid_Gyro);
//             delay_ms(100);
//             break;
//         }
//     }
// }

// void Car_Trun_90_forBack(uint8_t direction)
// {
//     if(direction == 0)//left
//     Target_Angle = Angle_Range(Target_Angle + 90);
//     else if(direction == 1)//right
//     Target_Angle = Angle_Range(Target_Angle - 90);
//     float Tolerant_Angle = 20;//宽容范围，允许误差存在
//     while(1)
//     {
//     Target_Gyro = PID_Calculate_Angle(&pid_Angle,imu_data.yaw,Target_Angle);//角度环
//     float TurnSpeed = PID_Calculate(&pid_Gyro,Z_Gyro,Target_Gyro);//角速度环    
//     float Angle_Diff = Angle_Range(Target_Angle-imu_data.yaw);
//     if(Angle_Diff > Tolerant_Angle && direction == 0)//to left
//         SET_MOTORS_SPEED(20, 40+TurnSpeed);//40
//     else if(Angle_Diff < (-Tolerant_Angle) && direction == 1)//to right
//             SET_MOTORS_SPEED(40-TurnSpeed, 20);//40
//         else//stop
//         {
//             SET_MOTORS_SPEED(0, 0);
//             PID_Reset(&pid_Angle);
//             PID_Reset(&pid_Gyro);
//             delay_ms(100);
//             break;
//         }
//     }
// }

// void Car_Back_90(uint8_t direction)
// {   
//     //退出虚线识别区
//     SET_MOTORS_SPEED(-40, -40);
//     delay_ms(1000);
//     //倒车
//     while(1){
//         Target_Gyro = PID_Calculate_Angle(&pid_Angle,imu_data.yaw,Target_Angle);//角度环
//         float TurnSpeed = PID_Calculate(&pid_Gyro,Z_Gyro,Target_Gyro);//角速度环		                                                                                                                                                                                                                                                                                                                                                                                            
//         SET_MOTORS_SPEED(-(36+TurnSpeed),-( 36-TurnSpeed));
//         delay_ms(50);
//         //if(Huidu_Flag == 2 || Huidu_Flag == 3 || Huidu_Flag == 4)
//         if(color_width > 40) //改为色块检测
//         break;
//     }
//     //前进并转向
//     while (1) {
//         Car_Tracing_High();
//         if (Huidu_Flag == 2 || Huidu_Flag == 3 || Huidu_Flag == 4) {
//          Car_Trun_90_forBack(direction);
//          break;
//         }
//     }

// }


// //虚线停车
// void dot_stop()
// {
//     int skip_frame_dot = 0;
//     Start_Tracing(500,1);
//     while(1){
//             Car_Tracing_LOW();
//             if (color_width == 0 ) {
//                 Fast_stop();
//                 delay_ms(500);
//                 break;
//                 }
//     }        
// }
// void Car_Tracing_High(void)
// {
//     float Target_ChaSu;  // 目标差速
//     Target_ChaSu = PID_Calculate(&pid_Turn, Huidu_Error, Huidu_Target) * (35) * 0.03; // 转向环输出差速
//     SET_MOTORS_SPEED(50 + Target_ChaSu, 50 - Target_ChaSu);
// }

// void Car_Tracing_Mid(void)
// {
//     float Target_ChaSu;  // 目标差速
//     Target_ChaSu = PID_Calculate(&pid_Turn, Huidu_Error, Huidu_Target) * (35) * 0.03; // 转向环输出差速
//     SET_MOTORS_SPEED(40 + Target_ChaSu, 40 - Target_ChaSu);
// }

// void Car_Tracing_LOW(void)
// {
//     float Target_ChaSu;  // 目标差速
//     Target_ChaSu = PID_Calculate(&pid_Turn, Huidu_Error, Huidu_Target) * (35) * 0.03; // 转向环输出差速
//     SET_MOTORS_SPEED(30 + Target_ChaSu, 30 - Target_ChaSu);
// }
//未使用函数
//0
// void Car_Tracing_back()
// {   
//     Target_Gyro = PID_Calculate_Angle(&pid_Angle_back,imu_data.yaw,Target_Angle);//角度环
//     float TurnSpeed = PID_Calculate(&pid_Gyro,Z_Gyro,Target_Gyro);//角速度环		                                                                                                                                                                                                                                                                                                                                                                                            
//     SET_MOTORS_SPEED(-40-TurnSpeed, -40+TurnSpeed);
// }
// //0
// void Car_Pirouetee_QPY(float Angle)
// {   
    
//     Target_Angle = Target_Angle + Angle;
//     float Tolerant_Angle = 5;//宽容范围，允许误差存在 
//     while(1){
//         Target_Gyro = PID_Calculate_Angle(&pid_Angle,imu_data.yaw,Target_Angle);//角度环
//         float TurnSpeed = PID_Calculate(&pid_Gyro,Z_Gyro,Target_Gyro);//角速度环		                                                                                                                                                                                                                                                                                                                                                                                            
//         if((Target_Angle-imu_data.yaw) > Tolerant_Angle)//to left
//         SET_MOTORS_SPEED(0-TurnSpeed, 0+TurnSpeed);
//         else if((Target_Angle-imu_data.yaw) < (-Tolerant_Angle))//to right
//         SET_MOTORS_SPEED(0-TurnSpeed, 0+TurnSpeed);
//         else//stop
//         {
//             SET_MOTORS_SPEED(0, 0);
//             PID_Reset(&pid_Angle);
//             PID_Reset(&pid_Gyro);
//             break;
//         }
//     }

// }
// //0
// void Car_Pirouetee_CJ(float Angle)
// {
//     Target_Angle = Target_Angle + Angle;
//     float TurnSpeed = 40;		   
//     float Tolerant_Angle = 5;//宽容范围，允许误差存在                                                                                                                                                                                                                                                                                                                                                                                         
//     while(1)
//     {
//         if((Target_Angle-imu_data.yaw) > Tolerant_Angle)//to left
//         SET_MOTORS_SPEED(0-TurnSpeed, 0+TurnSpeed);
//         else if((Target_Angle-imu_data.yaw) < (-Tolerant_Angle))//to right
//         SET_MOTORS_SPEED(0-TurnSpeed, 0+TurnSpeed);
//         else//stop
//         {
//             SET_MOTORS_SPEED(0, 0); 
//             break;
//         }
//     }

// }