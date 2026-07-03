#ifndef __imu_h
#define __imu_h

#include "board.h"
#include <stdio.h>


#define IMU_DATA_SIZE  11

// 解析结果结构体
typedef struct {
    float angular_velocity_x;  // 角速度X轴 (°/s)
    float angular_velocity_y;  // 角速度Y轴 (°/s)
    float angular_velocity_z;  // 角速度Z轴 (°/s)
    float yaw;                // 偏航角 (度)
} IMU_Data_t;

extern float Z_Gyro;
extern volatile IMU_Data_t imu_data;


void calibrate_z_axis_zero(void);

void HWT101_IRQHandler(void);
#endif


