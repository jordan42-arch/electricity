#ifndef __ENCODER_H__
#define __ENCODER_H__
# include <stdio.h>
/*
说明：
1.该模块是编码器测量模块。已经实现用定时器自动周期性触发测量更新，得到下面所提到的变量值数据：
    左、右轮的当前脉冲计数（一般用不到），左、右轮测得的速度(cm/s)、路程(cm)、平均路程(cm)。
2.该模块的采样窗口时间SAMPLE_TIME在下面作定义，它决定了数据读取、更新的频率。
3.该头文件声明的测量数据变量需要在主函数中做初始化。

使用方式：
1.在主函数中启动该模块、以及该模块相关的中断（3个）。
2.允许随时通过读取定义的测量数据相关的全局变量来获取测量值。

示例：
while (1)
{
    uart0_send_string("R: ");
    uart0_send_int(gEncoderCount_R);
    uart0_send_string("L: ");
    uart0_send_int(gEncoderCount_L);
    uart0_send_char('\n');
    delay_ms(10);   //数据的最大更新周期是SAMPLE_TIME = 0.01s =10ms
}
*/
#include "ti_msp_dl_config.h"
// 电机基本参数(512电机编码器)
#define ENCODE_13X 11    // 编码器线数
#define JIANSUBI 40      // 减速比
#define BEIPIN 2         // 倍频（就是实际计数是用A、B两线同时来计数，计出来会是多组脉冲数的总和。）
#define SAMPLE_TIME 0.1 // 采样时间（在多长的时间窗口内统计接收到的脉冲的次数，单位为s）
#define CC (ENCODE_13X * JIANSUBI * BEIPIN * SAMPLE_TIME)
// 每秒转的圈数=gEncoderCount_R/(ENCODE_13X*JIANSUBI*BEIPIN*SAMPLE_TIME)
// 即给定采样时间内的脉冲计数/（编码器线数*电机减速比*倍频值*采样时间）

#define PI 3.1415f
#define RR 6.5f // 车轮直径（cm)（已测）

#define Measure_Encode_Frequency 40000                                // 用于测量的定时器的周期
#define Measure_Encode_Count (SAMPLE_TIME * Measure_Encode_Frequency) // 用于测量的定时器的计数值。

// 右轮电机反转是前进；左轮电机正转是前进。

// 测得的变量值
// 关于读取编码器脉冲数的变量
extern volatile int32_t gEncoderCount_R; // 右轮的脉冲测量值
extern volatile int32_t gEncoderCount_L; // 左轮的脉冲测量值
// 电机速度、小车两轮路程、平均路程测量计算值
extern volatile float Motor1_Lucheng, Motor2_Lucheng;
extern volatile float Measure_Distance;
extern volatile float Motor1_Lucheng;
extern volatile float Motor1_Lucheng;
extern volatile float Motor2_Speed; // 每分几转（右）
extern volatile float Motor1_Speed; // 每分几转（左）

typedef struct {
    float segment_start_distance; // 分段起点时的总路程
    float segment_distance;       // 当前分段已走距离
} DistanceTracker;

extern volatile DistanceTracker dist_tracker;


// 初始化函数
void encoder_init(void);
// 单独获取一个轮子的速度
void Motor1_Get_Speed(void);//左
void Motor2_Get_Speed(void);//右
// 测量所有电机速度、路程
void MEASURE_MOTORS_SPEED(void);
//重置路程
//void Reset_Segment_Distance(void);
void Reset_Distance(void);
#endif
