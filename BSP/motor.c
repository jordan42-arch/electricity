#include "motor.h"
#include "lcd_init.h"
#include "stdio.h"


uint8_t Turn_PID_Flag = 1;
uint8_t Distance_PID_Flag = 0;
uint8_t Gyro_PID_Flag = 0;
uint8_t Angle_PID_Flag = 0;

/*---------------------------------------------------------------------------------------------------*/
/*---------------------------------------PID计算部分部分---------------------------------------------*/
/*---------------------------------------------------------------------------------------------------*/
// 速度环参数修改
pid_t pid_Motor1_Speed = {
    .Kp = 100,
    .Ki = 25,
    .Kd = 10,
    .Target = 0,
    .Measure = 0,
    {0, 0, 0},
    .KpOut = 0,
    .KiOut = 0,
    .KdOut = 0,
    .PID_Out = 0,
    .PID_Limit_MAX = 9999,
    .Ki_Limit_MAX = 9999,
};
pid_t pid_Motor2_Speed = {
    .Kp = 100,
    .Ki = 20,
    .Kd = 10,
    .Target = 0,
    .Measure = 0,
    {0, 0, 0},
    .KpOut = 0,
    .KiOut = 0,
    .KdOut = 0,
    .PID_Out = 0,
    .PID_Limit_MAX = 9999,
    .Ki_Limit_MAX = 9999,
};
// 转向环参数修改			60cm/s	6	140
//右转
pid_t pid_Turn = {
    .Kp = 3,//从5改为3,适配当前task1,
    .Ki = 0,
    .Kd = 15,//从5改为25改为15/改为15，适配当前task1,改为20

    .Target = 0,
    .Measure = 0,
    {0, 0, 0},
    .KpOut = 0,
    .KiOut = 0,
    .KdOut = 0,
    .PID_Out = 0,
    .PID_Limit_MAX = 25,
    .Ki_Limit_MAX = 9999,
};
// 距离环参数修改
pid_t pid_Distance = {
    .Kp = 10,
    .Ki = 0,
    .Kd = 5,
    .Target = 0,
    .Measure = 0,
    {0, 0, 0},
    .KpOut = 0,
    .KiOut = 0,
    .KdOut = 0,
    .PID_Out = 0,
    .PID_Limit_MAX = 9999,
    .Ki_Limit_MAX = 9999,
};
// 角速度环参数修改
pid_t pid_Gyro = {
    .Kp = 0.08,
    .Ki = 0.01,
    .Kd = 0.02,
    .Target = 0,
    .Measure = 0,
    {0, 0, 0},
    .KpOut = 0,
    .KiOut = 0,
    .KdOut = 0,
    .PID_Out = 0,
    .PID_Limit_MAX = 20,//limit=20,基础速度60，60-20>40(最小启动速度),用于直行，直角弯
    .Ki_Limit_MAX = 9999,
};
// 转向环参数修改:
pid_t pid_Angle = {
    .Kp = 1.5,//3.5
    //直线kp推荐1.5
    //2.5为送药小车时期固定数值,用于转弯有不错的效果
    .Ki = 0,
    .Kd = 55,//40
    .Target = 0,
    .Measure = 0,
    {0, 0, 0},
    .KpOut = 0,
    .KiOut = 0,
    .KdOut = 0,
    .PID_Out = 0,
    .PID_Limit_MAX =180,
    .Ki_Limit_MAX = 1000,
};

pid_t pid_Angle_back = {
    .Kp = 1.5,//3.5
    .Ki = 0,
    .Kd = 60,//40
    .Target = 0,
    .Measure = 0,
    {0, 0, 0},
    .KpOut = 0,
    .KiOut = 0,
    .KdOut = 0,
    .PID_Out = 0,
    .PID_Limit_MAX =180,
    .Ki_Limit_MAX = 1000,
};

// 角速度环参数修改
pid_t pid_Pirouetee_Gyro = {
    .Kp = 0.08,
    .Ki = 0.01,
    .Kd = 0.02,
    .Target = 0,
    .Measure = 0,
    {0, 0, 0},
    .KpOut = 0,
    .KiOut = 0,
    .KdOut = 0,
    .PID_Out = 0,
    .PID_Limit_MAX = 40,//40(最小启动速度),for Pirouetee
    .Ki_Limit_MAX = 9999,
};
// 转向环参数修改:
pid_t pid_Pirouetee_Angle = {
    .Kp = 7,
    .Ki = 0,
    .Kd = 50,
    .Target = 0,
    .Measure = 0,
    {0, 0, 0},
    .KpOut = 0,
    .KiOut = 0,
    .KdOut = 0,
    .PID_Out = 0,
    .PID_Limit_MAX =180,
    .Ki_Limit_MAX = 1000,
};

// PWM限幅函数 *a传入要限幅的参数  ABS_MAX限幅大小
void PID_Limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}
/*************************************************************************************************
 *	函 数 名:	PID_Calculate
 *
 *	函数功能:	PID计算公式
 *
 *   参    数：  PID结构体，当前值，目标值
 *************************************************************************************************/
float PID_Calculate(pid_t *pid, float Measure, float Target)
{
    pid->Target = Target;             // 赋值目标值
    pid->Measure = Measure;           // 赋值测量值
    pid->Error[0] = Target - Measure; // 计算当前误差

    pid->KpOut = pid->Kp * (pid->Error[0]);                 // 比例计算，Kp*当前误差
    pid->KiOut += pid->Ki * pid->Error[0];                  // 积分计算，Ki*误差累加值
    PID_Limit(&(pid->KiOut), pid->Ki_Limit_MAX);            // 积分限幅
    pid->KdOut = pid->Kd * (pid->Error[0] - pid->Error[1]); // 微分计算，Kd*(当前误差-上一次误差)

    pid->PID_Out = pid->KpOut + pid->KiOut + pid->KdOut; // PID计算
    PID_Limit(&(pid->PID_Out), pid->PID_Limit_MAX);      // PID输出限幅

    pid->Error[2] = pid->Error[1];
    pid->Error[1] = pid->Error[0];

    return pid->PID_Out;
}

float PID_Calculate_Angle(pid_t *pid, float Measure, float Target)
{
    pid->Target = Target;             // 赋值目标值
    pid->Measure = Measure;           // 赋值测量值
    pid->Error[0] = Target - Measure; // 计算当前误差
    if (pid->Error[0] > 180)
        pid->Error[0] -= 360;
    if (pid->Error[0] < -180)
        pid->Error[0] += 360;
    pid->KpOut = pid->Kp * (pid->Error[0]);                 // 比例计算，Kp*当前误差
    pid->KiOut += pid->Ki * pid->Error[0];                  // 积分计算，Ki*误差累加值
    PID_Limit(&(pid->KiOut), pid->Ki_Limit_MAX);            // 积分限幅
    pid->KdOut = pid->Kd * (pid->Error[0] - pid->Error[1]); // 微分计算，Kd*(当前误差-上一次误差)

    pid->PID_Out = pid->KpOut + pid->KiOut + pid->KdOut; // PID计算
    PID_Limit(&(pid->PID_Out), pid->PID_Limit_MAX);      // PID输出限幅

    pid->Error[2] = pid->Error[1];
    pid->Error[1] = pid->Error[0];

    return pid->PID_Out;
}

void Set_PID_Param(pid_t *pid, float P, float I, float D)
{
    pid->Kp = P;
    pid->Ki = I;
    pid->Kd = D;
    pid->KpOut = 0;
    pid->KiOut = 0;
    pid->KdOut = 0;
    pid->PID_Out = 0; // 输出清零
}

void PID_Reset(pid_t *pid) {
    pid->KpOut = 0;
    pid->KiOut = 0;
    pid->KdOut = 0;
    pid->PID_Out = 0;
    pid->Error[0] = 0;
    pid->Error[1] = 0;
    pid->Error[2] = 0;
}
/*************************************************************************************************
 *	pid调试参数、调试函数//v6
 *************************************************************************************************/
pid_t* current_pid = &pid_Angle; // 默认指向第一个
uint8_t current_param = 3;              // 0: Kp, 1: Ki, 2: Kd， 3：锁定
uint8_t pid_page = 0; // 可以根据需要设为 0 或 1；

//用于根据 current_param 修改对应的 PID 参数
void AdjustPIDParam(uint8_t and_or_sub)//0:加 1:减
{
    float kp = current_pid->Kp;
    float ki = current_pid->Ki;
    float kd = current_pid->Kd;
    float step_p = 0.5; // 默认步长
    float step_i = 0.1;
    float step_d = 1;

     if (current_pid == &pid_Angle) {
         step_p = 0.05; // Angle PID 的步长设置
        step_i = 0.005;
         step_d = 0.5;
     } else if (current_pid == &pid_Turn) {
        step_p = 0.5; // Turn PID 的步长设置
        step_i = 0.001;
        step_d = 2;
    } else if (current_pid == &pid_Gyro) {
         step_p = 0.01; // Gyro PID 的步长设置
         step_i = 0.001;
         step_d = 0.01;
     }
    switch (current_param) {
        case 0: // Kp
            if(and_or_sub == 0)
                kp += step_p;
            else if(and_or_sub == 1)
                kp -= step_p;
            break;
        case 1: // Ki
            if(and_or_sub == 0)
                ki += step_i;
            else if(and_or_sub == 1)
                ki -= step_i;
            break;    
        case 2: // Kd
            if(and_or_sub == 0)
                kd += step_d;
            else if(and_or_sub == 1)
                kd -= step_d;
            break;
        case 3:
            break;
    }

    Set_PID_Param(current_pid, kp, ki, kd);
}
//切换当前 PID 指针函数，用于选择下一个 PID 环
void SelectNextPID(void)
{
    current_param = 3;//pid参数锁定
    pid_page = 0;
    if (current_pid == &pid_Angle) {
         current_pid = &pid_Gyro;
    } else if (current_pid == &pid_Gyro) {
         current_pid = &pid_Turn;
     }else if (current_pid == &pid_Turn) {
         current_pid = &pid_Angle;
    }
}
//函数切换要修改的参数项（Kp/Ki/Kd）
void SelectNextParam(void)
{
    current_param = (current_param + 1) % 4;
}

void DisplayPIDInfo(void)
{
    // 清除屏幕，根据 pid_page 设置背景色
    if (pid_page == 0) {
        LCD_Fill(0, 0, LCD_W, LCD_H, WHITE); // 白色背景
    } else if (pid_page == 1) {
        LCD_Fill(0, 0, LCD_W, LCD_H, GRAY);  // 灰色背景
    }

    // 获取当前 PID 名称
    char *pid_name;
    if (current_pid == &pid_Motor1_Speed) {
        pid_name = "Motor1 Speed";
    } else if (current_pid == &pid_Motor2_Speed) {
        pid_name = "Motor2 Speed";
    } else if (current_pid == &pid_Turn) {
        pid_name = "Turn PID";
    } else if (current_pid == &pid_Distance) {
        pid_name = "Distance PID";
    } else if (current_pid == &pid_Gyro) {
        pid_name = "Gyro PID";
    } else if (current_pid == &pid_Angle) {
        pid_name = "Angle PID";
    } else {
        pid_name = "Unknown";
    }

    // 显示当前 PID 名称
    if(pid_page == 0)
    LCD_ShowString(10, 10, pid_name, BLACK, WHITE, 16, 0);
    else if(pid_page == 1)
    LCD_ShowString(10, 10, pid_name, BLACK, GRAY, 16, 0);

    // 显示 Kp/Ki/Kd 值
    char buffer[32];
    for (int i = 0; i < 3; i++) {
        float value;
        if (i == 0) {
            value = current_pid->Kp;
            sprintf(buffer, "p %.2f", value);
        } else if (i == 1) {
            value = current_pid->Ki;
            sprintf(buffer, "i %.2f", value);
        } else {
            value = current_pid->Kd;
            sprintf(buffer, "d %.2f", value);
        }

        if (i == current_param && pid_page == 0) {
            LCD_ShowString(10, 30 + i * 20, buffer, GREEN, WHITE, 12, 0);
        } else if (i == current_param && pid_page == 1) {
            LCD_ShowString(10, 30 + i * 20, buffer, GREEN, GRAY, 12, 0);
        } else {
            LCD_ShowString(10, 30 + i * 20, buffer, BLACK, WHITE, 12, 0);
        }
    }
}
/*************************************************************************************************/



/*-------------------------------------------------------------------------------------------*/
/*------------------------------------电机操作函数-------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void motor_init(void)
{
    // 小车的TIMER-PWM开始计时（包含四个输出通道：C0、C1、C2、C3），启动电机
    DL_TimerA_startCounter(PWM_Motor_INST); // 开始计数
    SET_MOTORS_SPEED(0, 0);                            // 初始化小车速度为0

    uart0_send_string("motor_init ok!\n");
}


void SET_MOTORS_SPEED(int motor_left_speed, int motor_right_speed)
{
    // 速度上限100，超出则警告则最多100运行
    if (motor_left_speed > 100)
    {
        printf("Left Motor Speed Warning! Max is 100\n");
        motor_left_speed = 100;
    }
    else if (motor_left_speed < -100)
    {
        printf("Left Motor Speed Warning! Min is -100\n");
        motor_left_speed = -100;
    }

    if (motor_right_speed > 100)
    {
        printf("Right Motor Speed Warning! Max is 100\n");
        motor_right_speed = 100;
    }
    else if (motor_right_speed < -100)
    {
        printf("Right Motor Speed Warning! Min is -100\n");
        motor_right_speed = -100;
    }

    if (motor_right_speed < 0) // 想要右轮后退->反转->M-输出PWM，M+输出低电平
    {
        PWM_A1(-motor_right_speed);
        PWM_A2(0);
    }
    else // 想要右轮前进->反转->M+输出低电平，M-输出PWM
    {
        PWM_A1(0);
        PWM_A2(motor_right_speed);
    }

    if (motor_left_speed < 0) // 想要左轮后退->正转->M+输出低电平，M-输出PWM
    {
        PWM_B1(0);
        PWM_B2(-motor_left_speed);
    }
    else // 想要左轮前进->正转->M+输出PWM，M-输出低电平
    {
        PWM_B1(motor_left_speed);
        PWM_B2(0);
    }
}

void Set_Motor1_Speed(int Target_Speed) //左轮
{
    //速度上限100，超出则警告则最多100运行
    if (Target_Speed > 100)
    {
        printf("Left Motor Speed Warning! Max is 100\n");
        Target_Speed = 100;
    }
    else if (Target_Speed < -100)
    {
        printf("Left Motor Speed Warning! Min is -100\n");
        Target_Speed = -100;
    }


    if (Target_Speed < 0) //后退
    {
        PWM_B1(0);
        PWM_B2(-Target_Speed);
    }
    else //前进
    {
        PWM_B1(Target_Speed);
        PWM_B2(0);
    }
}

void Set_Motor2_Speed(int Target_Speed) //右轮
{
    //速度上限100，超出则警告则最多100运行
    if (Target_Speed > 100)
    {
        printf("Right Motor Speed Warning! Max is 100\n");
        Target_Speed = 100;
    }
    else if (Target_Speed < -100)
    {
        printf("Right Motor Speed Warning! Min is -100\n");
        Target_Speed = -100;
    }

    if (Target_Speed < 0) //后退
    {
        PWM_A1(-Target_Speed);
        PWM_A2(0);
    }
    else //前进
    {
        PWM_A1(0);
        PWM_A2(Target_Speed);
    }
}

