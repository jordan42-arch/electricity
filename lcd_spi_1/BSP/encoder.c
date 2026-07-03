#include "encoder.h"
#include "stdio.h"


volatile int32_t gEncoderCount_R = 0;
volatile int32_t gEncoderCount_L = 0;

volatile float Motor1_Lucheng = 0;
volatile float Motor2_Lucheng = 0;

volatile float Measure_Distance = 0;

volatile float Motor2_Speed = 0; //右轮转速
volatile float Motor1_Speed = 0; //左轮转速

volatile DistanceTracker dist_tracker = {0};

//初始化函数
void encoder_init(void)
{
    // 清除串口中断标志
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_Encoder_R_INT_IRQN); // 脉冲计数中断
    NVIC_ClearPendingIRQ(GPIO_Encoder_L_INT_IRQN); // 脉冲计数中断
    // 使能串口中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_Encoder_R_INT_IRQN);
    NVIC_EnableIRQ(GPIO_Encoder_L_INT_IRQN);
    
    printf("encoder_init ok!");
}



/*
处理A、B相脉冲计数的中断函数
*/
void GROUP1_IRQHandler(void)    // GROUP1的中断处理函数
{
    #if 0
    // 获取中断信号
    uint32_t gpioR = DL_GPIO_getEnabledInterruptStatus(GPIOA,
    GPIO_Encoder_R_PIN_0_PIN | GPIO_Encoder_R_PIN_1_PIN);
    uint32_t gpioL = DL_GPIO_getEnabledInterruptStatus(GPIOB,
    GPIO_Encoder_L_PIN_2_PIN | GPIO_Encoder_L_PIN_3_PIN);

    // 如果是GPIO_Encoder_R_PIN_0_PIN产生的中断
    if((gpioR & GPIO_Encoder_R_PIN_0_PIN) == GPIO_Encoder_R_PIN_0_PIN)
    {
        // Pin0上升沿，查看Pin1的电平，为低电平则判断为反转，高电平判断为正转
        if(!DL_GPIO_readPins(GPIOA,GPIO_Encoder_R_PIN_1_PIN))// P1为低电平
        {
            gEncoderCount_R--;
        }
        else// P1为高电平
        {
            gEncoderCount_R++;
        }
    }
    // 类似STM32中编码器模式的AB相检测，可得到2倍的计数
    else if((gpioR & GPIO_Encoder_R_PIN_1_PIN) == GPIO_Encoder_R_PIN_1_PIN)
    {
        // Pin1上升沿（B相），此时A相为高电平则为反转；A相为低电平则为正转
        if(!DL_GPIO_readPins(GPIOA,GPIO_Encoder_R_PIN_0_PIN))// P0为低电平
        {
            gEncoderCount_R++;
        }
        else// P1为高电平
        {
            gEncoderCount_R--;
        }
    }

    // 如果是GPIO_Encoder_L_PIN_2_PIN产生的中断
    if((gpioL & GPIO_Encoder_L_PIN_2_PIN) == GPIO_Encoder_L_PIN_2_PIN)
    {
        // Pin0上升沿，查看Pin1的电平，为低电平则判断为反转，高电平判断为正转
        if(!DL_GPIO_readPins(GPIOB,GPIO_Encoder_L_PIN_3_PIN))// P1为低电平
        {
            gEncoderCount_L--;
        }
        else// P1为高电平
        {
            gEncoderCount_L++;
        }
    }
    // 类似STM32中编码器模式的AB相检测，可得到2倍的计数
    else if((gpioL & GPIO_Encoder_L_PIN_3_PIN) == GPIO_Encoder_L_PIN_3_PIN)
    {
        // Pin1上升沿（B相），此时A相为高电平则为反转；A相为低电平则为正转
        if(!DL_GPIO_readPins(GPIOB,GPIO_Encoder_L_PIN_2_PIN))// P0为低电平
        {
            gEncoderCount_L++;
        }
        else// P1为高电平
        {
            gEncoderCount_L--;
        }
    }
    DL_GPIO_clearInterruptStatus(GPIOA, GPIO_Encoder_R_PIN_0_PIN|GPIO_Encoder_R_PIN_1_PIN);
    DL_GPIO_clearInterruptStatus(GPIOB, GPIO_Encoder_L_PIN_2_PIN|GPIO_Encoder_L_PIN_3_PIN);
    #endif

    #if 1
    // 获取中断状态（保留原始读取）
     uint32_t gpioR = DL_GPIO_getEnabledInterruptStatus(GPIOA, 
                    GPIO_Encoder_R_PIN_0_PIN | GPIO_Encoder_R_PIN_1_PIN);
    uint32_t gpioL = DL_GPIO_getEnabledInterruptStatus(GPIOB,
                    GPIO_Encoder_L_PIN_2_PIN | GPIO_Encoder_L_PIN_3_PIN);
    // 处理右侧编码器（GPIOA）
    //GPIO_Encoder_R_PIN_0_PIN触发的中断
    if((gpioR & GPIO_Encoder_R_PIN_0_PIN) == GPIO_Encoder_R_PIN_0_PIN) {
        // 方向判断：当PIN0上升沿时，PIN1为低电平，则反转
        if (!DL_GPIO_readPins(GPIOA, GPIO_Encoder_R_PIN_1_PIN))
        {
            gEncoderCount_R--;
        }   
        else 
        {  //高电平，则正转
            gEncoderCount_R++;
        }
    }

    //GPIO_Encoder_R_PIN_1_PIN触发的中断
    if((gpioR & GPIO_Encoder_R_PIN_1_PIN) == GPIO_Encoder_R_PIN_1_PIN) {
        // 方向判断：当PIN1上升沿时，PIN0电平为低电平，则正转
        if (!DL_GPIO_readPins(GPIOA, GPIO_Encoder_R_PIN_0_PIN))
        {
            gEncoderCount_R++;
        }
        else 
        {  //高电平，则反转
            gEncoderCount_R--;
        }
    }

    // 处理左侧编码器（GPIOB）
    if((gpioL & GPIO_Encoder_L_PIN_2_PIN) == GPIO_Encoder_L_PIN_2_PIN) {
        // 方向判断：当PIN2上升沿时，PIN3低电平，则反转，高电平则正转
        if (!DL_GPIO_readPins(GPIOB, GPIO_Encoder_L_PIN_3_PIN))
        {
            gEncoderCount_L--;
        }
        else 
        {   // 高电平，则正转
            gEncoderCount_L++;
        }
    }

    if((gpioL & GPIO_Encoder_L_PIN_3_PIN) == GPIO_Encoder_L_PIN_3_PIN) {
        // 方向判断：当PIN3上升沿时，PIN2低电平，则正转，高电平则反转
        if (!DL_GPIO_readPins(GPIOB, GPIO_Encoder_L_PIN_2_PIN))
        {
            gEncoderCount_L++;
        }
        else 
        {   // 高电平，则反转
            gEncoderCount_L--;
        }
    }

    // 清除中断标志（使用实际触发的中断标志）
    DL_GPIO_clearInterruptStatus(GPIOA, GPIO_Encoder_R_PIN_0_PIN | GPIO_Encoder_R_PIN_1_PIN);
    DL_GPIO_clearInterruptStatus(GPIOB, GPIO_Encoder_L_PIN_2_PIN | GPIO_Encoder_L_PIN_3_PIN);
    #endif
}

// 左速
void Motor1_Get_Speed(void){
    int32_t Encoder_TIM = 0;
    float Speed = 0;
    Encoder_TIM= gEncoderCount_L; // 使用脉冲计数值代入计算
    gEncoderCount_L=0;
    //Speed =(float)Encoder_TIM*PI*RR/CC;// 计算速度cm/s
    Speed = (float)Encoder_TIM*PI*RR/CC; // 计算速度：(Encoder_TIM/(CC))=每秒多少转
    Motor1_Speed = -Speed;   // 左轮前进是计数为负值
}
// 右速
void Motor2_Get_Speed(void){
    int32_t Encoder_TIM = 0;
    float Speed = 0;
    Encoder_TIM = gEncoderCount_R;      // 使用脉冲计数值代入计算
    gEncoderCount_R = 0;
    //Speed = (float)Encoder_TIM*PI*RR/CC; // 计算速度：(Encoder_TIM/(CC))=每秒多少转
    Speed = (float)Encoder_TIM*PI*RR/CC; // 计算速度：(Encoder_TIM/(CC))=每秒多少转
    Motor2_Speed = Speed;               // 右轮前进时计数为正值
}

// 汇总测量所有电机速度、路程
void MEASURE_MOTORS_SPEED(void){
    Motor1_Get_Speed();
    Motor2_Get_Speed();
    
    Motor1_Lucheng += Motor1_Speed * SAMPLE_TIME; // 路程累计（cm）
    Motor2_Lucheng += Motor2_Speed * SAMPLE_TIME; // 路程累计（cm）
    Measure_Distance = Motor1_Lucheng/2.0 + Motor2_Lucheng/2.0;  // 左、右轮的平均路程
    
    if(Measure_Distance > 10000)  // 达到100m的路程后，置0
    {
        Measure_Distance = 0;
        Motor1_Lucheng = 0;
        Motor2_Lucheng = 0;
    }
}

void Reset_Distance(void) {
    Measure_Distance = 0;
    Motor1_Lucheng = 0;
    Motor2_Lucheng = 0;
}

// 状态机中重置分段距离
void Reset_Segment_Distance(void)
{
    dist_tracker.segment_start_distance = Measure_Distance;
    dist_tracker.segment_distance = 0;
    printf("SegmentDist Reset\n");
//    uart0_send_float(Measure_Distance, 1);
//    uart0_send_string("\r\n");
}