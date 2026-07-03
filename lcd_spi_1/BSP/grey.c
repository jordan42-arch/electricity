#include "grey.h"
#include "board.h"

// 合法模式结构体（新增 flag 字段用于表示状态）
typedef struct {
    uint8_t pattern;      // 7位掩码
    int8_t deviation;     // 对应偏移值
    uint8_t flag;         // 对应状态标志 (0: 非法, 1: 单线, 2: 十字路口, 3: 右转T形, 4: 左转T形)
} DeviationPattern;

// 查找表（包含偏移值和状态标识）
static const DeviationPattern validPatterns[] = {

    // 右转路口模式
    {0b0011111, -8, 2},  
    {0b0001111, -8, 2}, 
    {0b0000111, -8, 2},   

    // 左转路口模式
    // {0b1111100, 8, 3}, 
    // {0b1111000, 8, 3}, 
    // {0b1110000, 8, 3}, 
    // {0b0111000, 8, 3}, 
    // {0b1011100, 8, 3}, 
    // {0b1101100, 8, 3}, 
    // {0b1110100, 8, 3}, 
    // {0b1111000, 8, 3}, 
    // {0b0111000, 8, 3},
    // {0b1011000, 8, 3},
    // {0b1101000, 8, 3},
    // {0b1110000, 8, 3}, 
    
    {0b1111100, 0, 3}, 
    {0b1111000, 0, 3}, 
    {0b1110000, 0, 3}, 
    {0b0111000, 0, 3}, 
    {0b1011100, 0, 3}, 
    {0b1101100, 0, 3}, 
    {0b1110100, 0, 3}, 
    {0b1111000, 0, 3}, 
    {0b0111000, 0, 3},
    {0b1011000, 0, 3},
    {0b1101000, 0, 3},
    {0b1110000, 0, 3}, 
    
   
        // 单线模式
    {0b0000001, -6, 1}, 
    {0b0000011, -5, 1},
    {0b0000010, -4, 1},   // 单点中间偏移
    {0b0000110, -3, 1},
    {0b0000100, -2, 1},
    {0b0001100, -1, 1},
    {0b0001000,  0, 1},
    {0b0011000, +1, 1},
    {0b0010000, +2, 1},
    {0b0110000, +3, 1},
    {0b0100000, +4, 1},
    {0b1100000, +5, 1},
    {0b1000000, +6, 1},
};

// 上一次的有效偏移值
static int8_t lastValidDeviation = 0;
uint8_t Huidu_Flag;
int8_t Huidu_Target = 0;
int8_t Huidu_Error;


void Get_Huidu(void)
{
    uint8_t pattern = 0;

    uint32_t gpio_GREY = DL_GPIO_readPins(GREY_PORT,
        GREY_L3_PIN | GREY_L2_PIN | GREY_L1_PIN | GREY_M_PIN |
        GREY_R1_PIN | GREY_R2_PIN | GREY_R3_PIN);

    char greySensorValues[7];
    greySensorValues[0] = (gpio_GREY & GREY_L3_PIN) != 0;
    greySensorValues[1] = (gpio_GREY & GREY_L2_PIN) != 0;
    greySensorValues[2] = (gpio_GREY & GREY_L1_PIN) != 0;
    greySensorValues[3] = (gpio_GREY & GREY_M_PIN)  != 0;
    greySensorValues[4] = (gpio_GREY & GREY_R1_PIN) != 0;
    greySensorValues[5] = (gpio_GREY & GREY_R2_PIN) != 0;
    greySensorValues[6] = (gpio_GREY & GREY_R3_PIN) != 0;

    for (int i = 0; i < 7; i++) {
        if (greySensorValues[i]) {
            pattern |= (1 << (6 - i));  // 构建 7 位模式
        }
    }

    // 检查是否为合法模式
    Huidu_Flag = 0;
    for (int i = 0; i < sizeof(validPatterns)/sizeof(validPatterns[0]); i++) {
        if (validPatterns[i].pattern == pattern) {
            Huidu_Error  = validPatterns[i].deviation;
            Huidu_Flag = validPatterns[i].flag;
            break;
        }
    }

}
    