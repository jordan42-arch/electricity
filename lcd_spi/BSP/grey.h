#ifndef __GREY_H
#define __GREY_H

#include <stdint.h>
#include <stdbool.h>


// void IR_init();
// /**
//  * @brief 根据灰度传感器值计算小车偏离中心线的程度
//  * @param greySensorValues 指向7个传感器状态的数组（0=未检测到黑线，1=检测到）
//  * @return 偏移量，负值表示偏左，正值表示偏右，0表示在中心
//  *         如果传感器组合无效（不连续或 >2 个激活），返回上一次的有效值
//  */
// void Get_IR();
void Get_Huidu(void);


extern uint8_t Huidu_Flag;//灰度类型标志，原Huidu_Datas
extern uint8_t Huidu_Pattern;//7路灰度原始位图，非0表示至少一路检测到黑线
/*IR
*0:非法状态
*1：单线(仅连续两路获一路识别到)
*2：十字路口(六个及以上识别到)
*3：右转T形路口(L1-R3 or M-R3)
*4：左转T形路口(L3-R1 or L3-M)
*5：黑白相间线
*/
extern int8_t Huidu_Target;//巡线目标值，用于循迹pid
extern int8_t Huidu_Error;//灰度偏差值

#endif /* __GREY_H */
