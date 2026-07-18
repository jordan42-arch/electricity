#ifndef __KEY_H
#define __KEY_H

#include <stdbool.h>
#include <stdint.h>
extern bool red_flag;

// 枚举定义必须暴露给外部模块使用
typedef enum _KeyState {
    KEY_NONE = 0,
    KEY_K1,
    KEY_K2,
    KEY_K3,
    KEY_RED_GET,
    KEY_RED_REMOVE
} KeyState;
// 函数声明
KeyState get_key_state(void);
KeyState get_red_state(void);
// void get_drug(void);
//红外调用示例
//get_drug();
//  if(red_flag){
//                 LCD_ShowString(10, 100, "drug_get   ", BLACK, WHITE, 16, 0); // 显示字符串
//             }
//             else if(!red_flag){
//                 LCD_ShowString(10, 100, "drug_remove", BLACK, WHITE, 16, 0); // 显示字符串
//             }
#endif /* __KEY_H */
