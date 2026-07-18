#ifndef __K210_USE_H_
#define __K210_USE_H_




#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "stdint.h"
#include "ti_msp_dl_config.h"


typedef struct msg_buf
{
	uint16_t x; //横坐标
	uint16_t y; //纵坐标
	uint16_t w; //宽度
	uint16_t h; //长度
	uint16_t id; //标签（字符串类型）
	uint8_t class_n;//例程编号
	char msg_msg[30]; //有效数据位 3 0.87
}msg_k210;


/********k210通信相关************/
void recv_k210msg(uint8_t recv_msg);
void deal_recvmsg(void);
void deal_data(uint8_t egnum);

extern msg_k210 k210_msg;//收到k210信息结构体
extern int k210_num; //识别到的数字(int类型)
extern float k210_p; //识别的置信度
extern int color_width; //色块最大宽度（如果为0，说明没有识别到色块）
extern int color_center;    //获取最大色块的中心坐标（用于色块循迹）
extern int light_x; //识别到的激光的x坐标
extern int light_y; //识别到的激光的y坐标


//k210调用函数
int8_t k210_num_sure();
void Reset_k210_msg();
/*使用k210_msg的示例：


*/


#endif


