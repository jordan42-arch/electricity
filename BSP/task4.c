#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "board.h"
#include "lcd_init.h"
#include "lcd.h"
#include "imu.h"
#include "key.h"
#include "grey.h"
#include "motor.h"
#include "k210_use.h"
#include "car_control.h"
#include "test.h"
#include "task4.h"
//#include <cstdint>
#include <stdint.h>
#include <stdio.h>

void task4(void)
 {
//    LCD_Fill(0, 20, 120, 40, WHITE); // 清除上一次显示区域
//     //LCD_ShowString(10,20,"task4",BLACK,WHITE,16,0);
    
//     int State = 0;
//     int init_num = 0;                       //计数初始识别的数字
//     int left_num = 0, left_count = 0;           //路口的数字计数（左侧）
//     int right_num = 0, right_count = 0;         //路口的数字技术（右侧）
//     int sure_left_num = 0, sure_right_num = 0;
//     // int sure_num = 0;   //锁定判断条件
//     int skip_frame = 0;
//     // int State = 0;
//     int turn = 2;
//     // int init_num = 0;
//     // int skip_frame = 0;
//     int num[4] = {0, 0, 0, 0};
//     int num_count[4] = {0, 0, 0, 0};
//     int sure_num[4] = {0, 0, 0, 0};
//     //int sure_cross = 0;
//     int Angle = 10;
//     int TurnSpeed = 40;
//     int Tolerant_Angle = 5;
//     // int left_num = 0, left_count = 0;           //路口的数字计数（左侧）
//     // int right_num = 0, right_count = 0;         //路口的数字技术（右侧）
//     // int sure_left_num = 0, sure_right_num = 0;
//     int sure_cross = 0;   //锁定判断条件
//     int init_direction = -1; //初始路口的转向方向
//     int terminal = 0;
//     bool go_or_back = true;//true:go false:back
//     int map_depth = 0;//0:出发点 1：近端 2：中断 3：远端左 4：远端右
//     Reset_k210_msg(); //初始清除，防止之前干扰
    
//     while (1) {
//         KeyState key = get_key_state();
//         if (State == 0) {//识别数字
//             init_num = k210_num_sure(); //如果是0，不会跳出该函数；直到识别为有效数字才跳出并返回
//             State = 1;
//             delay_ms(500);
//         }
//         else if(State == 1)//装载出发
//         {
//                 Car_Tracing_High();
//                 Reset_k210_msg();
//                 if(init_num != 1 && init_num != 2){//说明不是1、2进行直走
//                     color_width = 0;//防止后面误判
//                     State = 3;
//                 }    
//                 else {
//                     State = 2;
//                 }
//         }
// //去程：近端
//         else if(State == 2) //尝试识别路口本身，在第一个十字路口进行判断直走、左右转
//         {
//             Start_Tracing(1000, 1);
//             map_depth = 1;//位于近端路口
//             while (1) {
//                 Car_Tracing_LOW();
//                  if (Huidu_Flag == 2 && init_num == 1) {//第一个路口左转进入1
//                     terminal = 1;//终点1
//                     Car_Trun_90(0);//左转
//                     dot_stop();//虚线停车
//                     State = 1001;//返程
//                     break;
//                 }
//                 else if(Huidu_Flag == 2 && init_num == 2){//第一个路口右转进入2
//                     terminal = 2;//重点2
//                     Car_Trun_90(1);//右转
//                     dot_stop();//虚线停车
//                     State = 1001;//返程
//                     break;
//                 }

//             }
//         }
// //去程：中端
//          else if(State == 3) //直走识别到第二个十字路口
//         {
//             Start_Tracing(2800,1);
            
//             while (1) {
//                 Car_Tracing_LOW(); //用较低的速度循迹
//                 if(color_width > 40) //如果识别到路口
//                 {
//                     map_depth = 2;//位于中断路口
//                     Fast_stop();
//                     Reset_k210_msg(); //此时再清除之前的识别信息，以避免将初始识别信息记错为路口识别信息。
//                     State = 4;
//                     break;
//                 }
//             }
//         }

//         else if (State == 4){ //第二个十字路口判断
//             //Car_Tracing_LOW(); //用较低的速度循迹
//             //数字识别（累计左侧和右侧的识别结果和识别计数，到达一定量则确定下来。），但是识别必须隔一定时间再识别。
//             skip_frame++;
//             if(skip_frame > 1 && sure_cross == 0) //这时候再做识别
//             {
//                 skip_frame = 0;
//                 if(k210_msg.x == 500){} //不做处理（表示K210还没有给MSP传识别结果，为默认值0）
//                 else{   
//                     //有识别结果，则计算识别框的中线位置
//                     double middle_x = k210_msg.x + k210_msg.w/2;
//                     // 如果识别框在左侧
//                     if(middle_x <= 160){
//                         // 之后的处理逻辑和sure函数是一样的。
//                         if(k210_num != left_num){
//                             left_num = k210_num;
//                             left_count = 1;
//                         }
//                         else{
//                             left_count++;
//                             if(left_count >= 3){sure_left_num = left_num;}
//                         }
//                     }
//                     // 如果识别框在右侧
//                     else{
//                         // 之后的处理逻辑和sure函数是一样的。
//                         if(k210_num != right_num){
//                             right_num = k210_num;
//                             right_count = 1;
//                         }
//                         else{
//                             right_count++;
//                             if(right_count >= 3){sure_right_num = right_num;}
//                         }
//                     }
//                 }
//             }
//             //每轮识别结束后：
//             //判断路口确定下来的其中某个值 是否等于 初始识别的值。
//             if(sure_left_num == init_num){//第二个十字路口，锁定左边数字进行左转
//                 sure_cross = 1;
//                 terminal = 3;
//                 Car_Tracing_LOW();
//                 if (Huidu_Flag == 2) {
//                     Car_Trun_90(0);
//                     dot_stop();
//                     State = 1001;
//                 }
//             }
//             else if(sure_right_num == init_num){//第二个十字路口，锁定右边数字进行右转
//                 sure_cross = 1;
//                 terminal = 4;
//                 Car_Tracing_LOW();
//                 if (Huidu_Flag == 2) {
//                     Car_Trun_90(1);
//                     dot_stop();
//                     State = 1001;
//                 }
//             }
//             else if (sure_left_num != 0 && sure_right_num != 0) {//并非所需数字，第二个十字路口直走
//                 sure_cross = 1;
//                 Car_Tracing_High();
//                 if (Huidu_Flag == 2) {
//                     //相关识别标志重置便于后续复用
//                     sure_cross = 0;
//                     sure_left_num = 0;
//                     sure_right_num = 0;
//                     left_count = 0;
//                     right_count = 0;
//                     left_num = 0;
//                     right_num = 0;
//                     State = 5;
//                 }
//                 Reset_k210_msg(); //此时再清除之前的识别信息，以避免将初始
//             }
//         }
// //去程：远端1
//         else if(State == 5) //尝试识别路口本身，第三个十字路口识别
//         {
//             Start_Tracing(1000, 1);
//             while (1) {
//                 Car_Tracing_LOW(); //用较低的速度循迹
//                 if(color_width > 30) //如果识别到路口
//                 {
//                     Fast_stop();
//                     Reset_k210_msg(); //此时再清除之前的识别信息，以避免将初始识别信息记错为路口识别信息。
//                     skip_frame = 0; //方便下次使用
//                     State = 7;
//                     break;
//                 }
//             }
//         }
       
//          else if (State == 7) {//第三个十字路口右摆头
//             SET_MOTORS_SPEED(0+TurnSpeed, 0-TurnSpeed);
//             if((-Angle-imu_data.yaw) > -Tolerant_Angle){
//                 Fast_stop();
//                 State = 8;
//             }
//         }
//         else if (State == 8) {//第三个十字路口识别右侧
//             skip_frame++;
//             if(skip_frame > 1) //这时候再做识别
//             {
//                 skip_frame = 0;
//                 if(k210_msg.x == 500){} //不做处理（表示K210还没有给MSP传识别结果，为默认值0）
//                 else{   
//                     //有识别结果，则计算识别框的中线位置
//                     double middle_x = k210_msg.x + k210_msg.w/2;
//                     // 如果识别框在左侧
//                     if(middle_x <= 200 && sure_num[2] == 0){
//                         // 之后的处理逻辑和sure函数是一样的。
//                         if(k210_num != num[2]){
//                             num[2] = k210_num;
//                             num_count[2] = 1;
//                         }
//                         else{
//                             num_count[2]++;
//                             if(num_count[2] >= 2){sure_num[2] = num[2];}
//                         }
//                     }
//                     // 如果识别框在右侧
//                     else if(middle_x >200 && sure_num[3] == 0){
//                         // 之后的处理逻辑和sure函数是一样的。
//                         if(k210_num != num[3]){
//                             num[3] = k210_num;
//                             num_count[3] = 1;
//                         }
//                         else{
//                             num_count[3]++;
//                             if(num_count[3] >= 2){sure_num[3] = num[3];}
//                         }
//                     }
//                 }
//             }
//             if (sure_num[2] != 0 && sure_num[3] != 0) {
//                 State = 9;
//             }
//         }

//         else if (State == 9) {//第三个十字路口锁定后回正，选择左转或者右转
//             SET_MOTORS_SPEED(0-TurnSpeed, 0+TurnSpeed);
//             for (int i = 0; i <= 3; i++) {
//                 if(sure_num[i] == 2){//8容易被误识别为2，这里修正一下
//                     sure_num[i] = 8;
//                 }
//             }
//             if (imu_data.yaw > -Tolerant_Angle) {//回正后，路口左右转
//                 //  Car_Tracing_LOW();
//                 Car_GoStraight(50);
//                 if (Huidu_Flag == 2 &&(sure_num[2] == init_num || sure_num[3] == init_num)) {
//                     init_direction = 0;
//                     map_depth = 4;
//                     Car_Trun_90(1);
//                     State = 10;
//                 }
//                 else if (Huidu_Flag == 2 &&(sure_num[2] != init_num && sure_num[3] != init_num)) {
//                     init_direction = 1;
//                     map_depth = 3;
//                     Car_Trun_90(0);
//                     State = 10;
//                 }
//             }
//         }
// //去程：远端2
//         else if(State == 10) //最后一个十字路口前的循迹和判断
//         {
//             Car_Tracing_LOW(); //低速循迹
//              skip_frame++;
//             if(color_width > 50 && skip_frame > 40) //如果识别到路口
//             {
//                 Fast_stop();
//                 Reset_k210_msg(); //此时再清除之前的识别信息，以避免将初始识别信息记错为路口识别信息。
//                 skip_frame = 0; //方便下次使用
//                 State = 11;
//             }
//         }

//         else if (State == 11){ // 进入真正带路口数字识别
//             skip_frame++;
//             if(skip_frame > 1 && sure_cross == 0) //这时候再做识别（注意此处是sure_cross而不是sure_num!!!）
//             {
//                 skip_frame = 0;
//                 if(k210_msg.x == 500){} //不做处理（表示K210还没有给MSP传识别结果，为默认值0）
//                 else{   
//                     //有识别结果，则计算识别框的中线位置
//                     double middle_x = k210_msg.x + k210_msg.w/2;
//                     // 如果识别框在左侧
//                     if(middle_x <= 160){
//                         // 之后的处理逻辑和sure函数是一样的。
//                         if(k210_num != left_num){
//                             left_num = k210_num;
//                             left_count = 1;
//                         }
//                         else{
//                             left_count++;
//                             if(left_count >= 2){sure_left_num = left_num;}
//                         }
//                     }
//                     // 如果识别框在右侧
//                     else{
//                         // 之后的处理逻辑和sure函数是一样的。
//                         if(k210_num != right_num){
//                             right_num = k210_num;
//                             right_count = 1;
//                         }
//                         else{
//                             right_count++;
//                             if(right_count >= 2){sure_right_num = right_num;}
//                         }
//                     }
//                 }
//             }
//             //每轮识别结束后：
//             //判断路口确定下来的其中某个值 是否等于 初始识别的值。
//             if(sure_left_num == init_num){//锁定左边数字进行左转，复用第二个十字路口的路线规划
//                 sure_cross = 1;
//                 Car_Tracing_LOW();
//                 if (Huidu_Flag == 2) {
//                     terminal = 5;
//                     Car_Trun_90(0);
//                     dot_stop();
//                     State = 1001;
//                 }
//             }
//             else if(sure_right_num == init_num){//锁定右边数字进行右转
//                 sure_cross = 1;
//                 Car_Tracing_LOW();
//                 if (Huidu_Flag == 2) {
//                     terminal = 6;
//                     Car_Trun_90(1);
//                     dot_stop();
//                     State = 1001;
//                 }
//             }
//         }
//     //返程处理
//         else if (State == 1001) {
//             if (key == KEY_K3) 
//                go_or_back = false;//back
//             if(!go_or_back) {
//                 // 统一处理转向方向
//                 uint8_t back_direction = (terminal % 2 == 0) ? 1 : 0;  // 偶数terminal右转，奇数左转
                
//                 // 执行倒车
//                 Car_Back_90(back_direction);
                
//                 // 根据terminal范围设置状态
//                 if (terminal == 5 || terminal == 6) {  // terminal 5和6
//                     State = 1002;       // 进入最后十字路口返程
//                 } else {
//                     State = 13;       // 进入全返程
//                 }
//             }
           
//         }
       
//         else if (State == 1002) {//最后十字路口返程
//             Car_Tracing_High();
//             if (Huidu_Flag == 3 || Huidu_Flag == 4 || Huidu_Flag == 2) {
//                 Car_Trun_90(init_direction);
//                 State = 13;
//             }
//         }
//         else if (State == 1003) {//全返程
//             if(map_depth > 1)
//             {
//                 Start_Tracing(1000, 1);
//             }
//             else {
//                 Start_Tracing(1200, 0);
//             }
//             while(1)
//             {
//                 Car_Tracing_High();
//                 if (color_width == 0 ) {
//                 Fast_stop();
//                 go_or_back = true;
//                 break;
//                  }
//             }
//             break;
//         }
        

//         #if 1
//         //imu显示
//         char buf_imu[32];
        
//         // sprintf(buf_imu, "CW %f\r\n", color_width); // 将偏移值转换为字符串
//         // LCD_ShowString(10, 40, buf_imu, BLACK, WHITE, 16, 0); // 显示字符串

//         sprintf(buf_imu, "yaw %f\r\n", imu_data.yaw); // 将偏移值转换为字符串
//         LCD_ShowString(10, 60, buf_imu, BLACK, WHITE, 16, 0); // 显示字符串

//         sprintf(buf_imu, "S %d\r\n", State); // 将偏移值转换为字符串
//         LCD_ShowString(10, 80, buf_imu, BLACK, WHITE, 16, 0); // 显示字符串

//         sprintf(buf_imu, "n0 %d n1 %d\r\n", num[0], num[1]); // 将偏移值转换为字符串
//         LCD_ShowString(10, 100, buf_imu, BLACK, WHITE, 16, 0); // 显示字符串s

//         sprintf(buf_imu, "n2 %d n3 %d\r\n", num[2], num[3]); // 将偏移值转换为字符串
//         LCD_ShowString(10, 120, buf_imu, BLACK, WHITE, 16, 0); // 显示字符串s

//         sprintf(buf_imu, "i %d\r\n", init_num); // 将偏移值转换为字符串
//         LCD_ShowString(10, 140, buf_imu, BLACK, WHITE, 16, 0); // 显示字符串

//         sprintf(buf_imu, "L %d R %d\r\n", sure_left_num, sure_right_num); // 将偏移值转换为字符串
//         LCD_ShowString(10, 20, buf_imu, BLACK, WHITE, 16, 0); // 显示字符串s
        

//         // 检测按键是否切换任务
//         if (key == KEY_K2) {
//             page_flag = 1;
//             SET_MOTORS_SPEED(0,0);
//             break; // 如果 K2 ，则退出 test 模式
//         }
//         #endif
//     }
 }
