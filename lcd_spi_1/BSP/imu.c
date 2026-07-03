#include "imu.h"


// 全局变量
volatile IMU_Data_t imu_data;
uint8_t rx_buffer[32];        // 接收缓冲区
uint8_t rx_index = 0;         // 缓冲区索引
bool frame_started = false;    // 帧开始标志
float Z_Gyro = 0;

// 解析角速度数据 (0x55 0x52 帧)
void parse_angular_velocity(const uint8_t* data) {
    // 小端模式组合数据并计算角速度
    int16_t wx = (int16_t)((data[3] << 8) | data[2]);
    int16_t wy = (int16_t)((data[5] << 8) | data[4]);
    int16_t wz = (int16_t)((data[7] << 8) | data[6]);
//    Z_Gyro = wz;
    // 根据协议计算角速度 (WT9011G4K需要乘以4000)
    imu_data.angular_velocity_x = (wx / 32768.0f) * 2000.0f;
    imu_data.angular_velocity_y = (wy / 32768.0f) * 2000.0f;
    imu_data.angular_velocity_z = (wz / 32768.0f) * 2000.0f;
		Z_Gyro = imu_data.angular_velocity_z;
}

// 解析角度数据 (0x55 0x53 帧) - 只提取偏航角
void parse_yaw(const uint8_t* data) {
    int16_t yaw = (int16_t)((data[7] << 8) | data[6]);  // YawH:YawL
    imu_data.yaw = (yaw / 32768.0f) * 180.0f;  // 范围 -180°~180°
}


bool verify_checksum(const uint8_t* data, uint8_t length) {
    uint8_t sum = 0;
    for (int i = 0; i < length - 1; i++) {
        sum += data[i];
    }
    return (sum == data[length - 1]);
}



void calibrate_z_axis_zero(void)
{
    /* 1. 发送解锁指令：FF AA 69 88 B5 */
		
    unsigned char unlock_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
		unsigned char calibrate_cmd[] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
		unsigned char save_cmd[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
		uart0_send_string("Start Calibration...\r\n");
    usart3_send_bytes(unlock_cmd, sizeof(unlock_cmd));
    delay_ms(200);  // 延时200ms
    /* 2. 发送校准指令：FF AA 01 04 00 */
    usart3_send_bytes(calibrate_cmd, sizeof(calibrate_cmd));
    delay_ms(3000); // 等待3秒
    /* 3. 发送保存指令：FF AA 00 00 00 */
    usart3_send_bytes(save_cmd, sizeof(save_cmd));
		delay_ms(50); 
		uart0_send_string("Calibration Done!\r\n");
}		

void HWT101_IRQHandler(void) {
    if(DL_UART_getEnabledInterruptStatus(UART_3_INST, DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX) {
        uint8_t uart_data = DL_UART_receiveData(UART_3_INST);
        
        // 帧头检测 (0x55)
        if (uart_data == 0x55 && !frame_started) {
            frame_started = true;
            rx_index = 0;
            rx_buffer[rx_index++] = uart_data;
        } 
        // 如果已经开始接收帧
        else if (frame_started) {
            rx_buffer[rx_index++] = uart_data;
            
            // 检查是否收到完整帧
            if ((rx_buffer[1] == 0x52 && rx_index == 11) ||  // 角速度帧长度
                (rx_buffer[1] == 0x53 && rx_index == 11)) {  // 角度帧长度
                
                // 验证校验和
                if (verify_checksum(rx_buffer, rx_index)) {
									
                   switch (rx_buffer[1]) {
                        case 0x52: parse_angular_velocity(rx_buffer); break;
                        case 0x53: parse_yaw(rx_buffer); break;
												default  : break;
                    }
                }
                
                frame_started = false;
            }
            
            // 防止缓冲区溢出
            if (rx_index >= sizeof(rx_buffer)) {
                frame_started = false;
            }
        }
        
        DL_UART_clearInterruptStatus(UART_3_INST, DL_UART_INTERRUPT_RX);
    }
}
	
