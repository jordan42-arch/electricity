#include "board.h"
#include "stdio.h"
#include <string.h>

#define RE_0_BUFF_LEN_MAX	128

volatile uint8_t  recv0_buff[RE_0_BUFF_LEN_MAX] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t  recv0_flag = 0;
uint8_t page_flag =0;//翻页防抖

void board_init(void)
{
	// SYSCFG初始化 SYSCFG Initialization
	//SYSCFG_DL_init();
	//清除串口中断标志 Clear the serial port interrupt flag
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	//使能串口中断 Enable serial port interrupt
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    //清除串口中断标志 Clear the serial port interrupt flag
	NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
	//使能串口中断 Enable serial port interrupt
	NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    //v4
	//清除串口中断标志 Clear the serial port interrupt flag
	NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
	//使能串口中断 Enable serial port interrupt
	NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
    //清除定时器中断标志
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    //使能定时器中断
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	
	printf("Board Init [[ ** LCKFB ** ]]\r\n");
}

//声光提示
//关闭设置在定时器中
bool alarm_enabled = 0;
void Start_Alarm(void)
{
    alarm_enabled = 1;
    LED1_ON();
    BEEP_ON();
}

//搭配滴答定时器实现的精确us延时 Accurate us delay with tick timer
void delay_us(unsigned long __us) 
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;

    // 计算需要的时钟数 = 延迟微秒数 * 每微秒的时钟数
	// Calculate the number of clocks required = delay microseconds * number of clocks per microsecond
    ticks = __us * (32000000 / 1000000);

    // 获取当前的SysTick值 Get the current SysTick value
    told = SysTick->VAL;

    while (1)
    {
        // 重复刷新获取当前的SysTick值 Repeatedly refresh to get the current SysTick value
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;

            told = tnow;

            // 如果达到了需要的时钟数，就退出循环
			// If the required number of clocks is reached, exit the loop
            if (tcnt >= ticks)
                break;
        }
    }
}
//搭配滴答定时器实现的精确ms延时 Accurate ms delay with tick timer
void delay_ms(unsigned long ms) 
{
	delay_us( ms * 1000 );
}

void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }

//串口发送单个字符 Send a single character through the serial port
void uart0_send_char(char ch)
{
	//当串口0忙的时候等待，不忙的时候再发送传进来的字符
	// Wait when serial port 0 is busy, and send the incoming characters when it is not busy
	while( DL_UART_isBusy(UART_0_INST) == true );
	//发送单个字符 Send a single character
	DL_UART_Main_transmitData(UART_0_INST, ch);

}
//串口发送字符串 Send string via serial port
void uart0_send_string(char* str)
{
	//当前字符串地址不在结尾 并且 字符串首地址不为空
	// The current string address is not at the end and the string's first address is not empty
	while(*str!=0&&str!=0)
	{
		//发送字符串首地址中的字符，并且在发送完成之后首地址自增
		// Send the characters in the first address of the string, and the first address will increment automatically after the sending is completed.
		uart0_send_char(*str++);
	}
}


/*
整数字符串的两个函数
*/
/**
 * @brief 原地反转字符串
 * @param s 要反转的字符串指针（必须为合法字符串，以'\0'结尾）
 * @return 反转后的字符串指针（与输入指针相同）
 * @note 使用双指针法，时间复杂度O(n/2)
 */
char *reverse(char *s)
{
    char *p = s;                 // 头部指针，初始指向字符串起始位置
    char *q = s + strlen(s) - 1; // 尾部指针，跳过'\0'指向最后一个有效字符

    // 双指针向中间移动并交换字符，直到p >= q
    while (q > p)
    {
        // 交换头部和尾部字符
        char temp = *p;
        *p = *q;
        *q = temp;

        // 移动指针：头部向后，尾部向前
        p++;
        q--;
    }
    return s; // 返回反转后的字符串
}

/**
 * @brief 将整数转换为字符串并存入外部缓冲区
 * @param n   要转换的整数（支持所有int范围值，包括INT_MIN）
 * @return 转换后的字符串指针（与s相同）
 * @warning 缓冲区长度需 >= 12（INT_MIN的"-2147483648"占11字符+'\0'）
 */
char *my_itoa(int n)
{
    int i = 0, isNegative = 0;
    static char s[100];       // 必须为static变量，或者是全局变量
    if ((isNegative = n) < 0) // 如果是负数，先转为正数
    {
        n = -n;
    }
    do // 从各位开始变为字符，直到最高位，最后应该反转
    {
        s[i++] = n % 10 + '0';
        n = n / 10;
    } while (n > 0);

    if (isNegative < 0) // 如果是负数，补上负号
    {
        s[i++] = '-';
    }
    s[i] = '\0'; // 最后加上字符串结束符
    return reverse(s);
}

/*
串口发送内容的函数汇总
*/
// 串口发送一个整数
void uart0_send_int(int num)
{
    // 先将整数转换为字符串，再用字符串发送函数发送
    char buffer[50];
    strcpy(buffer, my_itoa(num)); // 正确：将字符串拷贝到数组
    // strcat(buffer, "\n");         // 手动加入换行，使输出结果更易看清
    uart0_send_string(buffer); // 调用字符串输出函数，输出数字字符串
}




/*-----------------------------------
------------uart_3与imu通信-----------
-------------------------------------*/
//发送单个字节
void usart3_send_byte(unsigned char byte)
{
	DL_UART_Main_transmitDataBlocking(UART_3_INST, byte);
}

//串口发送单个字符
void uart3_send_char(char ch)
{
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(UART_3_INST) == true );
    //发送单个字符
    DL_UART_Main_transmitData(UART_3_INST, ch);
}

//发送数组
void usart3_send_bytes(unsigned char *buf, int len)
{
  while(len--)
  {
		DL_UART_Main_transmitDataBlocking(UART_3_INST, *buf);
    buf++;
  }
}






#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
//If you don't use the micro library, you need to add the following function
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
//If the compiler is AC5, define the following structure
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//定义_sys_exit()以避免使用半主机模式 Define _sys_exit() to avoid using semihosting mode
void _sys_exit(int x)
{
	x = x;
}
#endif


//printf函数重定义 printf function redefinition
int fputc(int ch, FILE *stream)
{
	//当串口0忙的时候等待，不忙的时候再发送传进来的字符
	// Wait when serial port 0 is busy, and send the incoming characters when it is not busy
	while( DL_UART_isBusy(UART_0_INST) == true );
	
	DL_UART_Main_transmitData(UART_0_INST, ch);
	
	return ch;
}

//串口的中断服务函数 Serial port interrupt service function
volatile unsigned char receivedData = 0;
void UART_0_INST_IRQHandler(void)
{
	
	//如果产生了串口中断 If a serial port interrupt occurs
	switch( DL_UART_getPendingInterrupt(UART_0_INST) )
	{
		case DL_UART_IIDX_RX://如果是接收中断 If it is a receive interrupt
			
			// 接收发送过来的数据保存 Receive and save the data sent
			receivedData = DL_UART_Main_receiveData(UART_0_INST);
            //uart0_send_char(receivedData); //不作处理
            recv_k210msg(receivedData); //解析k210数据
           
		
			break;
		
		default://其他的串口中断 Other serial port interrupts
			break;
	}
}

// UART1的接收中断
volatile unsigned char uart_data = 0;
void UART_1_INST_IRQHandler(void)
{
	//如果产生了串口中断 If a serial port interrupt occurs
	switch( DL_UART_getPendingInterrupt(UART_1_INST) )
	{
		case DL_UART_IIDX_RX://如果是接收中断 If it is a receive interrupt
			
			// 接收发送过来的数据保存 Receive and save the data sent
			uart_data = DL_UART_Main_receiveData(UART_1_INST);
            recv_k210msg(uart_data); //解析k210数据
		
			break;
		
		default://其他的串口中断 Other serial port interrupts
			break;
	}
}

//串口3中断服务函数
void UART_3_INST_IRQHandler(void)
{
	
	//接收中断
  HWT101_IRQHandler();
}