# Untitled - By: 86151 - Fri Aug 1 2025
# 继承自版本8，仅增加适用于第二问的灯光控制
import time, os, sys, utime, image
from machine import PWM, FPIOA

#from machine import Timer
#tim = Timer(-1)

from media.sensor import *
from media.display import *
from media.media import *

"""PWM控制相关"""
# 配置排针引脚号35，芯片引脚号为42的排针复用为PWM通道0输出
pwm_io = FPIOA()
pwm_io.set_function(42, FPIOA.PWM0)
pwm_io.set_function(52, FPIOA.PWM4)
# 初始化PWM参数
# 默认频率50Hz,占空比50%
pwm0 = PWM(0,50,7.5,enable = True)
pwm4 = PWM(4,50,7.5,enable = True)

"""UART相关"""
from machine import UART
fpioa = FPIOA()
#fpioa.help()
# 将指定引脚配置为 UART 功能
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)
uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
read_data = None

"""RGB灯相关"""
from machine import Pin
# 设置引脚功能，将指定的引脚配置为普通GPIO功能,
fpioa.set_function(62,FPIOA.GPIO62)
fpioa.set_function(20,FPIOA.GPIO20)
fpioa.set_function(63,FPIOA.GPIO63)
# 实例化Pin62, Pin20, Pin63为输出，分别用于控制红、绿、蓝三个LED灯
LED_R = Pin(62, Pin.OUT, pull=Pin.PULL_NONE, drive=7)  # 红灯
LED_G = Pin(20, Pin.OUT, pull=Pin.PULL_NONE, drive=7)  # 绿灯
LED_B = Pin(63, Pin.OUT, pull=Pin.PULL_NONE, drive=7)  # 蓝灯
# 初始化
LED_R.high()  # 关闭红灯
LED_G.high()  # 关闭绿灯
LED_B.high()  # 关闭蓝灯
# 映射
def LED_G_ON():
    LED_G.low()
def LED_G_OFF():
    LED_G.high()
def LED_R_ON():
    LED_R.low()
def LED_R_OFF():
    LED_R.high()
def LED_B_ON():
    LED_B.low()
def LED_B_OFF():
    LED_B.high()

"""设置继电器"""
laser = Pin(32, Pin.OUT, pull=Pin.PULL_NONE, drive=7)  # 初始化，默认低电平
laser.value(0)

#软件PWM
class SoftPWM:
    """
    纯软件 PWM 计算器
    - freq: 期望频率 (Hz)
    - bits: 计数器位数，默认 16 (0-65535)
    - duty: 占空比 0.0-1.0
    """
    def __init__(self, freq=1000, bits=16):
        self.max = (1 << bits) - 1
        self.period_us = 1_000_000 // freq
        self.t0 = time.ticks_us()

    def set_duty(self, duty):
        """duty: 0.0 – 1.0"""
        self.threshold = int(duty * self.max)

    def sample(self):
        """返回本时刻的理论电平（True 为高，False 为低）"""
        t = time.ticks_diff(time.ticks_us(), self.t0)
        cnt = (t * 0xFFFF) // self.period_us & self.max  # 循环计数器
        return cnt < self.threshold

# 下层舵机PWM控制
def SetBottomSpeed(d_PWM):
    PWM = 7.5 - d_PWM
    pwm0.duty(PWM)

#上层舵机PWM控制（之后控制为固定角度）
def SetTopAngle(angle):
    PWM = 2.5 + angle/180 *10
    pwm4.duty(PWM)

##定时器相关参数、函数
#speed_signed = 0
#count_tim = 0
#step_time =  3#step_time*period=work_time
#servo_flag = False

#def call_back_1ms(t, speed_callback):
#    global count_tim, servo_flag, speed_signed
#    if servo_flag:
#        count_tim += 1
#        if count_tim < step_time:
#            # 舵机工作
#            SetBottomSpeed(speed_callback)  # 实际应该控制舵机引脚
#        else:
#            # 舵机暂停
#            SetBottomSpeed(0)
#            count_tim = 0
#            servo_flag = False
#    else:
#        # 舵机不工作
#        SetBottomSpeed(0)

#def servo_step(work_speed):
#    global servo_flag, speed_signed
#    servo_flag = True
#    speed_signed = work_speed
#    count_tim = 0  # 重置计时器

###定时器初始化
#tim.init(period=5, mode=Timer.PERIODIC, callback=lambda t: call_back_1ms(t, speed_signed))

"""矩形识别相关"""
def line_intersection(line1, line2):
        """计算两条直线的交点"""
        (x1, y1), (x2, y2) = line1
        (x3, y3), (x4, y4) = line2

        den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if den == 0:
            return None  # 平行线无交点

        px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / den
        py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / den
        return (px, py)

def calculate_center_and_validate(corners):
    """
    计算四边形中心点，并验证是否合理
    参数:
        corners: 四个角点坐标 ((x1,y1), (x2,y2), (x3,y3), (x4,y4))
        min_area: 最小允许面积（默认1000像素）
    返回:
        (cx, cy) 或 None（如果四边形不合理）
    """
    # --- 检查四边形是否合理 ---
    # 解包坐标点
#    threshold = 60
#    y_mean = (corners[0][1]+corners[1][1]+corners[2][1]+corners[3][1])/4
#    if not (180-threshold <= y_mean <= 180+threshold):  # 允许180±20的偏差
#        return None

    # 2. 检查对角线交点是否在四边形内部（粗略验证）
    diagonal1 = (corners[0], corners[2])
    diagonal2 = (corners[1], corners[3])
    center = line_intersection(diagonal1, diagonal2)
    if center is None:
        return None  # 对角线平行，无效四边形
    # 3. 返回中心点（四舍五入取整）
    cx, cy = int(round(center[0])), int(round(center[1]))
    return (cx, cy)


#初始化
sensor_id = 2   #因为目前摄像头默认是接到sci2
sensor = None
clock = utime.clock()

# 舵机控制参数
# 在初始化部分添加新的全局变量（在其他全局变量附近）
yellow_zone_counter = 0  # 黄灯区计数器
max_threshold_value = 20  # 阈值最大值
threshold_step = 2  # 阈值调整步长
THRESHOLD_STOP = 10     # 停止阈值(像素)
THRESHOLD_STOP_y = 10
THRESHOLD_STOP_top = 5     # 停止阈值(像素)
THRESHOLD_SLOW = 70    # 慢速跟踪阈值(像素)
FAST_SPEED = 0.59        # 快速跟踪时的速度值
MID_SPEED  = 0.57
SLOW_SPEED = 0.55       # 慢速跟踪时的速度值
x_bias = 0.2             #小车运动时给追踪目标点的横坐标加上的偏移值（之后由msp传递）
#初始化旋转搜索的计数阈值
count_no_rect = 0
threshold_no_rect = 50
#软件PWM初始化
pwm = SoftPWM(freq=25)   #频率:35
pwm.set_duty(0.20)       #占空比
#等效速度=speed*duty
#工作时间=duty/freq,工作时间不能太短，回到导致无法驱动

# 激光状态标志
laser_is_on = False

try:
    """摄像头相关"""
    sensor=Sensor(id=sensor_id,fps=60)
    sensor.reset()
    #Sensor.VGA=640*480 Sensor.FHD=1920*1080，官网有各个尺寸参数
    #通道有3个可选
    sensor.set_framesize(width=640, height=360, chn=CAM_CHN_ID_0)  #设置sensor输出图像尺寸
    #RGB565：16位RGB；RGB888：24位RGB；GRAYSCALE：灰度
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)
    # 使用IDE的帧缓冲区作为显示输出
    Display.init(Display.VIRT, width=640, height=360, to_ide=True)
    # 初始化媒体管理器
    MediaManager.init()
    # 启动传感器
    sensor.run()

    #初始坐标
    rect_x = 320
    rect_y = 180
    target_x = 320
    target_y = 170

    #初始化上层舵机角度
    top_angle = 90
    angle_step = 0

    threshold_rect = (0, 94)# 94暗处不错
    print([threshold_rect])


    def CountAndRotate():
        global count_no_rect, laser_is_on
#         只在旋转时关闭激光
        if laser_is_on:
            laser.value(0)
            laser_is_on = False
        count_no_rect += 1
        if count_no_rect > threshold_no_rect:
            #旋转搜索
            SetTopAngle(90)
            SetBottomSpeed(1.5)
            LED_B_ON()

    #激光锁定计数
    laser_threshold = 2
    laser_count = 0

    while True:
        clock.tick()
        os.exitpoint()
        #LED_G_OFF()
        # 捕获通道0的图像
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        img_rect = img.copy()
        img_rect.binary([threshold_rect])
        rects = img_rect.find_rects(threshold=2000) #2000改

        if rects:
            max_rect = max(rects, key = lambda r:r.magnitude())
            corner = max_rect.corners() #矩形对象的四个角组成的四个元组(x,y)的列表
            #print(corner)
            #print(max_rect.magnitude())

            if not any(c == (-1, -1) for c in corner) and max_rect.magnitude() > 220000:
                # 中心点计算和绘制+判定是否为合法矩形
                center = calculate_center_and_validate(corner)
                if not center:    #表示未正确识别
                    CountAndRotate()
                    LED_G_OFF()
                    LED_R_OFF()#不在慢速区
                    continue
                else:
                    LED_B_OFF()
                    LED_G_ON()
                    count_no_rect = 0

                #确实识别到了矩形
                rect_x, rect_y = center
                #print("中心点坐标: (%d, %d)"%(center[0],center[1]))
                img.draw_cross(center, color=(255,0,0), thickness=2)
                # 绘制矩形
                img.draw_line(corner[0][0], corner[0][1], corner[1][0], corner[1][1], color=(0, 255, 0), thickness=2)
                img.draw_line(corner[1][0], corner[1][1], corner[2][0], corner[2][1], color=(0, 255, 0), thickness=2)
                img.draw_line(corner[2][0], corner[2][1], corner[3][0], corner[3][1], color=(0, 255, 0), thickness=2)
                img.draw_line(corner[3][0], corner[3][1], corner[0][0], corner[0][1], color=(0, 255, 0), thickness=2)


                """由于识别到矩形而进行横向跟踪"""
                # 计算x方向偏差
                error_x = target_x - rect_x
                error_y = target_y - rect_y


                # 根据偏差大小选择下舵机速度
                if abs(error_x) < THRESHOLD_STOP:
                    # 在停止阈值内，停止舵机
                    SetBottomSpeed(0)
                    #img.draw_string(10, 30, "STOP", color=(0,255,0))
                    print("Stop.")

                else:
                    # 确定方向
                    direction = 1 if error_x < 0 else -1
                    # 确定速度

#                    if abs(error_x) < THRESHOLD_SLOW:
#                        if abs(error_x) < THRESHOLD_SLOW/2:
#                            speed = SLOW_SPEED
#                            LED_R_ON()#在黄灯区
#                        else:
#                            LED_R_OFF()
#                            speed = MID_SPEED
#                        if pwm.sample():
#                            SetBottomSpeed(direction * speed)
#                        else:
#                            SetBottomSpeed(0)


                    # 将其修改为：
                    if abs(error_x) < THRESHOLD_SLOW:
                        if abs(error_x) < THRESHOLD_SLOW/2:
                            speed = SLOW_SPEED
                            LED_R_ON()  # 在黄灯区

                            # 增加黄灯区计数
                            yellow_zone_counter += 1

                            # 当计数超过10时，增加阈值
                            if yellow_zone_counter > 10:
                                # 增加阈值，但不超过最大值
                                if THRESHOLD_STOP < max_threshold_value:
                                    THRESHOLD_STOP = min(THRESHOLD_STOP + threshold_step, max_threshold_value)
                                    yellow_zone_counter = 0

                                if THRESHOLD_STOP_y < max_threshold_value:
                                    THRESHOLD_STOP_y = min(THRESHOLD_STOP_y + threshold_step, max_threshold_value)
                                    yellow_zone_counter = 0
                        else:
                            LED_R_OFF()
                            ##重置黄灯区计数器（离开黄灯区时）
                            # yellow_zone_counter = 0
                            speed = MID_SPEED

                        if pwm.sample():
                            SetBottomSpeed(direction * speed)
                        else:
                            SetBottomSpeed(0)

                    else:
                        speed = FAST_SPEED
                        SetBottomSpeed(direction * speed)
                    print(speed)

                # 根据偏差大小选择上舵机
                if abs(error_y) > THRESHOLD_STOP_y:
                    # 确定方向
                    angle_step = 0.4 if error_y > 0 else -0.4
                    top_angle = top_angle + angle_step if top_angle < 92 and top_angle > 88 else 90
                    SetTopAngle(top_angle)

                #垂直水平都在阈值内
                if abs(error_x) < THRESHOLD_STOP and abs(error_y) < THRESHOLD_STOP_y:
                    laser_count += 1
                    # 激光达到阈值后开启，并保持开启状态
                    #if laser_count >= laser_threshold and not laser_is_on:
#                    if laser_count >= laser_threshold:
#                        LED_R_ON()  #计数到达打开红灯，说明激光打开
#                    if not laser_is_on:
                    laser.value(1)
                    laser_is_on = True
                    if abs(error_x) < 10 and abs(error_y) < 10:
                        LED_G_OFF()
                    #LED_R_OFF()#不在慢速区

            else:
                # 没识别到矩形
                CountAndRotate()
                LED_G_OFF()
                LED_R_OFF()#不在慢速区

        else:
            # 没识别到矩形
            CountAndRotate()
            LED_G_OFF()
            LED_R_OFF()#不在慢速区


        # 显示捕获的图像
        #img.draw_string_advanced(50, 100, 32,"你好, 立创·庐山派K230-CanMV开发板!", color=(255, 255, 0), scale=2)
        img.draw_cross((target_x, target_y), color=(0,0,255), thickness = 2)
        img.draw_string_advanced(0,0,28,"fps:%.2f"%clock.fps(),color=(0,0,255)) #显示帧率
        Display.show_image(img)

        #print("fps = ", clock.fps())

        # 检测从MSP发来的消息，解析出偏移值
        read_data = uart.read()  # 尝试读取数据
        if read_data:
            print("Received:", read_data)




except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    time.sleep_ms(1000)
    laser.value(0)
    # 停止传感器运行
    if isinstance(sensor, Sensor):
        sensor.stop()
    pwm0.deinit()
    pwm4.deinit()
    uart.deinit()
    # 反初始化显示模块
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # 释放媒体缓冲区
    MediaManager.deinit()
