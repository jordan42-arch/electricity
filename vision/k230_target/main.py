import gc
import time

from machine import FPIOA, UART
from media.sensor import *
from media.media import *


SENSOR_ID = 2
WIDTH = 320
HEIGHT = 240
CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2

RUN_FRAMES = 0  # 0 means competition continuous mode.
SEND_INTERVAL_MS = 50
FILTER_OLD = 3
FILTER_NEW = 2
LOST_RESET_FRAMES = 10

UART_BAUD = 115200
UART_TX_PIN = 11
UART_RX_PIN = 12

WHITE_THRESHOLDS = ((110, 255), (100, 255), (120, 255), (130, 255))
MIN_WHITE_PIXELS = 2500
MIN_WHITE_AREA = 6000

DARK_THRESH = 94
RECT_THRESHOLD = 3000
MIN_RECT_AREA = 1200
MIN_RECT_MAG = 150000

CIRCLE_THRESHOLD = 2000
CIRCLE_MIN_CONF = 2200
CIRCLE_R_MIN = 8
CIRCLE_R_MAX = 80


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def poly_area(points):
    area2 = 0
    n = len(points)
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        area2 += x1 * y2 - x2 * y1
    if area2 < 0:
        area2 = -area2
    return area2 // 2


def rect_center(points):
    sx = 0
    sy = 0
    for x, y in points:
        sx += x
        sy += y
    return sx // len(points), sy // len(points)


def rect_bounds(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), min(ys), max(xs), max(ys)


def rect_roi(points, margin):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x1 = clamp(min(xs) - margin, 0, WIDTH - 1)
    y1 = clamp(min(ys) - margin, 0, HEIGHT - 1)
    x2 = clamp(max(xs) + margin, 0, WIDTH - 1)
    y2 = clamp(max(ys) + margin, 0, HEIGHT - 1)
    return x1, y1, x2 - x1 + 1, y2 - y1 + 1


def choose_white_target(blobs):
    best = None
    best_score = -1
    for b in blobs:
        if b.pixels() < MIN_WHITE_PIXELS or b.area() < MIN_WHITE_AREA:
            continue
        if b.w() < 45 or b.h() < 100:
            continue
        if b.h() * 100 < b.w() * 120:
            continue
        if b.x() <= 35 or b.y() <= 8:
            continue
        if b.x() + b.w() >= WIDTH - 35 or b.y() + b.h() >= HEIGHT - 4:
            continue
        shape_bonus = b.h() * 100 // max(1, b.w())
        score = b.pixels() + b.area() // 4 + shape_bonus
        if score > best_score:
            best = b
            best_score = score
    return best


def choose_rect(rects):
    best = None
    best_score = -1
    for r in rects:
        points = r.corners()
        area = poly_area(points)
        x1, y1, x2, y2 = rect_bounds(points)
        rw = x2 - x1
        rh = y2 - y1
        if area < MIN_RECT_AREA or r.magnitude() < MIN_RECT_MAG:
            continue
        if rw < 35 or rh < 50:
            continue
        score = r.magnitude() + area
        if score > best_score:
            best = r
            best_score = score
    return best


def choose_circle_center(circles, corners):
    x1, y1, x2, y2 = rect_bounds(corners)
    rh = max(1, y2 - y1)
    rw = max(1, x2 - x1)
    filtered = []
    for c in circles:
        rel_y = (c.y() - y1) * 100 // rh
        rel_x = (c.x() - x1) * 100 // rw
        if 10 <= rel_y <= 50 and 10 <= rel_x <= 90:
            filtered.append(c)
    if not filtered:
        return None

    weight_sum = 0
    x_sum = 0
    y_sum = 0
    conf = 0
    for c in filtered:
        weight = max(1, c.magnitude())
        weight_sum += weight
        x_sum += c.x() * weight
        y_sum += c.y() * weight
        conf += c.magnitude()
    return x_sum // weight_sum, y_sum // weight_sum, conf // len(filtered)


def init_uart():
    try:
        fpioa = FPIOA()
        fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
        fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
        return UART(
            UART.UART2,
            baudrate=UART_BAUD,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )
    except Exception as e:
        print("UART_INIT_FAIL", type(e).__name__, e)
        return None


def detect_target(img):
    try:
        for white_thresh in WHITE_THRESHOLDS:
            white_blobs = img.find_blobs(
                [white_thresh],
                pixels_threshold=40,
                area_threshold=40,
                merge=True,
            )
            target_blob = choose_white_target(white_blobs)
            if target_blob is not None:
                return (
                    target_blob.cx() - CENTER_X,
                    target_blob.cy() - CENTER_Y,
                    target_blob.pixels(),
                    "WHITE_TARGET",
                )
    except Exception:
        pass

    bin_img = img.copy()
    _ = bin_img.binary([(0, DARK_THRESH)])
    rect = choose_rect(bin_img.find_rects(threshold=RECT_THRESHOLD))
    if rect is None:
        return None

    corners = rect.corners()
    roi = rect_roi(corners, 8)
    circles = img.find_circles(
        roi=roi,
        threshold=CIRCLE_THRESHOLD,
        x_margin=10,
        y_margin=10,
        r_margin=10,
        r_min=CIRCLE_R_MIN,
        r_max=CIRCLE_R_MAX,
        r_step=2,
    )
    if not circles:
        return None

    center = choose_circle_center(circles, corners)
    if center is None:
        return None

    cx, cy, conf = center
    if conf < CIRCLE_MIN_CONF:
        return None
    return cx - CENTER_X, cy - CENTER_Y, conf, "CIRCLE_UPPER"


sensor = None
uart = None
frame_id = 0
filter_x = None
filter_y = None
lost_frames = 0
last_send_ms = time.ticks_ms()

try:
    print("VISION_UART_START")
    uart = init_uart()
    sensor = Sensor(id=SENSOR_ID, fps=30)
    sensor.reset()
    sensor.set_framesize(width=WIDTH, height=HEIGHT, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)
    MediaManager.init()
    sensor.run()
    time.sleep_ms(600)

    while RUN_FRAMES == 0 or frame_id < RUN_FRAMES:
        frame_id += 1
        result = detect_target(sensor.snapshot(chn=CAM_CHN_ID_0))
        if result is None:
            lost_frames += 1
            if lost_frames >= LOST_RESET_FRAMES:
                filter_x = None
                filter_y = None
            valid, out_x, out_y, conf, label = 0, 0, 0, 0, "NONE"
        else:
            dx, dy, conf, label = result
            lost_frames = 0
            if filter_x is None:
                filter_x = dx
                filter_y = dy
            else:
                filter_x = (filter_x * FILTER_OLD + dx * FILTER_NEW) // (FILTER_OLD + FILTER_NEW)
                filter_y = (filter_y * FILTER_OLD + dy * FILTER_NEW) // (FILTER_OLD + FILTER_NEW)
            valid, out_x, out_y = 1, filter_x, filter_y

        now = time.ticks_ms()
        if time.ticks_diff(now, last_send_ms) >= SEND_INTERVAL_MS:
            msg = "@V,%d,%d,%d,%d,%s\n" % (valid, out_x, out_y, conf, label)
            print(msg, end="")
            if uart is not None:
                try:
                    _ = uart.write(msg.encode())
                except Exception as e:
                    print("UART_WRITE_FAIL", type(e).__name__, e)
                    uart = None
            last_send_ms = now

        time.sleep_ms(5)

except KeyboardInterrupt:
    print("VISION_UART_STOP_BY_USER")
except Exception as e:
    print("VISION_UART_EXCEPTION", type(e).__name__, e)
finally:
    try:
        if sensor is not None:
            sensor.stop()
    except Exception as e:
        print("sensor stop err", e)
    try:
        MediaManager.deinit()
    except Exception as e:
        print("media deinit err", e)
    try:
        if uart is not None:
            uart.deinit()
    except Exception as e:
        print("uart deinit err", e)
    gc.collect()
    print("VISION_UART_DONE")
