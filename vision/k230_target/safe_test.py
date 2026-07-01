import gc
import time

from media.sensor import *
from media.media import *


SENSOR_ID = 2
WIDTH = 320
HEIGHT = 240
CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2

DARK_THRESH = 94
RECT_THRESHOLD = 3000
MIN_RECT_AREA = 1200
MIN_RECT_MAG = 150000

CIRCLE_THRESHOLD = 2000
CIRCLE_R_MIN = 8
CIRCLE_R_MAX = 80

RUN_FRAMES = 80  # 0 means run until Ctrl-C.
PRINT_ALL_CIRCLES = False

WHITE_THRESH = (110, 255)
MIN_WHITE_PIXELS = 2500
MIN_WHITE_AREA = 6000


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


def rect_roi(points, margin):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x1 = clamp(min(xs) - margin, 0, WIDTH - 1)
    y1 = clamp(min(ys) - margin, 0, HEIGHT - 1)
    x2 = clamp(max(xs) + margin, 0, WIDTH - 1)
    y2 = clamp(max(ys) + margin, 0, HEIGHT - 1)
    return x1, y1, x2 - x1 + 1, y2 - y1 + 1


def rect_bounds(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), min(ys), max(xs), max(ys)


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
        # The target paper/tablet area is a large bright vertical blob.
        shape_bonus = b.h() * 100 // max(1, b.w())
        score = b.pixels() + b.area() // 4 + shape_bonus
        if score > best_score:
            best = b
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

    return x_sum // weight_sum, y_sum // weight_sum, conf // len(filtered), len(filtered)


def print_result(valid, dx=0, dy=0, conf=0, label="NONE"):
    print("@V,%d,%d,%d,%d,%s" % (valid, dx, dy, conf, label))


sensor = None
frame_id = 0

try:
    print("VISION_SAFE_START")
    sensor = Sensor(id=SENSOR_ID, fps=30)
    sensor.reset()
    sensor.set_framesize(width=WIDTH, height=HEIGHT, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)
    MediaManager.init()
    sensor.run()
    time.sleep_ms(600)

    while RUN_FRAMES == 0 or frame_id < RUN_FRAMES:
        frame_id += 1
        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        try:
            white_blobs = img.find_blobs(
                [WHITE_THRESH],
                pixels_threshold=40,
                area_threshold=40,
                merge=True,
            )
            target_blob = choose_white_target(white_blobs)
            if target_blob is not None:
                dx = target_blob.cx() - CENTER_X
                dy = target_blob.cy() - CENTER_Y
                print_result(1, dx, dy, target_blob.pixels(), "WHITE_TARGET")
                time.sleep_ms(30)
                continue
        except Exception as e:
            pass

        bin_img = img.copy()
        _ = bin_img.binary([(0, DARK_THRESH)])
        rect = choose_rect(bin_img.find_rects(threshold=RECT_THRESHOLD))
        if rect is None:
            print_result(0)
            time.sleep_ms(30)
            continue

        corners = rect.corners()
        rcx, rcy = rect_center(corners)
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

        if circles:
            center = choose_circle_center(circles, corners)
            if center is None:
                print_result(0)
                time.sleep_ms(30)
                continue
            cx, cy, conf, count = center
            dx = cx - CENTER_X
            dy = cy - CENTER_Y
            print_result(1, dx, dy, conf, "CIRCLE_UPPER")
            if PRINT_ALL_CIRCLES:
                for c in circles[:6]:
                    print("C x=%d y=%d r=%d mag=%d dx=%d dy=%d" %
                          (c.x(), c.y(), c.r(), c.magnitude(),
                           c.x() - CENTER_X, c.y() - CENTER_Y))
        else:
            print_result(0)

        time.sleep_ms(30)

except KeyboardInterrupt:
    print("VISION_SAFE_STOP_BY_USER")
except Exception as e:
    print("VISION_SAFE_EXCEPTION", type(e).__name__, e)
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
    gc.collect()
    print("VISION_SAFE_DONE")
