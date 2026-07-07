import gc
import time
import os

from machine import FPIOA, Pin, PWM
from media.sensor import *
from media.display import *
from media.media import *


# Direct K230 gimbal control for LCKFB Lushan Pi K230.
#
# Wiring:
#   physical pin 37 -> GPIO32 -> relay IN1
#   physical pin 33 -> GPIO52 -> PWM4 -> lower servo, pan, DS3115MG
#   physical pin 35 -> GPIO42 -> PWM0 -> upper servo, tilt, MG996R
#
# Servo red wire: external 5-6 V. Servo brown/black wire: external GND.
# Servo signal wire: K230 pin 33/35. K230 GND and external GND must be common.

SENSOR_ID = 2
WIDTH = 320
HEIGHT = 240
CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2

# Final laser aiming note:
# The gimbal should ultimately align the detected target center with the
# laser spot in the camera image, not necessarily the image center. The laser
# wire is currently broken, so offset calibration is left for later. After the
# laser is repaired, turn it on briefly, observe its fixed image offset from
# the camera center, and set AIM_OFFSET_X/Y accordingly.
AIM_OFFSET_X = 0
AIM_OFFSET_Y = 0
AIM_X = CENTER_X + AIM_OFFSET_X
AIM_Y = CENTER_Y + AIM_OFFSET_Y

RUN_FRAMES = 0
SERVO_UPDATE_MS = 80
PRINT_INTERVAL_MS = 100
FILTER_OLD = 7
FILTER_NEW = 1
LOST_RESET_FRAMES = 10
STARTUP_HOLD_MS = 300
STABLE_FRAMES_BEFORE_MOVE = 2
MAX_TRACK_ERROR_X = 140
MAX_TRACK_ERROR_Y = 105

ENABLE_IDE_PREVIEW = False
ENABLE_LASER_WHEN_LOCKED = False
ENABLE_PAN_SERVO = False
ENABLE_TILT_SERVO = True
REQUIRE_FRAME_CENTER_FOR_MOVE = True
LOCK_DEADBAND_PX = 10
LOCK_FRAMES_REQUIRED = 12

RELAY_GPIO = 32
RELAY_ACTIVE_LEVEL = 1

PAN_GPIO = 52
PAN_PWM_FUNC = FPIOA.PWM4
PAN_PWM_CH = 4

TILT_GPIO = 42
TILT_PWM_FUNC = FPIOA.PWM0
TILT_PWM_CH = 0

SERVO_FREQ_HZ = 50
SERVO_PERIOD_US = 20000

PAN_CENTER_US = 1500
PAN_MIN_US = 700
PAN_MAX_US = 2300
PAN_DIR = -1
PAN_KP_US_PER_PX = 0.45
PAN_KD_US_PER_PX = 0.05

TILT_CENTER_US = 1500
TILT_MIN_US = 1200
TILT_MAX_US = 1800
TILT_DIR = -1
TILT_KP_US_PER_PX = 0.50
TILT_KD_US_PER_PX = 0.06

MAX_PAN_STEP_US = 2
MAX_TILT_STEP_US = 3
PAN_DEADBAND_PX = 16
TILT_DEADBAND_PX = 12
PAN_ERROR_CONFIRM_FRAMES = 1
TILT_ERROR_CONFIRM_FRAMES = 1
AXIS_NO_IMPROVE_FRAMES = 24
AXIS_IMPROVE_MARGIN_PX = 2

WHITE_THRESHOLDS = (
    (55, 100, -28, 28, -28, 28),
    (45, 100, -20, 20, -20, 20),
    (65, 100, -36, 36, -36, 36),
)
RED_THRESHOLDS = (
    (10, 100, 18, 127, -10, 127),
    (20, 100, 25, 127, -20, 100),
    (0, 80, 12, 127, 0, 127),
)
BLACK_THRESHOLDS = (
    (0, 38, -28, 28, -28, 28),
    (0, 52, -22, 22, -22, 22),
)

MIN_WHITE_PIXELS = 1500
MIN_WHITE_AREA = 2500
MIN_PAPER_W = 45
MIN_PAPER_H = 65
PAPER_RECT_THRESHOLD = 3500
PAPER_ROI_MARGIN = 12
MIN_PAPER_RED_PIXELS = 18
FRAME_RECT_THRESHOLD = 4500
MIN_FRAME_W = 60
MIN_FRAME_H = 80
MIN_FRAME_RED_PIXELS = 18
MIN_BLACK_SIDES_WITH_RED = 2
MIN_BLACK_SIDES_NO_RED = 3

MIN_RED_ANCHOR_PIXELS = 10
MIN_RED_ANCHOR_BLOBS = 3
MIN_RED_ANCHOR_W = 18
MIN_RED_ANCHOR_H = 18

MIN_TARGET_RED_PIXELS = 18
MIN_TARGET_RED_BLOBS = 1
MIN_TARGET_RED_W = 8
MIN_TARGET_RED_H = 8
MAX_RED_CENTER_JUMP_PX = 55
A4_CENTER_OK_PX = 18


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def target_error(cx, cy):
    return cx - AIM_X, cy - AIM_Y


def us_to_duty(pulse_us):
    return pulse_us * 100.0 / SERVO_PERIOD_US


class Servo:
    def __init__(self, pwm_ch, center_us, min_us, max_us):
        self.pwm = PWM(pwm_ch, SERVO_FREQ_HZ)
        self.min_us = min_us
        self.max_us = max_us
        self.us = center_us
        self.write_us(center_us)
        self.pwm.enable(1)

    def write_us(self, pulse_us):
        self.us = clamp(int(pulse_us), self.min_us, self.max_us)
        self.pwm.duty(round(us_to_duty(self.us), 3))

    def add_us(self, delta_us):
        self.write_us(self.us + delta_us)

    def deinit(self):
        try:
            self.pwm.enable(0)
        except Exception:
            pass
        try:
            self.pwm.deinit()
        except Exception:
            pass


class DisabledServo:
    def __init__(self, center_us):
        self.us = center_us

    def write_us(self, pulse_us):
        self.us = int(pulse_us)

    def add_us(self, delta_us):
        pass

    def deinit(self):
        pass


def order_corners(points):
    tl = points[0]
    tr = points[0]
    bl = points[0]
    br = points[0]
    for p in points:
        x, y = p
        if x + y < tl[0] + tl[1]:
            tl = p
        if x + y > br[0] + br[1]:
            br = p
        if x - y > tr[0] - tr[1]:
            tr = p
        if x - y < bl[0] - bl[1]:
            bl = p
    return tl, tr, br, bl


def diagonal_center(corners):
    tl, tr, br, bl = corners
    x1, y1 = tl
    x2, y2 = br
    x3, y3 = tr
    x4, y4 = bl
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if den == 0:
        return (tl[0] + tr[0] + br[0] + bl[0]) // 4, (tl[1] + tr[1] + br[1] + bl[1]) // 4
    a = x1 * y2 - y1 * x2
    b = x3 * y4 - y3 * x4
    px = (a * (x3 - x4) - (x1 - x2) * b) // den
    py = (a * (y3 - y4) - (y1 - y2) * b) // den
    return clamp(px, 0, WIDTH - 1), clamp(py, 0, HEIGHT - 1)


def blob_rect(b):
    return b.x(), b.y(), b.w(), b.h()


def corners_bbox(corners):
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    x1 = min(xs)
    y1 = min(ys)
    x2 = max(xs)
    y2 = max(ys)
    return x1, y1, x2 - x1 + 1, y2 - y1 + 1


def expand_rect(rect, margin):
    x, y, w, h = rect
    x1 = clamp(x - margin, 0, WIDTH - 1)
    y1 = clamp(y - margin, 0, HEIGHT - 1)
    x2 = clamp(x + w - 1 + margin, 0, WIDTH - 1)
    y2 = clamp(y + h - 1 + margin, 0, HEIGHT - 1)
    return x1, y1, x2 - x1 + 1, y2 - y1 + 1


def point_in_rect(x, y, rect):
    rx, ry, rw, rh = rect
    return rx <= x <= rx + rw - 1 and ry <= y <= ry + rh - 1


def rects_intersect(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    return not (
        ax + aw - 1 < bx or
        bx + bw - 1 < ax or
        ay + ah - 1 < by or
        by + bh - 1 < ay
    )


def find_red_blobs(img):
    try:
        return img.find_blobs(
            RED_THRESHOLDS,
            pixels_threshold=1,
            area_threshold=1,
            merge=False,
        )
    except Exception:
        return []


def find_black_blobs(img):
    try:
        return img.find_blobs(
            BLACK_THRESHOLDS,
            pixels_threshold=20,
            area_threshold=20,
            merge=True,
        )
    except Exception:
        return []


def red_pixels_in_rect(red_blobs, rect):
    total = 0
    count = 0
    for b in red_blobs:
        if point_in_rect(b.cx(), b.cy(), rect):
            total += b.pixels()
            count += 1
    return total, count


def red_anchor_stats(red_blobs):
    x1 = WIDTH
    y1 = HEIGHT
    x2 = 0
    y2 = 0
    total = 0
    count = 0
    wx = 0
    wy = 0

    for b in red_blobs:
        if b.pixels() < 1:
            continue
        if b.x() <= 1 or b.y() <= 1:
            continue
        if b.x() + b.w() >= WIDTH - 1 or b.y() + b.h() >= HEIGHT - 1:
            continue
        x1 = min(x1, b.x())
        y1 = min(y1, b.y())
        x2 = max(x2, b.x() + b.w() - 1)
        y2 = max(y2, b.y() + b.h() - 1)
        total += b.pixels()
        count += 1
        wx += b.cx() * b.pixels()
        wy += b.cy() * b.pixels()

    if total < MIN_RED_ANCHOR_PIXELS or count < MIN_RED_ANCHOR_BLOBS:
        return None
    w = x2 - x1 + 1
    h = y2 - y1 + 1
    if w < MIN_RED_ANCHOR_W or h < MIN_RED_ANCHOR_H:
        return None

    return {
        "x": x1,
        "y": y1,
        "w": w,
        "h": h,
        "cx": wx // total,
        "cy": wy // total,
        "total": total,
        "count": count,
    }


def choose_red_target(img):
    red_blobs = find_red_blobs(img)
    x1 = WIDTH
    y1 = HEIGHT
    x2 = 0
    y2 = 0
    total = 0
    count = 0
    wx = 0
    wy = 0

    for b in red_blobs:
        if b.pixels() < 1:
            continue
        # Ignore blobs touching the image edge; partial outer rings bias the
        # center badly when the target is only half in view.
        if b.x() <= 1 or b.y() <= 1:
            continue
        if b.x() + b.w() >= WIDTH - 1 or b.y() + b.h() >= HEIGHT - 1:
            continue
        x1 = min(x1, b.x())
        y1 = min(y1, b.y())
        x2 = max(x2, b.x() + b.w() - 1)
        y2 = max(y2, b.y() + b.h() - 1)
        total += b.pixels()
        count += 1
        wx += b.cx() * b.pixels()
        wy += b.cy() * b.pixels()

    if total < MIN_TARGET_RED_PIXELS or count < MIN_TARGET_RED_BLOBS:
        return None
    w = x2 - x1 + 1
    h = y2 - y1 + 1
    if w < MIN_TARGET_RED_W or h < MIN_TARGET_RED_H:
        return None

    cx = wx // total
    cy = wy // total
    corners = ((x1, y1), (x2, y1), (x2, y2), (x1, y2))
    conf = total * 120 + count * 600 + w * h
    dx, dy = target_error(cx, cy)
    return dx, dy, conf, "RED_TARGET", corners


def black_border_score(black_blobs, rect):
    x, y, w, h = rect
    strips = (
        (x - 8, y - 14, w + 16, 14),
        (x - 8, y + h, w + 16, 14),
        (x - 14, y - 8, 14, h + 16),
        (x + w, y - 8, 14, h + 16),
    )
    total = 0
    sides = 0
    for sx, sy, sw, sh in strips:
        sx = clamp(sx, 0, WIDTH - 1)
        sy = clamp(sy, 0, HEIGHT - 1)
        sw = clamp(sw, 1, WIDTH - sx)
        sh = clamp(sh, 1, HEIGHT - sy)
        side_px = 0
        for b in black_blobs:
            if rects_intersect(blob_rect(b), (sx, sy, sw, sh)):
                side_px += b.pixels()
        if side_px >= 40:
            sides += 1
        total += side_px
    return total, sides


def choose_black_frame(img):
    red_blobs = find_red_blobs(img)
    mask = img.copy()
    try:
        _ = mask.binary(BLACK_THRESHOLDS)
        rects = mask.find_rects(threshold=FRAME_RECT_THRESHOLD)
    except Exception:
        rects = []

    best = None
    best_score = -1
    for r in rects:
        corners = order_corners(r.corners())
        x, y, w, h = corners_bbox(corners)
        if w < MIN_FRAME_W or h < MIN_FRAME_H:
            continue
        if w >= WIDTH - 4 and h >= HEIGHT - 4:
            continue
        aspect = w * 100 // max(1, h)
        if aspect < 30 or aspect > 180:
            continue
        red_px, red_count = red_pixels_in_rect(red_blobs, (x, y, w, h))
        if red_px < MIN_FRAME_RED_PIXELS and red_count < 2:
            continue
        cx, cy = diagonal_center(corners)
        score = r.magnitude() + red_px * 260 + red_count * 900 + (w * h) // 4
        score -= abs(cx - CENTER_X) * 3 + abs(cy - CENTER_Y) * 3
        if score > best_score:
            best = (corners, (x, y, w, h), r.magnitude() + red_px)
            best_score = score
    if best is None:
        black_blobs = find_black_blobs(img)
        for b in black_blobs:
            if b.w() < MIN_FRAME_W or b.h() < MIN_FRAME_H:
                continue
            if b.x() <= 2 or b.y() <= 2:
                continue
            if b.x() + b.w() >= WIDTH - 2 or b.y() + b.h() >= HEIGHT - 2:
                continue
            if b.w() > 170 or b.h() > 230:
                continue
            aspect = b.w() * 100 // max(1, b.h())
            if aspect < 35 or aspect > 90:
                continue
            if abs(b.cx() - CENTER_X) > 105 or abs(b.cy() - CENTER_Y) > 100:
                continue
            x1 = b.x()
            y1 = b.y()
            x2 = b.x() + b.w() - 1
            y2 = b.y() + b.h() - 1
            corners = ((x1, y1), (x2, y1), (x2, y2), (x1, y2))
            score = b.pixels() * 20 + b.w() * b.h()
            score -= abs(b.cx() - CENTER_X) * 6 + abs(b.cy() - CENTER_Y) * 3
            if score > best_score:
                best = (corners, (x1, y1, b.w(), b.h()), score)
                best_score = score
    return best


def choose_white_blob(img):
    try:
        blobs = img.find_blobs(
            WHITE_THRESHOLDS,
            pixels_threshold=80,
            area_threshold=80,
            merge=False,
        )
    except Exception:
        return None

    red_blobs = find_red_blobs(img)
    red_stats = red_anchor_stats(red_blobs)
    black_blobs = find_black_blobs(img)
    best = None
    best_score = -1
    for b in blobs:
        if b.pixels() < MIN_WHITE_PIXELS or b.area() < MIN_WHITE_AREA:
            continue
        if b.w() < MIN_PAPER_W or b.h() < MIN_PAPER_H:
            continue
        aspect = b.w() * 100 // max(1, b.h())
        if aspect < 25 or aspect > 180:
            continue
        if b.w() >= WIDTH - 4 and b.h() >= HEIGHT - 4:
            continue
        black_px, black_sides = black_border_score(black_blobs, blob_rect(b))
        center_bonus = 180 - abs(b.cx() - CENTER_X) - abs(b.cy() - CENTER_Y)

        if red_stats is not None:
            red_w = red_stats["w"]
            red_h = red_stats["h"]
            red_cx = red_stats["cx"]
            red_cy = red_stats["cy"]
            if not point_in_rect(red_cx, red_cy, blob_rect(b)):
                continue
            if b.w() < red_w or b.h() < red_h:
                continue
            if b.w() > red_w * 3 + 60:
                continue
            if b.h() > red_h * 3 + 80:
                continue
            red_px, red_count = red_pixels_in_rect(red_blobs, expand_rect(blob_rect(b), 3))
            if red_px < max(MIN_PAPER_RED_PIXELS, red_stats["total"] // 5) and red_count < max(2, red_stats["count"] // 4):
                continue
            if black_sides < MIN_BLACK_SIDES_WITH_RED:
                continue
            red_center_penalty = abs(b.cx() - red_cx) + abs(b.cy() - red_cy)
            score = red_px * 220 + red_count * 1000 + center_bonus * 5 + b.pixels() // 8
            score += black_px * 30 + black_sides * 1800
            score -= red_center_penalty * 35
        else:
            if black_sides < MIN_BLACK_SIDES_NO_RED:
                continue
            # A4 target is portrait in the camera image. This no-red fallback is
            # intentionally narrower so bright clutter is less likely to move the gimbal.
            if aspect < 45 or aspect > 115:
                continue
            score = center_bonus * 8 + b.pixels() // 6 + black_px * 45 + black_sides * 2600

        if b.area() > 45000:
            score -= (b.area() - 45000) // 2
        if score > best_score:
            best = b
            best_score = score
    return best


def choose_paper_rect(img, paper_blob):
    roi = expand_rect(blob_rect(paper_blob), PAPER_ROI_MARGIN)
    mask = img.copy()
    try:
        _ = mask.binary(WHITE_THRESHOLDS)
    except Exception:
        return None

    try:
        rects = mask.find_rects(roi=roi, threshold=PAPER_RECT_THRESHOLD)
    except Exception:
        try:
            rects = mask.find_rects(threshold=PAPER_RECT_THRESHOLD)
        except Exception:
            rects = []

    best = None
    best_score = -1
    blob_cx = paper_blob.cx()
    blob_cy = paper_blob.cy()
    blob_area = paper_blob.w() * paper_blob.h()
    for r in rects:
        corners = order_corners(r.corners())
        cx, cy = diagonal_center(corners)
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        rx = min(xs)
        ry = min(ys)
        w = max(xs) - min(xs) + 1
        h = max(ys) - min(ys) + 1
        if w < MIN_PAPER_W or h < MIN_PAPER_H:
            continue
        if rx < roi[0] - 2 or ry < roi[1] - 2:
            continue
        if rx + w > roi[0] + roi[2] + 2 or ry + h > roi[1] + roi[3] + 2:
            continue
        aspect = w * 100 // max(1, h)
        if aspect < 35 or aspect > 150:
            continue
        area = w * h
        area_delta = abs(area - blob_area)
        if area * 100 < blob_area * 55:
            continue
        if area * 100 > blob_area * 190:
            continue
        score = r.magnitude() + area // 2 - area_delta // 3
        score -= abs(cx - blob_cx) * 20 + abs(cy - blob_cy) * 20
        if score > best_score:
            best = corners
            best_score = score
    return best


def bbox_corners(b):
    x1 = b.x()
    y1 = b.y()
    x2 = b.x() + b.w() - 1
    y2 = b.y() + b.h() - 1
    return (x1, y1), (x2, y1), (x2, y2), (x1, y2)


def detect_target(img):
    frame = choose_black_frame(img)
    if frame is not None:
        corners, frame_roi, conf = frame
        cx, cy = diagonal_center(corners)
        dx, dy = target_error(cx, cy)
        return dx, dy, conf, "FRAME_CENTER", corners

    paper = choose_white_blob(img)
    if paper is not None:
        corners = choose_paper_rect(img, paper)
        label = "PAPER_CENTER"
        if corners is None:
            corners = bbox_corners(paper)
            label = "PAPER_BBOX"
        cx, cy = diagonal_center(corners)
        dx, dy = target_error(cx, cy)
        return dx, dy, paper.pixels(), label, corners

    red_target = choose_red_target(img)
    if red_target is not None:
        dx, dy, conf, label, corners = red_target
        tx = CENTER_X + dx
        ty = CENTER_Y + dy
        dx, dy = target_error(tx, ty)
        return dx, dy, conf, "RED_FALLBACK", corners

    return None


def filtered_value(old, new):
    if old is None:
        return new
    return (old * FILTER_OLD + new * FILTER_NEW) // (FILTER_OLD + FILTER_NEW)


def error_to_step(err_px, last_err_px, direction, kp, kd, max_step, deadband_px):
    if abs(err_px) <= deadband_px:
        return 0
    d_err = err_px - last_err_px
    step = direction * (kp * err_px + kd * d_err)
    return clamp(step, -max_step, max_step)


def is_safe_track_error(dx, dy):
    return abs(dx) <= MAX_TRACK_ERROR_X and abs(dy) <= MAX_TRACK_ERROR_Y


def update_error_confirm(err_px, deadband_px, last_sign, count):
    if abs(err_px) <= deadband_px:
        return 0, 0, False
    sign = 1 if err_px > 0 else -1
    if sign == last_sign:
        count += 1
    else:
        count = 1
    return sign, count, True


def update_axis_guard(err_px, deadband_px, best_abs, no_improve_count):
    err_abs = abs(err_px)
    if err_abs <= deadband_px:
        return err_abs, 0, True
    if err_abs + AXIS_IMPROVE_MARGIN_PX < best_abs:
        return err_abs, 0, True
    no_improve_count += 1
    return best_abs, no_improve_count, no_improve_count < AXIS_NO_IMPROVE_FRAMES


def draw_debug(img, result, pan_us, tilt_us):
    img.draw_cross(AIM_X, AIM_Y, color=(255, 255, 0), size=14)
    img.draw_circle(AIM_X, AIM_Y, 5, color=(255, 255, 0))
    if AIM_X != CENTER_X or AIM_Y != CENTER_Y:
        img.draw_cross(CENTER_X, CENTER_Y, color=(128, 128, 128), size=8)
    if result is None:
        img.draw_string_advanced(0, 0, 12, "NO TARGET", color=(255, 0, 0))
        return
    dx, dy, conf, label, corners = result
    tx = AIM_X + dx
    ty = AIM_Y + dy
    img.draw_cross(tx, ty, color=(0, 255, 0), size=16)
    img.draw_circle(tx, ty, 6, color=(0, 255, 0), thickness=2)
    img.draw_line(tx, ty, AIM_X, AIM_Y, color=(0, 0, 255), thickness=2)
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % 4]
        img.draw_line(x1, y1, x2, y2, color=(0, 255, 0), thickness=2)
    img.draw_string_advanced(0, 0, 10, "%s dx=%d dy=%d" % (label, dx, dy), color=(255, 0, 0))
    img.draw_string_advanced(0, 14, 10, "pan=%d tilt=%d" % (pan_us, tilt_us), color=(255, 0, 0))


def main():
    print("K230_GIMBAL_DIRECT_START")
    print("PAN GPIO52/PIN33 PWM4, TILT GPIO42/PIN35 PWM0, RELAY GPIO32/PIN37")
    print("IDE_PREVIEW=%d, FRAME=%dx%d" % (1 if ENABLE_IDE_PREVIEW else 0, WIDTH, HEIGHT))
    print("SERVO_ENABLE pan=%d tilt=%d" % (1 if ENABLE_PAN_SERVO else 0, 1 if ENABLE_TILT_SERVO else 0))

    sensor = None
    pan = None
    tilt = None
    relay = None

    filter_x = None
    filter_y = None
    lost_frames = 0
    frame_id = 0
    last_servo_ms = time.ticks_ms()
    last_print_ms = time.ticks_ms()
    last_dx = 0
    last_dy = 0
    lock_count = 0
    stable_count = 0
    move_enabled = 0
    pan_err_sign = 0
    pan_err_count = 0
    tilt_err_sign = 0
    tilt_err_count = 0
    pan_best_abs = MAX_TRACK_ERROR_X + 1
    tilt_best_abs = MAX_TRACK_ERROR_Y + 1
    pan_no_improve = 0
    tilt_no_improve = 0
    start_ms = time.ticks_ms()

    try:
        try:
            MediaManager.deinit()
            time.sleep_ms(200)
        except Exception:
            pass

        fpioa = FPIOA()
        fpioa.set_function(RELAY_GPIO, FPIOA.GPIO32)
        if ENABLE_PAN_SERVO:
            fpioa.set_function(PAN_GPIO, PAN_PWM_FUNC)
        if ENABLE_TILT_SERVO:
            fpioa.set_function(TILT_GPIO, TILT_PWM_FUNC)

        relay = Pin(RELAY_GPIO, Pin.OUT, value=1 - RELAY_ACTIVE_LEVEL)
        if ENABLE_PAN_SERVO:
            pan = Servo(PAN_PWM_CH, PAN_CENTER_US, PAN_MIN_US, PAN_MAX_US)
        else:
            pan = DisabledServo(PAN_CENTER_US)
        if ENABLE_TILT_SERVO:
            tilt = Servo(TILT_PWM_CH, TILT_CENTER_US, TILT_MIN_US, TILT_MAX_US)
        else:
            tilt = DisabledServo(TILT_CENTER_US)

        sensor = Sensor(id=SENSOR_ID, fps=30)
        sensor.reset()
        sensor.set_framesize(width=WIDTH, height=HEIGHT, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
        if ENABLE_IDE_PREVIEW:
            Display.init(Display.VIRT, width=WIDTH, height=HEIGHT, to_ide=True)
        MediaManager.init()
        sensor.run()
        time.sleep_ms(600)

        while RUN_FRAMES == 0 or frame_id < RUN_FRAMES:
            os.exitpoint()
            frame_id += 1
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            raw_result = detect_target(img)

            if raw_result is None:
                lost_frames += 1
                lock_count = 0
                stable_count = 0
                move_enabled = 0
                pan_err_sign = 0
                pan_err_count = 0
                tilt_err_sign = 0
                tilt_err_count = 0
                pan_best_abs = MAX_TRACK_ERROR_X + 1
                tilt_best_abs = MAX_TRACK_ERROR_Y + 1
                pan_no_improve = 0
                tilt_no_improve = 0
                if lost_frames >= LOST_RESET_FRAMES:
                    filter_x = None
                    filter_y = None
                result = None
                if relay is not None:
                    relay.value(1 - RELAY_ACTIVE_LEVEL)
            else:
                dx, dy, conf, label, corners = raw_result
                lost_frames = 0
                filter_x = filtered_value(filter_x, dx)
                filter_y = filtered_value(filter_y, dy)
                result = (filter_x, filter_y, conf, label, corners)
                if is_safe_track_error(filter_x, filter_y):
                    stable_count += 1
                else:
                    stable_count = 0

                now = time.ticks_ms()
                if time.ticks_diff(now, last_servo_ms) >= SERVO_UPDATE_MS:
                    last_servo_ms = now
                    can_move = (
                        stable_count >= STABLE_FRAMES_BEFORE_MOVE and
                        time.ticks_diff(now, start_ms) >= STARTUP_HOLD_MS and
                        (not REQUIRE_FRAME_CENTER_FOR_MOVE or result[3] == "FRAME_CENTER")
                    )
                    move_enabled = 1 if can_move else 0
                    if can_move:
                        pan_err_sign, pan_err_count, pan_over = update_error_confirm(
                            filter_x, PAN_DEADBAND_PX, pan_err_sign, pan_err_count
                        )
                        tilt_err_sign, tilt_err_count, tilt_over = update_error_confirm(
                            filter_y, TILT_DEADBAND_PX, tilt_err_sign, tilt_err_count
                        )
                        if not pan_over or pan_err_count == 1:
                            pan_best_abs = MAX_TRACK_ERROR_X + 1
                            pan_no_improve = 0
                        if not tilt_over or tilt_err_count == 1:
                            tilt_best_abs = MAX_TRACK_ERROR_Y + 1
                            tilt_no_improve = 0
                        pan_step = 0
                        tilt_step = 0
                        if pan_over and pan_err_count >= PAN_ERROR_CONFIRM_FRAMES:
                            pan_best_abs, pan_no_improve, pan_allowed = update_axis_guard(
                                filter_x, PAN_DEADBAND_PX, pan_best_abs, pan_no_improve
                            )
                            if pan_allowed:
                                pan_step = error_to_step(
                                    filter_x, last_dx, PAN_DIR, PAN_KP_US_PER_PX,
                                    PAN_KD_US_PER_PX, MAX_PAN_STEP_US, PAN_DEADBAND_PX
                                )
                        if tilt_over and tilt_err_count >= TILT_ERROR_CONFIRM_FRAMES:
                            tilt_best_abs, tilt_no_improve, tilt_allowed = update_axis_guard(
                                filter_y, TILT_DEADBAND_PX, tilt_best_abs, tilt_no_improve
                            )
                            if tilt_allowed:
                                tilt_step = error_to_step(
                                    filter_y, last_dy, TILT_DIR, TILT_KP_US_PER_PX,
                                    TILT_KD_US_PER_PX, MAX_TILT_STEP_US, TILT_DEADBAND_PX
                                )
                        pan.add_us(pan_step)
                        tilt.add_us(tilt_step)
                    last_dx = filter_x
                    last_dy = filter_y

                    if can_move and abs(filter_x) <= LOCK_DEADBAND_PX and abs(filter_y) <= LOCK_DEADBAND_PX:
                        lock_count += 1
                    else:
                        lock_count = 0

                    if ENABLE_LASER_WHEN_LOCKED and lock_count >= LOCK_FRAMES_REQUIRED:
                        relay.value(RELAY_ACTIVE_LEVEL)
                    else:
                        relay.value(1 - RELAY_ACTIVE_LEVEL)

            now = time.ticks_ms()
            if time.ticks_diff(now, last_print_ms) >= PRINT_INTERVAL_MS:
                last_print_ms = now
                if result is None:
                    print("@G,0,0,0,%d,%d,NONE" % (pan.us, tilt.us))
                else:
                    dx, dy, conf, label, _ = result
                    print(
                        "@G,1,%d,%d,%d,%d,%s,stable=%d,move=%d,pc=%d,tc=%d,pn=%d,tn=%d" %
                        (
                            dx, dy, pan.us, tilt.us, label, stable_count, move_enabled,
                            pan_err_count, tilt_err_count, pan_no_improve, tilt_no_improve
                        )
                    )

            if ENABLE_IDE_PREVIEW:
                draw_debug(img, result, pan.us, tilt.us)
                Display.show_image(img)

            del img
            if frame_id % 30 == 0:
                gc.collect()
            time.sleep_ms(5)

    except KeyboardInterrupt:
        print("K230_GIMBAL_DIRECT_STOP_BY_USER")
    except Exception as e:
        print("K230_GIMBAL_DIRECT_EXCEPTION", type(e).__name__, e)
    finally:
        try:
            if relay is not None:
                relay.value(1 - RELAY_ACTIVE_LEVEL)
        except Exception:
            pass
        try:
            if sensor is not None:
                sensor.stop()
        except Exception as e:
            print("sensor stop err", e)
        if ENABLE_IDE_PREVIEW:
            try:
                Display.deinit()
            except Exception:
                pass
        try:
            MediaManager.deinit()
        except Exception as e:
            print("media deinit err", e)
        try:
            if pan is not None:
                pan.deinit()
        except Exception:
            pass
        try:
            if tilt is not None:
                tilt.deinit()
        except Exception:
            pass
        gc.collect()
        print("K230_GIMBAL_DIRECT_DONE")


main()
