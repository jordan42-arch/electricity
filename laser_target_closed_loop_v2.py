"""
K230 / CanMV vision-only target-lock verifier for the 2025 NUEDC E target.

This program starts from the proven laser_spot_calibration_monitor.py and
closes the loop: pulse -> measure actual spot -> move gimbal -> settle ->
measure again. It does not use the image centre as an aiming substitute.

Run it from CanMV IDE and watch the virtual preview:
  SEARCH   : no credible target
  ACQUIRE  : same target has appeared for several consecutive frames
  LOCK     : stable target centre (the green cross is the value to use later)
  HOLD     : a few missed frames; retain the last lock rather than jump to
             background clutter
"""

import gc
import os
import time

from machine import FPIOA, Pin, PWM
from media.sensor import *
from media.display import *
from media.media import *
import cv_lite


SENSOR_ID = 2
WIDTH = 320
HEIGHT = 240
SHAPE = [HEIGHT, WIDTH]
CAMERA_FPS = 90
CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2

# Detection thresholds.  They use the same RGB565 Lab threshold convention as
# the previously-tested gimbal program, but detection is intentionally stricter.
BLACK_THRESHOLDS = (
    (0, 38, -28, 28, -28, 28),
    (0, 52, -22, 22, -22, 22),
)
RED_THRESHOLDS = (
    (10, 100, 18, 127, -10, 127),
    (20, 100, 25, 127, -20, 100),
    (0, 80, 12, 127, 0, 127),
)

# The target is an A4 portrait sheet.  These bounds intentionally allow normal
# perspective distortion, but reject thin black objects and most background
# rectangles.  Tune only after looking at the on-screen candidate rectangle.
RECT_THRESHOLD = 4500
MIN_FRAME_W = 60
MIN_FRAME_H = 80
MIN_FRAME_AREA = 6500
MAX_FRAME_AREA = 62000
ASPECT_MIN_X100 = 35
ASPECT_MAX_X100 = 135
EDGE_MARGIN = 8
MIN_BLACK_SIDES = 3
MIN_RED_PIXELS = 18
MIN_RED_BLOBS = 2

# Temporal association is the part that stops a brief background false
# positive from stealing the lock.  A candidate needs this many adjacent frames
# before becoming LOCK.  Once locked, a wrong/far-away candidate is rejected;
# short detection holes become HOLD instead of a new target.
ACQUIRE_FRAMES = 3
HOLD_FRAMES = 3
MAX_ACQUIRE_JUMP_PX = 26
MIN_LOCK_GATE_PX = 30
LOCK_GATE_FRACTION_X100 = 18
FILTER_OLD = 4
FILTER_NEW = 1
PRINT_INTERVAL_MS = 120

# Wiring used by the current gimbal:
#   lower servo / pan  -> PIN33 / GPIO52 / PWM4
#   upper servo / tilt -> PIN35 / GPIO42 / PWM0
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
# Use duty_u16 for servo pulses. Axis response was measured with this API:
#   PAN 1650us/180ms -> target cx +23, PAN 1350us/180ms -> target cx -29
#   TILT 1550us -> target cy +33, TILT 1450us -> target cy -61
PWM_DUTY_U16_SCALE = 65535
PAN_CENTER_US = 1500
PAN_MIN_US = 1350
PAN_MAX_US = 1650
TILT_CENTER_US = 1500
TILT_MIN_US = 1250
TILT_MAX_US = 1750
# DS3115 is continuous rotation: pulse away from 1500 us, then stop at 1500.
PAN_STOP_US = 1500
PAN_PLUS_US = 1650
PAN_MINUS_US = 1350
PAN_PULSE_MS = 90
ENABLE_FINAL_FIRE = False
TILT_DIR = -1

# One loop consists of an OFF/ON spot measurement and, if necessary, a tiny
# gimbal correction. P-only incremental control avoids integral wind-up and
# hunting on hobby servos.
CAL_PULSE_ON_MS = 120
SETTLE_AFTER_MOVE_MS = 35
MISS_RETRY_MS = 25
LOOP_PERIOD_MS = 320
MAX_CONTROL_LOOPS = 200
ALIGN_X_PX = 6
ALIGN_Y_PX = 7
ALIGN_CONFIRM_PULSES = 3
FINAL_FIRE_MS = 700
TILT_KP_US_PER_PX = 0.28
TILT_MAX_STEP_US = 18
MIN_SERVO_STEP_US = 8
NO_IMPROVE_LIMIT = 4
IMPROVE_MARGIN_PX = 2
MAX_SAFE_ERROR_X = 120
MAX_SAFE_ERROR_Y = 95

# Detect the spot by comparing a camera frame taken with the laser OFF to
# frames taken immediately after it turns ON. This is deliberately colour
# agnostic: on UV paper a violet laser can look purple, blue, white, or nearly
# colourless to the camera depending on exposure. The red rings are present in
# both frames, so they disappear from this difference image.
# Spot detection is based on laser-ON minus laser-OFF frames.
#
# Important: the laser is allowed to hit the black tape frame while the loop is
# still correcting.  The old detector only accepted blobs well inside the target
# rectangle, so a real spot on the black frame was rejected as "no_spot".  Keep
# the strict target-centre detector unchanged, but make the spot detector more
# tolerant near the frame edge:
#   1. search a small padded ROI around the detected target;
#   2. allow candidates on the border instead of forcing an inner margin;
#   3. try lower thresholds when the black tape reflects weakly;
#   4. keep size/compactness gates so background reflections are still rejected.
DELTA_THRESHOLD = 24
DELTA_THRESHOLDS = (24, 16, 10)
DELTA_MIN_PIXELS = 2
DELTA_MAX_PIXELS = 220
DELTA_MAX_SIDE = 28
DELTA_EDGE_MAX_PIXELS = 320
DELTA_EDGE_MAX_SIDE = 36
DELTA_ROI_PAD = 8
DELTA_SETTLE_MS = 40
DELTA_CONTINUITY_GATE_PX = 28
STALE_SPOT_GATE_PX = 4
STALE_MOVE_MIN_US = 14
STALE_SPOT_LIMIT = 2


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def us_to_duty(pulse_us):
    return int((int(pulse_us) * PWM_DUTY_U16_SCALE + SERVO_PERIOD_US // 2) // SERVO_PERIOD_US)


def pwm_write_compat(pwm, duty_u16):
    try:
        pwm.duty_u16(int(duty_u16))
        return
    except Exception as e:
        print("PWM duty_u16 fallback:", type(e).__name__, e)
    try:
        pwm.duty(int(duty_u16) * 100 // PWM_DUTY_U16_SCALE)
    except Exception as e:
        print("PWM duty fallback err", type(e).__name__, e)


def pwm_enable_compat(pwm, enabled):
    if enabled:
        # MicroPython PWM is active after freq/duty are set; enable() is unsupported.
        return
    try:
        pwm_write_compat(pwm, 0)
    except Exception:
        pass


def pwm_create_compat(pwm_ch, freq_hz, duty_percent):
    pwm = PWM(pwm_ch)
    try:
        pwm.freq(freq_hz)
    except Exception as e:
        print("PWM freq set err", type(e).__name__, e)
    try:
        pwm_write_compat(pwm, duty_percent)
    except Exception as e:
        print("PWM duty init err", type(e).__name__, e)
    return pwm


class Servo:
    def __init__(self, pwm_ch, center_us, min_us, max_us):
        self.pwm = pwm_create_compat(pwm_ch, SERVO_FREQ_HZ, us_to_duty(center_us))
        self.min_us = min_us
        self.max_us = max_us
        self.us = center_us
        self.duty = None
        self.write_us(center_us)
        pwm_enable_compat(self.pwm, True)

    def write_us(self, pulse_us):
        self.us = clamp(int(pulse_us), self.min_us, self.max_us)
        self.duty = us_to_duty(self.us)
        pwm_write_compat(self.pwm, self.duty)

    def add_us(self, delta_us):
        before_duty = self.duty
        before_us = self.us
        self.write_us(self.us + delta_us)
        moved = self.duty != before_duty
        print("@TILT_SET,from_us=%d,to_us=%d,from_duty=%d,to_duty=%d,moved=%d" %
              (before_us, self.us, before_duty, self.duty, 1 if moved else 0))
        return moved

    def deinit(self):
        pwm_enable_compat(self.pwm, False)
        try:
            self.pwm.deinit()
        except Exception:
            pass


class ProgressGuard:
    """Stops an axis after repeated non-improving measured laser errors."""
    def __init__(self):
        self.best_abs = None
        self.no_improve = 0
        self.blocked = False

    def reset(self):
        self.best_abs = None
        self.no_improve = 0
        self.blocked = False

    def allow(self, error, tolerance):
        if abs(error) <= tolerance:
            self.reset()
            return False
        self.best_abs = abs(error)
        self.no_improve = 0
        self.blocked = False
        return True


def p_step(error, direction, kp, max_step):
    step = int(round(kp * error))
    if step == 0 and error != 0:
        step = 1 if error > 0 else -1
    if abs(error) > 0 and abs(step) < MIN_SERVO_STEP_US:
        step = MIN_SERVO_STEP_US if step > 0 else -MIN_SERVO_STEP_US
    return clamp(direction * step, -max_step, max_step)


def order_corners(points):
    # Make the order independent of find_rects() corner order.
    tl = min(points, key=lambda p: p[0] + p[1])
    br = max(points, key=lambda p: p[0] + p[1])
    tr = max(points, key=lambda p: p[0] - p[1])
    bl = min(points, key=lambda p: p[0] - p[1])
    return (tl, tr, br, bl)


def diagonal_center(corners):
    # True projective centre: intersection of the two diagonals, not a simple
    # diagonal midpoint.  This makes the centre robust to target tilt.
    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = (
        corners[0], corners[2], corners[1], corners[3]
    )
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if den == 0:
        return (x1 + x2) // 2, (y1 + y2) // 2
    p1 = x1 * y2 - y1 * x2
    p2 = x3 * y4 - y3 * x4
    px = (p1 * (x3 - x4) - (x1 - x2) * p2) / den
    py = (p1 * (y3 - y4) - (y1 - y2) * p2) / den
    return int(round(px)), int(round(py))


def corners_bbox(corners):
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    x = min(xs)
    y = min(ys)
    return x, y, max(xs) - x + 1, max(ys) - y + 1


def expand_rect(rect, pad):
    x, y, w, h = rect
    nx = clamp(x - pad, 0, WIDTH - 1)
    ny = clamp(y - pad, 0, HEIGHT - 1)
    x2 = clamp(x + w - 1 + pad, 0, WIDTH - 1)
    y2 = clamp(y + h - 1 + pad, 0, HEIGHT - 1)
    return nx, ny, x2 - nx + 1, y2 - ny + 1


def point_in_rect(x, y, rect, margin=0):
    rx, ry, rw, rh = rect
    return (rx + margin <= x <= rx + rw - 1 - margin and
            ry + margin <= y <= ry + rh - 1 - margin)


def rects_intersect(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    return not (ax + aw <= bx or bx + bw <= ax or ay + ah <= by or by + bh <= ay)


def blob_rect(blob):
    return blob.x(), blob.y(), blob.w(), blob.h()


def find_red_blobs(img):
    try:
        return img.find_blobs(RED_THRESHOLDS, pixels_threshold=1,
                              area_threshold=1, merge=False)
    except Exception:
        return []


def find_black_blobs(img):
    try:
        return img.find_blobs(BLACK_THRESHOLDS, pixels_threshold=20,
                              area_threshold=20, merge=True)
    except Exception:
        return []


def detect_laser_spot_delta(img, off_frame, target_corners, previous_spot=None):
    """Return (spot, delta_max, blob_count) from the laser ON/OFF difference.

    ``difference`` is the absolute per-channel difference supported by the
    K230 image API. Converting that image to grayscale leaves a bright compact
    blob wherever the laser changed the paper, without assuming its hue.
    """
    if off_frame is None or target_corners is None:
        return None, 0, 0
    rect = corners_bbox(target_corners)
    roi = expand_rect(rect, DELTA_ROI_PAD)
    try:
        delta = img.copy()
        delta.difference(off_frame)
        gray = delta.to_grayscale()
        delta_max = gray.get_statistics(roi=roi).max()
    except Exception:
        return None, 0, 0

    best_valid = []
    best_blobs = []
    best_thr = DELTA_THRESHOLD
    last_edge_reject = 0
    last_big_reject = 0
    last_side_reject = 0
    last_total = 0
    for thr in DELTA_THRESHOLDS:
        try:
            blobs = gray.find_blobs(((thr, 255),), roi=roi,
                                    pixels_threshold=DELTA_MIN_PIXELS,
                                    area_threshold=DELTA_MIN_PIXELS,
                                    merge=True)
        except Exception:
            blobs = []
        valid = []
        edge_reject = 0
        big_reject = 0
        side_reject = 0
        for blob in blobs:
            bx = blob.cx()
            by = blob.cy()
            if not point_in_rect(bx, by, roi, margin=0):
                edge_reject += 1
                continue
            on_main_rect = point_in_rect(bx, by, rect, margin=0)
            max_pixels = DELTA_MAX_PIXELS if on_main_rect else DELTA_EDGE_MAX_PIXELS
            max_side = DELTA_MAX_SIDE if on_main_rect else DELTA_EDGE_MAX_SIDE
            if blob.pixels() > max_pixels:
                big_reject += 1
                continue
            if blob.w() > max_side or blob.h() > max_side:
                side_reject += 1
                continue
            valid.append(blob)
        last_edge_reject = edge_reject
        last_big_reject = big_reject
        last_side_reject = side_reject
        last_total = len(blobs)
        if valid:
            best_valid = valid
            best_blobs = blobs
            best_thr = thr
            break
    valid = best_valid
    blobs = best_blobs
    if not valid:
        print("@SPOT_REJECT,delta=%d,thr_last=%d,blobs=%d,edge=%d,big=%d,side=%d,roi=%d,%d,%d,%d" %
              (delta_max, DELTA_THRESHOLDS[-1], last_total, last_edge_reject,
               last_big_reject, last_side_reject, roi[0], roi[1], roi[2], roi[3]))
        return None, delta_max, last_total
    # Prefer a compact ON/OFF change that remains near the previous spot.
    # A tiny servo step should not teleport the laser tens of pixels; this
    # continuity gate rejects reflections and residual glow from the paper.
    px = None
    py = None
    if previous_spot is not None:
        px, py, _ = previous_spot

    def spot_score(b):
        return b.pixels() * 12 - b.w() * b.h()

    if px is not None:
        gate_sq = DELTA_CONTINUITY_GATE_PX * DELTA_CONTINUITY_GATE_PX
        near = [b for b in valid
                if (b.cx() - px) * (b.cx() - px) + (b.cy() - py) * (b.cy() - py) <= gate_sq]
        if near:
            blob = max(near, key=spot_score)
            if best_thr != DELTA_THRESHOLD:
                print("@SPOT_FALLBACK,thr=%d,x=%d,y=%d,pixels=%d" %
                      (best_thr, blob.cx(), blob.cy(), blob.pixels()))
            return (blob.cx(), blob.cy(), blob.pixels()), delta_max, len(blobs)

    blob = max(valid, key=spot_score)
    if best_thr != DELTA_THRESHOLD:
        print("@SPOT_FALLBACK,thr=%d,x=%d,y=%d,pixels=%d" %
              (best_thr, blob.cx(), blob.cy(), blob.pixels()))
    return (blob.cx(), blob.cy(), blob.pixels()), delta_max, len(blobs)


def red_evidence(red_blobs, rect, cx, cy):
    """Return (pixels, blob_count, centre_ok).

    The prescribed target has several concentric red circles centred inside its
    black frame.  Requiring both red evidence and its centre alignment removes
    the common false positive of a plain black rectangle in the background.
    """
    pixels = 0
    count = 0
    wx = 0
    wy = 0
    for blob in red_blobs:
        if blob.pixels() < 1:
            continue
        if not point_in_rect(blob.cx(), blob.cy(), rect, margin=2):
            continue
        p = blob.pixels()
        pixels += p
        count += 1
        wx += blob.cx() * p
        wy += blob.cy() * p
    if pixels == 0:
        return 0, 0, False
    rcx = wx // pixels
    rcy = wy // pixels
    _, _, w, h = rect
    # Red circles can be partly broken by exposure, hence a generous 24% gate.
    centre_ok = abs(rcx - cx) * 100 <= w * 24 and abs(rcy - cy) * 100 <= h * 24
    return pixels, count, centre_ok


def black_border_sides(black_blobs, rect):
    x, y, w, h = rect
    thickness = max(7, min(w, h) // 14)
    strips = (
        (x - thickness, y - thickness, w + 2 * thickness, thickness * 2),
        (x - thickness, y + h - thickness, w + 2 * thickness, thickness * 2),
        (x - thickness, y - thickness, thickness * 2, h + 2 * thickness),
        (x + w - thickness, y - thickness, thickness * 2, h + 2 * thickness),
    )
    sides = 0
    pixels = 0
    for sx, sy, sw, sh in strips:
        sx = clamp(sx, 0, WIDTH - 1)
        sy = clamp(sy, 0, HEIGHT - 1)
        sw = clamp(sw, 1, WIDTH - sx)
        sh = clamp(sh, 1, HEIGHT - sy)
        side_pixels = 0
        for blob in black_blobs:
            if rects_intersect(blob_rect(blob), (sx, sy, sw, sh)):
                side_pixels += blob.pixels()
        if side_pixels >= 35:
            sides += 1
            pixels += side_pixels
    return sides, pixels


def detect_candidates(img):
    red_blobs = find_red_blobs(img)
    black_blobs = find_black_blobs(img)
    mask = img.copy()
    try:
        mask.binary(BLACK_THRESHOLDS)
        rects = mask.find_rects(threshold=RECT_THRESHOLD)
    except Exception:
        rects = []

    candidates = []
    for rect_obj in rects:
        corners = order_corners(rect_obj.corners())
        x, y, w, h = corners_bbox(corners)
        area = w * h
        if w < MIN_FRAME_W or h < MIN_FRAME_H:
            continue
        if area < MIN_FRAME_AREA or area > MAX_FRAME_AREA:
            continue
        if x <= EDGE_MARGIN or y <= EDGE_MARGIN:
            continue
        if x + w >= WIDTH - EDGE_MARGIN or y + h >= HEIGHT - EDGE_MARGIN:
            continue
        aspect_x100 = w * 100 // max(1, h)
        if aspect_x100 < ASPECT_MIN_X100 or aspect_x100 > ASPECT_MAX_X100:
            continue

        cx, cy = diagonal_center(corners)
        frame = (x, y, w, h)
        sides, border_pixels = black_border_sides(black_blobs, frame)
        if sides < MIN_BLACK_SIDES:
            continue
        red_pixels, red_count, red_center_ok = red_evidence(red_blobs, frame, cx, cy)
        if red_pixels < MIN_RED_PIXELS or red_count < MIN_RED_BLOBS or not red_center_ok:
            continue

        # Score geometry/evidence only.  Temporal continuity is applied later,
        # so a highly-scored but distant rectangle cannot steal an existing lock.
        score = (rect_obj.magnitude() + border_pixels * 12 + red_pixels * 220 +
                 red_count * 1000 + area // 12)
        candidates.append({
            "cx": cx, "cy": cy, "w": w, "h": h,
            "corners": corners, "score": score,
            "sides": sides, "red": red_pixels,
        })
    return candidates


def dist_sq(a, b):
    dx = a["cx"] - b["cx"]
    dy = a["cy"] - b["cy"]
    return dx * dx + dy * dy


def filter_value(old, new):
    if old is None:
        return new
    return (old * FILTER_OLD + new * FILTER_NEW) // (FILTER_OLD + FILTER_NEW)


class TargetLock:
    def __init__(self):
        self.mode = "SEARCH"
        self.candidate = None
        self.acquire_count = 0
        self.lock = None
        self.fx = None
        self.fy = None
        self.hold_count = 0

    def choose(self, candidates):
        if not candidates:
            return None
        reference = self.lock if self.lock is not None else self.candidate
        if reference is None:
            return max(candidates, key=lambda c: c["score"])

        # While locked, continuity beats raw image score.  A gate proportional
        # to target size permits normal camera/small-car motion but blocks a
        # different background rectangle.
        gate = max(MIN_LOCK_GATE_PX, min(reference["w"], reference["h"]) *
                   LOCK_GATE_FRACTION_X100 // 100)
        nearby = [c for c in candidates if dist_sq(c, reference) <= gate * gate]
        if not nearby:
            return None
        return max(nearby, key=lambda c: c["score"])

    def update(self, candidates):
        chosen = self.choose(candidates)

        if self.mode in ("SEARCH", "ACQUIRE"):
            if chosen is None:
                self.mode = "SEARCH"
                self.candidate = None
                self.acquire_count = 0
                return None
            if self.candidate is not None and dist_sq(chosen, self.candidate) <= MAX_ACQUIRE_JUMP_PX ** 2:
                self.acquire_count += 1
            else:
                self.candidate = chosen
                self.acquire_count = 1
            self.candidate = chosen
            self.mode = "ACQUIRE"
            if self.acquire_count >= ACQUIRE_FRAMES:
                self.mode = "LOCK"
                self.lock = chosen
                self.fx = chosen["cx"]
                self.fy = chosen["cy"]
                self.hold_count = 0
            return chosen

        # LOCK / HOLD.  Keep the last coordinate for a bounded time on missing
        # frames; never accept a far-away replacement during that interval.
        if chosen is None:
            self.hold_count += 1
            if self.hold_count <= HOLD_FRAMES and self.lock is not None:
                self.mode = "HOLD"
                return self.lock
            self.mode = "SEARCH"
            self.candidate = None
            self.lock = None
            self.fx = None
            self.fy = None
            self.acquire_count = 0
            self.hold_count = 0
            return None

        self.mode = "LOCK"
        self.hold_count = 0
        self.lock = chosen
        self.fx = filter_value(self.fx, chosen["cx"])
        self.fy = filter_value(self.fy, chosen["cy"])
        return chosen

    def output_center(self):
        if self.mode == "LOCK" and self.fx is not None:
            return self.fx, self.fy
        return None


# --- Frozen target_center_final.py core, embedded for single-file IDE use. ---
FINAL_CANNY_LOW = 45
FINAL_CANNY_HIGH = 130
FINAL_EPSILON = 0.035
FINAL_MIN_AREA_RATIO = 0.010
FINAL_MAX_ANGLE_COS = 0.55
FINAL_BLUR = 5
FINAL_MIN_AREA = 4800
FINAL_MAX_AREA = 65000
FINAL_MIN_EDGE = 42
FINAL_MIN_RATIO = 38
FINAL_MAX_RATIO = 95
FINAL_MAX_OPPOSITE_ERROR = 72
FINAL_ACQUIRE_FRAMES = 2
FINAL_LIVE_GATE_PX = 38
FINAL_PREDICT_FRAMES = 3


def final_isqrt(v):
    if v <= 0:
        return 0
    x = v
    y = (x + 1) // 2
    while y < x:
        x = y
        y = (x + v // x) // 2
    return x


def final_order_corners(p):
    tl = min(p, key=lambda q: q[0] + q[1])
    br = max(p, key=lambda q: q[0] + q[1])
    tr = max(p, key=lambda q: q[0] - q[1])
    bl = min(p, key=lambda q: q[0] - q[1])
    return (tl, tr, br, bl) if len({tl, tr, br, bl}) == 4 else tuple(p)


def final_center(c):
    if len(c) != 4:
        return None
    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = c[0], c[2], c[1], c[3]
    den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if den == 0:
        return None
    a = x1*y2-y1*x2
    b = x3*y4-y3*x4
    return (int(round((a*(x3-x4)-(x1-x2)*b)/den)),
            int(round((a*(y3-y4)-(y1-y2)*b)/den)))


def final_area(c):
    total = 0
    for i in range(4):
        x1, y1 = c[i]
        x2, y2 = c[(i+1) % 4]
        total += x1*y2-y1*x2
    return abs(total) // 2


def final_dist2(x, y, item):
    return (x-item['cx'])**2 + (y-item['cy'])**2


def final_detect(img):
    try:
        raw_list = cv_lite.rgb888_find_rectangles_with_corners(
            SHAPE, img.to_numpy_ref(), FINAL_CANNY_LOW, FINAL_CANNY_HIGH,
            FINAL_EPSILON, FINAL_MIN_AREA_RATIO, FINAL_MAX_ANGLE_COS, FINAL_BLUR)
    except Exception as err:
        print('@CV_ERROR,error=%s' % type(err).__name__)
        return [], [], 0
    raws, strict = [], []
    for raw in raw_list:
        try:
            if len(raw) < 12:
                continue
            x, y, w, h = int(raw[0]), int(raw[1]), int(raw[2]), int(raw[3])
            corners = final_order_corners(((int(raw[4]),int(raw[5])),(int(raw[6]),int(raw[7])),
                                           (int(raw[8]),int(raw[9])),(int(raw[10]),int(raw[11]))))
            centre = final_center(corners)
            if centre is None:
                centre = (x+w//2, y+h//2)
            item = {'cx':centre[0], 'cy':centre[1], 'w':w, 'h':h,
                    'bbox_area':w*h, 'corners':corners}
            raws.append(item)
            area = final_area(corners)
            edges = [final_isqrt((corners[i][0]-corners[(i+1)%4][0])**2+
                                 (corners[i][1]-corners[(i+1)%4][1])**2) for i in range(4)]
            short, long = min(edges), max(edges)
            ratio = short*100//max(1,long)
            e0 = abs(edges[0]-edges[2])*100//max(1,max(edges[0],edges[2]))
            e1 = abs(edges[1]-edges[3])*100//max(1,max(edges[1],edges[3]))
            if (area >= FINAL_MIN_AREA and area <= FINAL_MAX_AREA and short >= FINAL_MIN_EDGE and
                    FINAL_MIN_RATIO <= ratio <= FINAL_MAX_RATIO and
                    e0 <= FINAL_MAX_OPPOSITE_ERROR and e1 <= FINAL_MAX_OPPOSITE_ERROR):
                item = dict(item)
                item['score'] = area + ratio*30 - (e0+e1)*25
                strict.append(item)
        except Exception:
            pass
    return strict, raws, len(raw_list)


class FinalTargetTracker:
    def __init__(self):
        self.lock = self.pending = None
        self.pending_n = self.vx = self.vy = self.miss = self.strict_run = 0
        # Kept for the base file's on-screen debug text.
        self.acquire_count = 0
        self.hold_count = 0
        self.x = self.y = None
        self.mode, self.source = 'SEARCH', '-'
    def choose(self, items, x, y, strict):
        if not items:
            return None
        if x is None:
            return max(items, key=lambda c:c.get('score', c['bbox_area']))
        gate = FINAL_LIVE_GATE_PX + min(20,abs(self.vx)+abs(self.vy))
        near = [c for c in items if final_dist2(x,y,c) <= gate*gate]
        return (max(near,key=lambda c:c['score']) if strict and near else
                (min(near,key=lambda c:final_dist2(x,y,c)) if near else None))
    def update(self, strict, raws):
        if self.lock is None:
            px = self.pending['cx'] if self.pending else None
            py = self.pending['cy'] if self.pending else None
            c = self.choose(strict,px,py,True)
            if c is None:
                self.pending=None; self.pending_n=0; self.acquire_count=0; self.mode='SEARCH'; return None
            self.pending_n = self.pending_n+1 if self.pending and final_dist2(px,py,c)<=42*42 else 1
            self.acquire_count = self.pending_n
            self.pending=c; self.mode='ACQUIRE'; self.source='S'
            if self.pending_n >= FINAL_ACQUIRE_FRAMES:
                self.lock=c; self.x,self.y=c['cx'],c['cy']; self.vx=self.vy=self.miss=0; self.hold_count=0; self.strict_run=1; self.mode='LOCK'
            return c
        c=self.choose(strict,self.x+self.vx,self.y+self.vy,True); source='S'
        if c is None:
            c=self.choose(raws,self.x+self.vx,self.y+self.vy,False); source='R'
        if c is None:
            self.miss += 1; self.strict_run=0
            self.hold_count = self.miss
            if self.miss <= FINAL_PREDICT_FRAMES:
                self.x=max(0,min(WIDTH-1,self.x+self.vx)); self.y=max(0,min(HEIGHT-1,self.y+self.vy))
                self.mode='PREDICT'; self.source='P'
            return self.lock if self.miss <= FINAL_PREDICT_FRAMES else None
        ox,oy=self.x,self.y; self.x=(ox+3*c['cx'])//4; self.y=(oy+3*c['cy'])//4
        self.vx=(self.vx*65+(c['cx']-ox)*35)//100; self.vy=(self.vy*65+(c['cy']-oy)*35)//100
        self.lock=c; self.miss=0; self.hold_count=0; self.strict_run=self.strict_run+1 if source=='S' else 0; self.mode='LOCK'; self.source=source
        return c
    def output_center(self):
        return (self.x,self.y) if self.x is not None and self.mode == 'LOCK' else None
    def aim_ready(self):
        return self.mode=='LOCK' and self.source=='S' and self.strict_run>=3


class PanPulse:
    def __init__(self):
        self.pwm = pwm_create_compat(PAN_PWM_CH, SERVO_FREQ_HZ, us_to_duty(PAN_STOP_US))
        self.us = PAN_STOP_US
        self.duty = us_to_duty(self.us)
    def pulse(self, error):
        if abs(error) <= ALIGN_X_PX:
            return False
        # Camera and laser are fixed on the same gimbal:
        #   dx = target_x - spot_x > 0 means the spot is left of target.
        #   To aim the laser right, the camera view target must move left.
        # Measured axis response:
        #   PAN_PLUS_US  -> target cx increases
        #   PAN_MINUS_US -> target cx decreases
        command = PAN_MINUS_US if error > 0 else PAN_PLUS_US
        self.us=command; self.duty=us_to_duty(command); pwm_write_compat(self.pwm,self.duty)
        print("@PAN_PULSE,error=%d,cmd_us=%d,cmd_duty=%d,hold_ms=%d" %
              (error, self.us, self.duty, PAN_PULSE_MS))
        time.sleep_ms(PAN_PULSE_MS)
        self.us=PAN_STOP_US; self.duty=us_to_duty(self.us); pwm_write_compat(self.pwm,self.duty)
        print("@PAN_STOP,stop_us=%d,stop_duty=%d" % (self.us, self.duty))
        return True
    def deinit(self):
        try: pwm_write_compat(self.pwm,us_to_duty(PAN_STOP_US)); self.pwm.deinit()
        except Exception: pass


def draw_debug(img, tracker, shown, phase, loop_count, laser_on, spot,
               pan, tilt, err_x, err_y, aim_centre=None):
    img.draw_string_advanced(0, 0, 10, "TARGET CLOSED LOOP", color=(255, 255, 0))
    img.draw_string_advanced(0, 13, 10, "%s acq=%d hold=%d" %
                             (tracker.mode, tracker.acquire_count, tracker.hold_count),
                             color=(255, 255, 0))
    if shown is not None:
        color = (0, 255, 0) if tracker.mode == "LOCK" else (255, 180, 0)
        for i in range(4):
            x1, y1 = shown["corners"][i]
            x2, y2 = shown["corners"][(i + 1) % 4]
            img.draw_line(x1, y1, x2, y2, color=color, thickness=2)
        img.draw_cross(shown["cx"], shown["cy"], color=(255, 0, 0), size=10)
    centre = aim_centre if aim_centre is not None else tracker.output_center()
    if centre is not None:
        cx, cy = centre
        img.draw_cross(cx, cy, color=(0, 255, 0), size=18)
        img.draw_circle(cx, cy, 7, color=(0, 255, 0), thickness=2)
        img.draw_string_advanced(0, 26, 10, "center=%d,%d err=%d,%d" %
                                 (cx, cy, err_x, err_y),
                                 color=(0, 255, 0))
    elif tracker.mode == "SEARCH":
        img.draw_string_advanced(0, 26, 10, "No verified target", color=(255, 0, 0))
    state = "%s %d/%d" % (phase, loop_count, MAX_CONTROL_LOOPS)
    img.draw_string_advanced(0, 39, 10, state,
                             color=(255, 0, 0) if laser_on else (0, 255, 255))
    img.draw_string_advanced(0, 52, 10, "pan=%d/%d tilt=%d/%d" %
                             (pan.us, pan.duty, tilt.us, tilt.duty),
                             color=(0, 255, 255))
    if spot is not None and centre is not None:
        sx, sy, pixels = spot
        dx = centre[0] - sx
        dy = centre[1] - sy
        img.draw_cross(sx, sy, color=(255, 0, 255), size=16)
        img.draw_circle(sx, sy, 6, color=(255, 0, 255), thickness=2)
        img.draw_line(centre[0], centre[1], sx, sy, color=(255, 0, 255), thickness=1)
        img.draw_string_advanced(0, 65, 10, "spot=%d,%d err=%d,%d" %
                                 (sx, sy, dx, dy), color=(255, 0, 255))


def main():
    print("LASER_TARGET_CLOSED_LOOP_START")
    print("measure spot -> small servo move -> settle -> re-measure")
    print("pan PIN33/GPIO52, tilt PIN35/GPIO42, relay PIN37/GPIO32")
    sensor = None
    relay = None
    pan = None
    tilt = None
    tracker = FinalTargetTracker()
    pan_guard = ProgressGuard()
    tilt_guard = ProgressGuard()
    last_print_ms = time.ticks_ms()
    next_action_ms = time.ticks_ms()
    frame_id = 0
    loop_count = 0
    aligned_count = 0
    phase = "SEARCH"
    laser_started_ms = None
    pulse_background = None
    measured_this_pulse = False
    final_started_ms = None
    err_x = 0
    err_y = 0
    spot = None
    last_spot = None
    pulse_centre = None
    pulse_corners = None
    last_delta_max = 0
    last_blob_count = 0
    last_move_mag = 0
    stale_spot_count = 0

    try:
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except Exception as e:
            print("exitpoint enable err", type(e).__name__, e)

        try:
            MediaManager.deinit()
            time.sleep_ms(200)
        except Exception:
            pass

        fpioa = FPIOA()
        fpioa.set_function(RELAY_GPIO, FPIOA.GPIO32)
        fpioa.set_function(PAN_GPIO, PAN_PWM_FUNC)
        fpioa.set_function(TILT_GPIO, TILT_PWM_FUNC)
        relay = Pin(RELAY_GPIO, Pin.OUT, value=1 - RELAY_ACTIVE_LEVEL)
        pan = PanPulse()
        tilt = Servo(TILT_PWM_CH, TILT_CENTER_US, TILT_MIN_US, TILT_MAX_US)

        sensor = Sensor(id=SENSOR_ID, fps=CAMERA_FPS)
        sensor.reset()
        sensor.set_framesize(width=WIDTH, height=HEIGHT, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
        Display.init(Display.VIRT, width=WIDTH, height=HEIGHT, to_ide=True)
        MediaManager.init()
        sensor.run()
        time.sleep_ms(250)
        sensor.auto_exposure(False)
        sensor.exposure(3000)
        sensor.again(24.0)
        time.sleep_ms(250)

        while True:
            os.exitpoint()
            frame_id += 1
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            laser_measure_active = (laser_started_ms is not None or
                                    phase == "FINAL_FIRE")
            if laser_measure_active:
                candidates = []
                raw_count = 0
                shown = tracker.lock
            else:
                strict, candidates, raw_count = final_detect(img)
                shown = tracker.update(strict, candidates)
            now = time.ticks_ms()
            centre = tracker.output_center()
            if laser_started_ms is not None and pulse_centre is not None:
                centre = pulse_centre
            spot = last_spot

            if phase == "FINAL_FIRE":
                if time.ticks_diff(now, final_started_ms) >= FINAL_FIRE_MS:
                    relay.value(1 - RELAY_ACTIVE_LEVEL)
                    phase = "DONE"
                    print("@DONE,final_fire_ms=%d" % FINAL_FIRE_MS)
            elif phase == "DONE" or phase == "STOP":
                relay.value(1 - RELAY_ACTIVE_LEVEL)
            elif laser_started_ms is not None:
                pulse_elapsed_ms = time.ticks_diff(now, laser_started_ms)
                if (not measured_this_pulse and
                        pulse_centre is not None and pulse_corners is not None and
                        pulse_elapsed_ms >= DELTA_SETTLE_MS):
                    new_spot, delta_max, blob_count = detect_laser_spot_delta(
                        img, pulse_background, pulse_corners, last_spot)
                    last_delta_max = delta_max
                    last_blob_count = blob_count
                    if new_spot is not None:
                        previous_spot = last_spot
                        spot = new_spot
                        measured_this_pulse = True
                        err_x = pulse_centre[0] - spot[0]
                        err_y = pulse_centre[1] - spot[1]
                        relay.value(1 - RELAY_ACTIVE_LEVEL)
                        laser_started_ms = None
                        pulse_background = None
                        print("@MEASURE,n=%d,aim=%d,%d,sx=%d,sy=%d,px=%d,dx=%d,dy=%d,delta=%d,blobs=%d" %
                              (loop_count, pulse_centre[0], pulse_centre[1], spot[0], spot[1],
                               spot[2], err_x, err_y, delta_max, blob_count))

                        stale_spot = False
                        if previous_spot is not None and last_move_mag >= STALE_MOVE_MIN_US:
                            pdx = spot[0] - previous_spot[0]
                            pdy = spot[1] - previous_spot[1]
                            if pdx * pdx + pdy * pdy <= STALE_SPOT_GATE_PX * STALE_SPOT_GATE_PX:
                                stale_spot_count += 1
                                if stale_spot_count >= STALE_SPOT_LIMIT:
                                    stale_spot = True
                            else:
                                stale_spot_count = 0
                        else:
                            stale_spot_count = 0

                        if stale_spot:
                            last_spot = None
                            last_move_mag = 0
                            aligned_count = 0
                            phase = "SETTLE"
                            next_action_ms = now + MISS_RETRY_MS
                            print("@SKIP,stale_spot,sx=%d,sy=%d,count=%d" %
                                  (spot[0], spot[1], stale_spot_count))
                            continue

                        last_spot = new_spot

                        aligned = (abs(err_x) <= ALIGN_X_PX and
                                   abs(err_y) <= ALIGN_Y_PX)
                        if aligned:
                            last_move_mag = 0
                            aligned_count += 1
                            print("@ALIGN,count=%d/%d" %
                                  (aligned_count, ALIGN_CONFIRM_PULSES))
                            if aligned_count >= ALIGN_CONFIRM_PULSES:
                                if ENABLE_FINAL_FIRE:
                                    phase = "FINAL_FIRE"
                                    final_started_ms = now
                                    relay.value(RELAY_ACTIVE_LEVEL)
                                    print("@FIRE,verified_dx=%d,verified_dy=%d" %
                                          (err_x, err_y))
                                else:
                                    phase = "STOP"
                                    print("@ALIGNED,final_fire_disabled=1,dx=%d,dy=%d" %
                                          (err_x, err_y))
                            else:
                                phase = "SETTLE"
                                next_action_ms = now + SETTLE_AFTER_MOVE_MS
                        else:
                            aligned_count = 0
                            safe = (abs(err_x) <= MAX_SAFE_ERROR_X and
                                    abs(err_y) <= MAX_SAFE_ERROR_Y)
                            if not safe:
                                phase = "SETTLE"
                                next_action_ms = now + MISS_RETRY_MS
                                print("@SKIP,unsafe_error=%d,%d" % (err_x, err_y))
                            else:
                                pan_step = 0
                                tilt_step = 0
                                if pan_guard.allow(err_x, ALIGN_X_PX):
                                    pan_step = err_x
                                if tilt_guard.allow(err_y, ALIGN_Y_PX):
                                    tilt_step = p_step(err_y, TILT_DIR,
                                                       TILT_KP_US_PER_PX, TILT_MAX_STEP_US)
                                pan_moved = pan.pulse(pan_step) if pan_step else False
                                tilt_moved = tilt.add_us(tilt_step) if tilt_step else False
                                print("@MOVE,n=%d,dx=%d,dy=%d,pan_step=%d,tilt_step=%d,pan=%d,tilt=%d,pan_duty=%d,tilt_duty=%d" %
                                      (loop_count, err_x, err_y, pan_step, tilt_step, pan.us, tilt.us,
                                       pan.duty, tilt.duty))
                                if not pan_moved and not tilt_moved:
                                    last_move_mag = 0
                                    phase = "SETTLE"
                                    next_action_ms = now + MISS_RETRY_MS
                                    print("@HOLD,no_servo_move,pan=%d,tilt=%d" %
                                          (pan.us, tilt.us))
                                else:
                                    last_move_mag = abs(pan_step) + abs(tilt_step)
                                    phase = "SETTLE"
                                    next_action_ms = now + SETTLE_AFTER_MOVE_MS
                    elif pulse_elapsed_ms >= CAL_PULSE_ON_MS:
                        relay.value(1 - RELAY_ACTIVE_LEVEL)
                        laser_started_ms = None
                        pulse_background = None
                        last_move_mag = 0
                        phase = "SETTLE"
                        next_action_ms = now + MISS_RETRY_MS
                        print("@MISS,n=%d,no_spot,delta=%d,blobs=%d" %
                              (loop_count, last_delta_max, last_blob_count))
                elif pulse_elapsed_ms >= CAL_PULSE_ON_MS:
                    relay.value(1 - RELAY_ACTIVE_LEVEL)
                    laser_started_ms = None
                    pulse_background = None
                    last_move_mag = 0
                    phase = "SETTLE"
                    next_action_ms = now + MISS_RETRY_MS
                    print("@MISS,n=%d,no_spot,delta=%d,blobs=%d" %
                          (loop_count, last_delta_max, last_blob_count))
            elif phase == "SETTLE":
                if time.ticks_diff(now, next_action_ms) >= 0:
                    phase = "SEARCH"
            elif phase == "SEARCH":
                if tracker.aim_ready() and centre is not None and shown is not None:
                    if loop_count >= MAX_CONTROL_LOOPS:
                        phase = "STOP"
                        print("@STOP,max_control_loops")
                    else:
                        loop_count += 1
                        pulse_centre = (centre[0], centre[1])
                        if shown is not None:
                            pulse_corners = shown["corners"]
                        elif tracker.lock is not None:
                            pulse_corners = tracker.lock["corners"]
                        else:
                            pulse_corners = None
                        if pulse_corners is None:
                            loop_count -= 1
                            print("@WAIT,no_target_corners")
                            continue
                        pulse_background = img.copy()
                        measured_this_pulse = False
                        laser_started_ms = now
                        relay.value(RELAY_ACTIVE_LEVEL)
                        phase = "MEASURE"
                        print("@PULSE,n=%d,aim=%d,%d" %
                              (loop_count, pulse_centre[0], pulse_centre[1]))
                else:
                    relay.value(1 - RELAY_ACTIVE_LEVEL)
                    pulse_centre = None
                    pulse_corners = None
                    pulse_background = None
                    measured_this_pulse = False
                    last_spot = None
                    last_move_mag = 0
                    stale_spot_count = 0

            laser_on = (phase == "MEASURE" or phase == "FINAL_FIRE")
            debug_aim = pulse_centre if phase in ("MEASURE", "SETTLE") else None
            draw_debug(img, tracker, shown, phase, loop_count, laser_on, spot,
                       pan, tilt, err_x, err_y, debug_aim)
            Display.show_image(img)

            if time.ticks_diff(now, last_print_ms) >= PRINT_INTERVAL_MS:
                last_print_ms = now
                if centre is None:
                    print("@LOCK,frame=%d,state=%s,candidates=%d,phase=%s" %
                          (frame_id, tracker.mode, len(candidates), phase))
                else:
                    print("@LOCK,frame=%d,state=%s,cx=%d,cy=%d,phase=%s,loops=%d" %
                          (frame_id, tracker.mode, centre[0], centre[1],
                           phase, loop_count))
            # Avoid preview stutter from collecting every camera frame.
            if frame_id % 30 == 0:
                gc.collect()
    finally:
        if relay is not None:
            try:
                relay.value(1 - RELAY_ACTIVE_LEVEL)
            except Exception:
                pass
        try:
            if sensor is not None:
                sensor.stop()
        except Exception:
            pass
        try:
            Display.deinit()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass
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


main()
