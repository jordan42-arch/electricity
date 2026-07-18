"""
K230 gimbal axis response test by target-centre motion.

Purpose:
  Automatically infer servo response from the detected target centre. This
  avoids judging direction by eye.

Tests:
  PAN_PLUS   : DS3115 on GPIO52/PWM4, 1650 us for 180 ms, then 1500 us stop
  PAN_MINUS  : DS3115 on GPIO52/PWM4, 1350 us for 180 ms, then 1500 us stop
  TILT_PLUS  : MG996R on GPIO42/PWM0, 1500 us -> 1550 us
  TILT_MINUS : MG996R on GPIO42/PWM0, 1550 us -> 1450 us

Output:
  @AXIS_RESULT,label=...,before=x,y,after=x,y,dcx=...,dcy=...

No relay and no laser are used.
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
SENSOR_FPS = 90
MANUAL_EXPOSURE_US = 3000
MANUAL_GAIN = 24.0

CANNY_LOW = 45
CANNY_HIGH = 130
EPSILON = 0.035
MIN_AREA_RATIO = 0.010
MAX_ANGLE_COS = 0.55
BLUR = 5

STRICT_MIN_AREA = 4800
STRICT_MAX_AREA = 65000
STRICT_MIN_EDGE = 42
STRICT_MIN_RATIO = 38
STRICT_MAX_RATIO = 95
STRICT_MAX_OPPOSITE_ERROR = 72

ACQUIRE_FRAMES = 2
LIVE_GATE_PX = 38
PREDICT_FRAMES = 3
POSITION_OLD_X100 = 25
VELOCITY_KEEP_X100 = 65
STRICT_AIM_FRAMES = 3

PAN_GPIO = 52
PAN_PWM_CH = 4
PAN_STOP_US = 1500
PAN_PLUS_US = 1650
PAN_MINUS_US = 1350
PAN_MOVE_MS = 180

TILT_GPIO = 42
TILT_PWM_CH = 0
TILT_CENTER_US = 1500
TILT_PLUS_US = 1550
TILT_MINUS_US = 1450

FREQ_HZ = 50
PERIOD_US = 20000

SETTLE_MS = 450
SAMPLE_FRAMES = 12
LOCK_TIMEOUT_MS = 3500
HOLD_PREVIEW_AFTER_DONE = True


def isqrt(v):
    if v <= 0:
        return 0
    x = v
    y = (x + 1) // 2
    while y < x:
        x = y
        y = (x + v // x) // 2
    return x


def order_corners(p):
    tl = min(p, key=lambda q: q[0] + q[1])
    br = max(p, key=lambda q: q[0] + q[1])
    tr = max(p, key=lambda q: q[0] - q[1])
    bl = min(p, key=lambda q: q[0] - q[1])
    return (tl, tr, br, bl) if len({tl, tr, br, bl}) == 4 else tuple(p)


def diagonal_center(c):
    if len(c) != 4:
        return None
    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = c[0], c[2], c[1], c[3]
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if den == 0:
        return None
    a = x1 * y2 - y1 * x2
    b = x3 * y4 - y3 * x4
    return (int(round((a * (x3 - x4) - (x1 - x2) * b) / den)),
            int(round((a * (y3 - y4) - (y1 - y2) * b) / den)))


def polygon_area(c):
    total = 0
    for i in range(4):
        x1, y1 = c[i]
        x2, y2 = c[(i + 1) % 4]
        total += x1 * y2 - y1 * x2
    return abs(total) // 2


def candidate_dist2(x, y, item):
    dx = x - item["cx"]
    dy = y - item["cy"]
    return dx * dx + dy * dy


def parse_raw(raw):
    try:
        if len(raw) < 12:
            return None
        x, y, w, h = int(raw[0]), int(raw[1]), int(raw[2]), int(raw[3])
        if w < 20 or h < 20 or w * h < 1200:
            return None
        corners = order_corners(((int(raw[4]), int(raw[5])), (int(raw[6]), int(raw[7])),
                                 (int(raw[8]), int(raw[9])), (int(raw[10]), int(raw[11]))))
        centre = diagonal_center(corners)
        if centre is None or not (0 <= centre[0] < WIDTH and 0 <= centre[1] < HEIGHT):
            centre = (x + w // 2, y + h // 2)
        return {"cx": centre[0], "cy": centre[1], "w": w, "h": h,
                "bbox_area": w * h, "corners": corners}
    except Exception:
        return None


def strictify(item):
    try:
        c = item["corners"]
        if len(c) != 4 or len(set(c)) != 4:
            return None
        area = polygon_area(c)
        if area < STRICT_MIN_AREA or area > STRICT_MAX_AREA:
            return None
        edges = [isqrt((c[i][0] - c[(i + 1) % 4][0]) ** 2 +
                       (c[i][1] - c[(i + 1) % 4][1]) ** 2) for i in range(4)]
        short, long = min(edges), max(edges)
        if short < STRICT_MIN_EDGE or long == 0:
            return None
        ratio = short * 100 // long
        if ratio < STRICT_MIN_RATIO or ratio > STRICT_MAX_RATIO:
            return None
        e0 = abs(edges[0] - edges[2]) * 100 // max(1, max(edges[0], edges[2]))
        e1 = abs(edges[1] - edges[3]) * 100 // max(1, max(edges[1], edges[3]))
        if e0 > STRICT_MAX_OPPOSITE_ERROR or e1 > STRICT_MAX_OPPOSITE_ERROR:
            return None
        out = dict(item)
        out["score"] = area + ratio * 30 - (e0 + e1) * 25
        return out
    except Exception:
        return None


def detect(img):
    try:
        raw_list = cv_lite.rgb888_find_rectangles_with_corners(
            SHAPE, img.to_numpy_ref(), CANNY_LOW, CANNY_HIGH, EPSILON,
            MIN_AREA_RATIO, MAX_ANGLE_COS, BLUR)
    except Exception as err:
        print("@CV_ERROR,error=%s" % type(err).__name__)
        return [], [], 0
    raws, strict = [], []
    for raw in raw_list:
        item = parse_raw(raw)
        if item is not None:
            raws.append(item)
            good = strictify(item)
            if good is not None:
                strict.append(good)
    return strict, raws, len(raw_list)


class TargetTracker:
    def __init__(self):
        self.lock = None
        self.pending = None
        self.pending_n = 0
        self.x = None
        self.y = None
        self.vx = 0
        self.vy = 0
        self.miss = 0
        self.strict_run = 0
        self.mode = "SEARCH"
        self.source = "-"

    def choose(self, candidates, x, y, strict):
        if not candidates:
            return None
        if x is None:
            return max(candidates, key=lambda c: c.get("score", c["bbox_area"]))
        gate = LIVE_GATE_PX + min(20, abs(self.vx) + abs(self.vy))
        near = [c for c in candidates if candidate_dist2(x, y, c) <= gate * gate]
        if not near:
            return None
        return max(near, key=lambda c: c["score"]) if strict else min(
            near, key=lambda c: candidate_dist2(x, y, c))

    def clear(self):
        self.lock = self.pending = None
        self.pending_n = 0
        self.x = self.y = None
        self.vx = self.vy = 0
        self.miss = self.strict_run = 0
        self.mode, self.source = "SEARCH", "-"

    def update(self, strict, raws):
        if self.lock is None:
            px = self.pending["cx"] if self.pending is not None else None
            py = self.pending["cy"] if self.pending is not None else None
            chosen = self.choose(strict, px, py, True)
            if chosen is None:
                self.pending = None
                self.pending_n = 0
                self.mode, self.source = "SEARCH", "-"
                return None
            if self.pending is not None and candidate_dist2(px, py, chosen) <= 42 * 42:
                self.pending_n += 1
            else:
                self.pending_n = 1
            self.pending = chosen
            self.mode, self.source = "ACQUIRE", "S"
            if self.pending_n >= ACQUIRE_FRAMES:
                self.lock = chosen
                self.x, self.y = chosen["cx"], chosen["cy"]
                self.vx = self.vy = self.miss = 0
                self.strict_run = 1
                self.mode = "LOCK"
            return chosen

        px, py = self.x + self.vx, self.y + self.vy
        chosen = self.choose(strict, px, py, True)
        source = "S"
        if chosen is None:
            chosen = self.choose(raws, px, py, False)
            source = "R"
        if chosen is None:
            self.miss += 1
            self.strict_run = 0
            if self.miss <= PREDICT_FRAMES:
                self.x = max(0, min(WIDTH - 1, self.x + self.vx))
                self.y = max(0, min(HEIGHT - 1, self.y + self.vy))
                self.mode, self.source = "PREDICT", "P"
                return self.lock
            self.clear()
            return None

        oldx, oldy = self.x, self.y
        self.x = (oldx * POSITION_OLD_X100 + chosen["cx"] * (100 - POSITION_OLD_X100)) // 100
        self.y = (oldy * POSITION_OLD_X100 + chosen["cy"] * (100 - POSITION_OLD_X100)) // 100
        dx, dy = chosen["cx"] - oldx, chosen["cy"] - oldy
        self.vx = (self.vx * VELOCITY_KEEP_X100 + dx * (100 - VELOCITY_KEEP_X100)) // 100
        self.vy = (self.vy * VELOCITY_KEEP_X100 + dy * (100 - VELOCITY_KEEP_X100)) // 100
        self.lock = chosen
        self.miss = 0
        self.strict_run = self.strict_run + 1 if source == "S" else 0
        self.mode, self.source = "LOCK", source
        return chosen

    def aim_ready(self):
        return self.mode == "LOCK" and self.source == "S" and self.strict_run >= STRICT_AIM_FRAMES


def draw(img, tracker, shown, label):
    if shown is not None:
        for i in range(4):
            p1, p2 = shown["corners"][i], shown["corners"][(i + 1) % 4]
            img.draw_line(p1[0], p1[1], p2[0], p2[1], color=(0, 255, 0), thickness=2)
    if tracker.x is not None:
        img.draw_cross(tracker.x, tracker.y, color=(0, 255, 0), size=18)
        img.draw_circle(tracker.x, tracker.y, 7, color=(0, 255, 0), thickness=2)
    img.draw_string_advanced(0, 0, 10, label, color=(255, 255, 0))
    img.draw_string_advanced(0, 14, 10, "%s %s" % (tracker.mode, tracker.source),
                             color=(255, 255, 0))


def us_to_duty_u16(us):
    return (int(us) * 65535 + PERIOD_US // 2) // PERIOD_US


def write_us(pwm, us):
    pwm.duty_u16(us_to_duty_u16(us))


def init_pwm(ch, gpio, func):
    fpioa = FPIOA()
    fpioa.set_function(gpio, func)
    pwm = PWM(ch)
    try:
        pwm.freq(FREQ_HZ)
    except Exception as e:
        print("@PWM_FREQ_ERR,ch=%d,type=%s,msg=%s" % (ch, type(e).__name__, e))
    return pwm


def sample_center(sensor, tracker, label):
    xs, ys = [], []
    start = time.ticks_ms()
    frame = 0
    while time.ticks_diff(time.ticks_ms(), start) < LOCK_TIMEOUT_MS:
        os.exitpoint()
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        strict, raws, raw_count = detect(img)
        shown = tracker.update(strict, raws)
        draw(img, tracker, shown, label)
        Display.show_image(img)
        frame += 1
        if tracker.aim_ready():
            xs.append(tracker.x)
            ys.append(tracker.y)
            if len(xs) >= SAMPLE_FRAMES:
                xs.sort()
                ys.sort()
                cx = xs[len(xs) // 2]
                cy = ys[len(ys) // 2]
                print("@CENTER,label=%s,cx=%d,cy=%d,n=%d,raw=%d,strict=%d" %
                      (label, cx, cy, len(xs), raw_count, len(strict)))
                return (cx, cy)
        if frame % 20 == 0:
            print("@LOCK_WAIT,label=%s,state=%s,src=%s,n=%d" %
                  (label, tracker.mode, tracker.source, len(xs)))
        gc.collect()
    print("@CENTER_FAIL,label=%s,state=%s,src=%s,n=%d" %
          (label, tracker.mode, tracker.source, len(xs)))
    return None


def pan_pulse(pwm, us):
    write_us(pwm, us)
    print("@PAN_CMD,us=%d,ms=%d" % (us, PAN_MOVE_MS))
    time.sleep_ms(PAN_MOVE_MS)
    write_us(pwm, PAN_STOP_US)
    print("@PAN_STOP,us=%d" % PAN_STOP_US)


def measure_action(sensor, tracker, label, action):
    before = sample_center(sensor, tracker, label + "_BEFORE")
    if before is None:
        print("@AXIS_ABORT,label=%s,reason=no_before" % label)
        return
    action()
    time.sleep_ms(SETTLE_MS)
    after = sample_center(sensor, tracker, label + "_AFTER")
    if after is None:
        print("@AXIS_ABORT,label=%s,reason=no_after,before=%d,%d" %
              (label, before[0], before[1]))
        return
    print("@AXIS_RESULT,label=%s,before=%d,%d,after=%d,%d,dcx=%d,dcy=%d" %
          (label, before[0], before[1], after[0], after[1],
           after[0] - before[0], after[1] - before[1]))


def hold_preview(sensor, tracker):
    print("@HOLD_PREVIEW,manual_IDE_interrupt_to_exit=1")
    frame = 0
    while True:
        os.exitpoint()
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        strict, raws, raw_count = detect(img)
        shown = tracker.update(strict, raws)
        draw(img, tracker, shown, "DONE HOLD PREVIEW")
        Display.show_image(img)
        frame += 1
        if frame % 30 == 0:
            if tracker.x is None:
                print("@HOLD,state=%s,raw=%d,strict=%d" %
                      (tracker.mode, raw_count, len(strict)))
            else:
                print("@HOLD,state=%s,src=%s,cx=%d,cy=%d,raw=%d,strict=%d" %
                      (tracker.mode, tracker.source, tracker.x, tracker.y,
                       raw_count, len(strict)))
        if frame % 30 == 0:
            gc.collect()


def main():
    print("AXIS_RESPONSE_BY_TARGET_START")
    print("No relay/laser. Uses target centre movement to infer axis response.")
    sensor = None
    pan = None
    tilt = None
    tracker = TargetTracker()
    try:
        try:
            MediaManager.deinit()
            time.sleep_ms(200)
        except Exception:
            pass

        # Keep relay/laser off in case the line is connected.
        fpioa = FPIOA()
        fpioa.set_function(32, FPIOA.GPIO32)
        relay = Pin(32, Pin.OUT, value=0)

        pan = init_pwm(PAN_PWM_CH, PAN_GPIO, FPIOA.PWM4)
        tilt = init_pwm(TILT_PWM_CH, TILT_GPIO, FPIOA.PWM0)
        write_us(pan, PAN_STOP_US)
        write_us(tilt, TILT_CENTER_US)
        time.sleep_ms(700)

        sensor = Sensor(id=SENSOR_ID, fps=SENSOR_FPS)
        sensor.reset()
        sensor.set_framesize(width=WIDTH, height=HEIGHT, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
        sensor.auto_exposure(False)
        Display.init(Display.VIRT, width=WIDTH, height=HEIGHT, to_ide=True)
        MediaManager.init()
        sensor.run()
        time.sleep_ms(250)
        sensor.exposure(MANUAL_EXPOSURE_US)
        sensor.again(MANUAL_GAIN)
        time.sleep_ms(250)

        measure_action(sensor, tracker, "PAN_PLUS", lambda: pan_pulse(pan, PAN_PLUS_US))
        measure_action(sensor, tracker, "PAN_MINUS", lambda: pan_pulse(pan, PAN_MINUS_US))
        measure_action(sensor, tracker, "TILT_PLUS", lambda: write_us(tilt, TILT_PLUS_US))
        measure_action(sensor, tracker, "TILT_MINUS", lambda: write_us(tilt, TILT_MINUS_US))

        write_us(pan, PAN_STOP_US)
        write_us(tilt, TILT_CENTER_US)
        relay.value(0)
        print("@DONE,pan_stop=%d,tilt_center=%d" % (PAN_STOP_US, TILT_CENTER_US))
        if HOLD_PREVIEW_AFTER_DONE:
            hold_preview(sensor, tracker)
    finally:
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
                write_us(pan, PAN_STOP_US)
                pan.deinit()
        except Exception:
            pass
        try:
            if tilt is not None:
                write_us(tilt, TILT_CENTER_US)
                tilt.deinit()
        except Exception:
            pass


main()
