"""
Final visual target-centre detector for the K230 laser-target project.

Single RGB888 stream, fixed 3 ms exposure, and cv_lite only.  No PWM, relay,
laser or global colour scan is included.  Green=strict measured target,
cyan=nearby relaxed measured target, orange=short prediction.

Important anti-glare rule for later expansion:
Do NOT scan the whole image for red blobs.  In this room it mistakes the wall
and background for the red rings and causes both false centres and preview
stutter.  If reflection compensation is ever needed, inspect only the small
region around an already locked/predicted frame and use the remaining black
edge segments as supporting evidence.  Such support may maintain tracking but
must never by itself authorize aiming or laser firing.
"""

import gc
import os
import time

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

# New target acquisition accepts only this geometry.
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
PRINT_MS = 120


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
        # Future servo code may use only a genuine strict measured centre.
        return self.mode == "LOCK" and self.source == "S" and self.strict_run >= STRICT_AIM_FRAMES


def draw(img, tracker, shown):
    if tracker.mode == "LOCK" and tracker.source == "S":
        color = (0, 255, 0)
    elif tracker.mode == "LOCK" and tracker.source == "R":
        color = (0, 255, 255)
    elif tracker.mode == "PREDICT":
        color = (255, 150, 0)
    else:
        return
    if shown is not None:
        for i in range(4):
            p1, p2 = shown["corners"][i], shown["corners"][(i + 1) % 4]
            img.draw_line(p1[0], p1[1], p2[0], p2[1], color=color, thickness=2)
    img.draw_cross(tracker.x, tracker.y, color=color, size=18)
    img.draw_circle(tracker.x, tracker.y, 7, color=color, thickness=2)


def main():
    print("TARGET_CENTER_FINAL_V6_START")
    print("No PWM/relay/laser. Green=S cyan=R orange=P. aim=1 only after strict measured frames.")
    sensor = None
    tracker = TargetTracker()
    frame, last_print = 0, time.ticks_ms()
    try:
        try:
            MediaManager.deinit()
            time.sleep_ms(200)
        except Exception:
            pass
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
        print("@CAM,ae=%s,exposure_us=%d" % (str(sensor.auto_exposure()), int(sensor.exposure())))
        while True:
            os.exitpoint()
            frame += 1
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            strict, raws, raw_count = detect(img)
            shown = tracker.update(strict, raws)
            draw(img, tracker, shown)
            Display.show_image(img)
            now = time.ticks_ms()
            if time.ticks_diff(now, last_print) >= PRINT_MS:
                last_print = now
                if tracker.x is None:
                    print("@TARGET,f=%d,state=%s,raw=%d,strict=%d,aim=0" %
                          (frame, tracker.mode, raw_count, len(strict)))
                else:
                    print("@TARGET,f=%d,state=%s,src=%s,cx=%d,cy=%d,vx=%d,vy=%d,miss=%d,strict_run=%d,raw=%d,strict=%d,aim=%d" %
                          (frame, tracker.mode, tracker.source, tracker.x, tracker.y,
                           tracker.vx, tracker.vy, tracker.miss, tracker.strict_run,
                           raw_count, len(strict), 1 if tracker.aim_ready() else 0))
            gc.collect()
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


main()
