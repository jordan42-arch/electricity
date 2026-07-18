"""
K230 / CanMV IDE standalone laser-spot lock test.

Purpose:
  - NO servo movement.
  - NO closed-loop control.
  - Laser relay stays ON while the program runs.
  - Verify whether the laser spot can be continuously locked while it appears
    purple on white paper or mixed/white on other materials.

Run:
  1. Copy this file to /sd/ or run it directly from CanMV IDE.
  2. Keep the target/camera still first, then move the target/laser spot.
  3. Watch purple cross/circle and logs:
       @SPOT_ACQUIRE = candidate is being confirmed
       @SPOT_LOCK    = current frame found a confirmed spot
       @SPOT_HOLD    = current frame missed, but short hold is active
       @SPOT_MISS    = lock lost

Notes:
  - This test deliberately does NOT use laser ON/OFF difference, because the
    laser is always on here.
  - Spot detection uses two direct cues:
       1) compact bright blob in grayscale;
       2) compact purple/red/blue-ish blob in Lab thresholds.
  - This file intentionally searches the whole image for diagnosis.  The final
    closed-loop controller may later restrict valid spots to the target area.
  - Continuity is used heavily.  Once locked, a candidate near the previous
    spot beats a brighter but distant reflection.
"""

import os
import time
import gc
from machine import FPIOA, Pin
from media.sensor import *
from media.display import *
from media.media import *


SENSOR_ID = 2
WIDTH = 320
HEIGHT = 240
CAMERA_FPS = 90

RELAY_GPIO = 32
RELAY_ACTIVE_LEVEL = 1

# Same camera baseline as the accepted target-centre code.  Keeping exposure
# manual is important; auto exposure will chase the laser and make thresholds
# unstable.
EXPOSURE_US = 3000
ANALOG_GAIN = 24.0

# Direct constant-on spot detection.
#
# Bright thresholds catch white/pale laser spots.  The lower values are useful
# when the spot hits black tape and becomes dimmer.
BRIGHT_THRESHOLDS = (248, 235, 220, 205, 190)

# Lab thresholds for direct colour detection on RGB images.  They are broad on
# purpose: violet/UV modules can appear magenta, red, blue, or almost white
# depending on material and exposure.
LASER_COLOR_THRESHOLDS = (
    # Prefer the actual purple/blue laser colour.  Do not include broad red
    # thresholds here, otherwise the red target rings can become candidates.
    (20, 100, 12, 127, -128, -4),   # magenta / violet
    (20, 100, -25, 95, -128, -8),   # blue / violet / UV bloom
)

MIN_PIXELS = 1
COLOR_MIN_PIXELS = 3
MAX_PIXELS = 260
MAX_SIDE = 32
BRIGHT_MAX_PIXELS = 90
BRIGHT_MAX_SIDE = 18
BRIGHT_MIN_COMPACT = 55
CONTINUITY_GATE_PX = 38
ACQUIRE_FRAMES = 3
ACQUIRE_GATE_PX = 12
HOLD_FRAMES = 5
LOG_EVERY_FRAMES = 5
GC_EVERY_FRAMES = 30


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def blob_ok(blob, kind="B"):
    min_pixels = COLOR_MIN_PIXELS if kind == "C" else MIN_PIXELS
    if blob.pixels() < min_pixels:
        return False
    max_pixels = MAX_PIXELS
    max_side = MAX_SIDE
    if kind == "B":
        # Generic bright blobs include many false positives: white paper glare,
        # bottle highlights, and desk reflections.  Only accept very compact
        # bright blobs as fallback.
        max_pixels = BRIGHT_MAX_PIXELS
        max_side = BRIGHT_MAX_SIDE
    if blob.pixels() > max_pixels:
        return False
    if blob.w() > max_side or blob.h() > max_side:
        return False
    area = max(1, blob.w() * blob.h())
    compact = blob.pixels() * 100 // area
    if kind == "B" and compact < BRIGHT_MIN_COMPACT:
        return False
    return True


def add_candidate(out, blob, kind, thr):
    if not blob_ok(blob, kind):
        return
    # Compactness score.  This penalizes long red ring fragments and wide glare.
    area = max(1, blob.w() * blob.h())
    compact = blob.pixels() * 100 // area
    score = blob.pixels() * 18 + compact * 4 - area
    if kind == "C":
        # Colour is a strong hint, not an absolute override.  The spot can look
        # white/mixed on black tape or background, so bright candidates remain
        # in the same candidate pool.
        score += 900
    out.append({
        "x": blob.cx(),
        "y": blob.cy(),
        "pixels": blob.pixels(),
        "w": blob.w(),
        "h": blob.h(),
        "kind": kind,
        "thr": thr,
        "score": score,
    })


LAST_TOP = []


def detect_spot_direct(img, previous=None):
    global LAST_TOP
    color_candidates = []
    bright_candidates = []

    # 1) Colour blobs on RGB image.  This is the primary signal for your
    # current purple laser spot.
    try:
        blobs = img.find_blobs(LASER_COLOR_THRESHOLDS,
                               pixels_threshold=COLOR_MIN_PIXELS,
                               area_threshold=COLOR_MIN_PIXELS,
                               merge=True)
        for b in blobs:
            add_candidate(color_candidates, b, "C", 0)
    except Exception as e:
        print("@COLOR_ERR,type=%s" % type(e).__name__)

    # 2) Bright grayscale blobs.  This is only a fallback for cases where the
    # camera sees the spot as white instead of purple.  It must not outrank a
    # valid colour candidate.
    try:
        gray = img.to_grayscale()
        for thr in BRIGHT_THRESHOLDS:
            blobs = gray.find_blobs(((thr, 255),),
                                    pixels_threshold=MIN_PIXELS,
                                    area_threshold=MIN_PIXELS,
                                    merge=True)
            for b in blobs:
                add_candidate(bright_candidates, b, "B", thr)
            # Do not immediately break; collecting several thresholds lets the
            # continuity selector keep a dim black-frame spot instead of jumping
            # to a distant bright reflection.
    except Exception as e:
        print("@BRIGHT_ERR,type=%s" % type(e).__name__)

    candidates = color_candidates + bright_candidates
    LAST_TOP = sorted(candidates, key=lambda c: c["score"], reverse=True)[:3]
    if not candidates:
        LAST_TOP = []
        return None, 0
    total_count = len(candidates)

    if previous is not None:
        px, py = previous[0], previous[1]
        gate2 = CONTINUITY_GATE_PX * CONTINUITY_GATE_PX
        near = []
        for c in candidates:
            dx = c["x"] - px
            dy = c["y"] - py
            if dx * dx + dy * dy <= gate2:
                # Near previous point gets a strong bonus.  This is what stops
                # the lock from jumping to bottle/desktop highlights.
                c["score"] += 1200 - (dx * dx + dy * dy) // 2
                near.append(c)
        if near:
            return max(near, key=lambda c: c["score"]), total_count

    return max(candidates, key=lambda c: c["score"]), total_count


class SpotLock:
    def __init__(self):
        self.x = None
        self.y = None
        self.pending_x = None
        self.pending_y = None
        self.pending_n = 0
        self.pixels = 0
        self.kind = "-"
        self.thr = 0
        self.miss = 0
        self.state = "SEARCH"

    def update(self, candidate):
        if candidate is None:
            self.miss += 1
            if self.x is not None and self.miss <= HOLD_FRAMES:
                self.state = "HOLD"
                return True
            self.state = "MISS"
            self.x = None
            self.y = None
            self.pending_x = None
            self.pending_y = None
            self.pending_n = 0
            self.pixels = 0
            self.kind = "-"
            self.thr = 0
            return False

        if self.x is None:
            cx = candidate["x"]
            cy = candidate["y"]
            if self.pending_x is not None:
                dx = cx - self.pending_x
                dy = cy - self.pending_y
                if dx * dx + dy * dy <= ACQUIRE_GATE_PX * ACQUIRE_GATE_PX:
                    self.pending_n += 1
                    self.pending_x = (self.pending_x + cx) // 2
                    self.pending_y = (self.pending_y + cy) // 2
                else:
                    self.pending_n = 1
                    self.pending_x = cx
                    self.pending_y = cy
            else:
                self.pending_n = 1
                self.pending_x = cx
                self.pending_y = cy
            self.pixels = candidate["pixels"]
            self.kind = candidate["kind"]
            self.thr = candidate["thr"]
            self.miss = 0
            if self.pending_n < ACQUIRE_FRAMES:
                self.state = "ACQUIRE"
                return False
            self.x = self.pending_x
            self.y = self.pending_y
        else:
            # Light smoothing only.  Too much smoothing hides lock jitter.
            self.x = (self.x * 2 + candidate["x"] * 3) // 5
            self.y = (self.y * 2 + candidate["y"] * 3) // 5
        self.pixels = candidate["pixels"]
        self.kind = candidate["kind"]
        self.thr = candidate["thr"]
        self.miss = 0
        self.pending_x = None
        self.pending_y = None
        self.pending_n = 0
        self.state = "LOCK"
        return True

    def previous(self):
        if self.x is None:
            return None
        return (self.x, self.y)


def draw_spot(img, lock, frame, candidate_count):
    img.draw_string_advanced(0, 0, 10, "SPOT LOCK ALWAYS-ON", color=(255, 255, 0))
    img.draw_string_advanced(0, 13, 10, "state=%s acq=%d cand=%d miss=%d" %
                             (lock.state, lock.pending_n, candidate_count, lock.miss),
                             color=(255, 255, 0))
    for i, c in enumerate(LAST_TOP):
        color = (0, 255, 255) if c["kind"] == "C" else (255, 180, 0)
        img.draw_circle(c["x"], c["y"], 4 + i * 2, color=color, thickness=1)
        img.draw_string_advanced(clamp(c["x"] + 5, 0, WIDTH - 45),
                                 clamp(c["y"] - 6, 0, HEIGHT - 12),
                                 8, "%d%s%d" % (i + 1, c["kind"], c["pixels"]),
                                 color=color)
    if lock.x is not None:
        color = (255, 0, 255) if lock.state == "LOCK" else (255, 180, 0)
        img.draw_cross(lock.x, lock.y, color=color, size=18)
        img.draw_circle(lock.x, lock.y, 8, color=color, thickness=2)
        img.draw_string_advanced(0, 26, 10, "spot=%d,%d px=%d %s%d" %
                                 (lock.x, lock.y, lock.pixels, lock.kind, lock.thr),
                                 color=color)
    elif lock.pending_x is not None:
        img.draw_cross(lock.pending_x, lock.pending_y, color=(255, 180, 0), size=14)
        img.draw_circle(lock.pending_x, lock.pending_y, 6, color=(255, 180, 0), thickness=2)
        img.draw_string_advanced(0, 26, 10, "acquire=%d/%d at %d,%d" %
                                 (lock.pending_n, ACQUIRE_FRAMES,
                                  lock.pending_x, lock.pending_y),
                                 color=(255, 180, 0))
    else:
        img.draw_string_advanced(0, 26, 10, "No laser spot", color=(255, 0, 0))
    img.draw_string_advanced(0, 39, 10, "frame=%d laser=ON" % frame,
                             color=(0, 255, 255))


def main():
    print("LASER_SPOT_ALWAYS_ON_LOCK_TEST_START")
    print("No servo/PWM. Relay laser stays ON until IDE interrupt.")
    sensor = None
    relay = None
    lock = SpotLock()
    frame = 0
    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
        fpioa = FPIOA()
        fpioa.set_function(RELAY_GPIO, FPIOA.GPIO32)
        relay = Pin(RELAY_GPIO, Pin.OUT, value=1 - RELAY_ACTIVE_LEVEL)

        sensor = Sensor(id=SENSOR_ID, fps=CAMERA_FPS)
        sensor.reset()
        sensor.set_framesize(width=WIDTH, height=HEIGHT, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
        Display.init(Display.VIRT, width=WIDTH, height=HEIGHT, to_ide=True)
        MediaManager.init()
        sensor.run()
        time.sleep_ms(250)
        sensor.auto_exposure(False)
        sensor.exposure(EXPOSURE_US)
        sensor.again(ANALOG_GAIN)
        time.sleep_ms(250)

        relay.value(RELAY_ACTIVE_LEVEL)
        print("@LASER_ON,relay_gpio=%d,active=%d,exposure=%d,gain_x10=%d" %
              (RELAY_GPIO, RELAY_ACTIVE_LEVEL, EXPOSURE_US, int(ANALOG_GAIN * 10)))

        while True:
            os.exitpoint()
            frame += 1
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            cand, cand_count = detect_spot_direct(img, lock.previous())
            had = lock.update(cand)

            if frame % LOG_EVERY_FRAMES == 0 or lock.state != "LOCK":
                if lock.state == "LOCK":
                    print("@SPOT_LOCK,frame=%d,x=%d,y=%d,pixels=%d,kind=%s,thr=%d,candidates=%d" %
                          (frame, lock.x, lock.y, lock.pixels, lock.kind, lock.thr, cand_count))
                elif lock.state == "HOLD":
                    print("@SPOT_HOLD,frame=%d,x=%d,y=%d,miss=%d,candidates=%d" %
                          (frame, lock.x, lock.y, lock.miss, cand_count))
                elif lock.state == "ACQUIRE":
                    print("@SPOT_ACQUIRE,frame=%d,x=%d,y=%d,n=%d/%d,pixels=%d,kind=%s,thr=%d,candidates=%d" %
                          (frame, lock.pending_x, lock.pending_y, lock.pending_n,
                           ACQUIRE_FRAMES, lock.pixels, lock.kind, lock.thr, cand_count))
                else:
                    print("@SPOT_MISS,frame=%d,candidates=%d" % (frame, cand_count))

            draw_spot(img, lock, frame, cand_count)
            Display.show_image(img)
            if frame % GC_EVERY_FRAMES == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("user stop")
    except BaseException as e:
        print("Exception:", type(e).__name__, e)
    finally:
        if relay is not None:
            try:
                relay.value(1 - RELAY_ACTIVE_LEVEL)
                print("@LASER_OFF")
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
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            time.sleep_ms(100)
            MediaManager.deinit()
        except Exception:
            pass


if __name__ == "__main__":
    main()
