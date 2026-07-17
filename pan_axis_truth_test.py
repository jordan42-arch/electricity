"""
K230 CanMV PAN axis truth test.

Only tests the horizontal DS3115 continuous-rotation servo:
  signal -> GPIO52 / PWM4

No camera, no display, no relay, no laser.
Watch the pan servo. It should rotate in one direction, stop, rotate in the
other direction, then stop. The log says which PWM write API is being tested.

Important DS3115 control rules:
  - This PAN servo is a 360-degree continuous-rotation servo, not a position
    servo. A pulse width does not mean "go to this angle".
  - About 1500 us means STOP. The true stop point may be slightly different
    on a real servo, such as 1490 us or 1510 us.
  - Greater than STOP rotates continuously in one direction.
  - Less than STOP rotates continuously in the other direction.
  - The farther away from STOP, the faster it rotates.
  - The longer the pulse is held, the larger the pan angle changes.
  - Closed-loop aiming must use a short speed pulse, then immediately write
    STOP again. Example: 1520 us for 40 ms, then 1500 us.
  - Values like 1900 us / 1100 us for 2 seconds are only for this truth test.
    They are intentionally strong and must not be used directly for aiming.
"""

import time
from machine import FPIOA, PWM


PAN_GPIO = 52
PAN_PWM_CH = 4
PAN_STOP_US = 1500
PAN_FWD_US = 1900
PAN_REV_US = 1100
FREQ_HZ = 50
PERIOD_US = 20000


def us_to_duty16(us):
    return (int(us) * 65535 + PERIOD_US // 2) // PERIOD_US


def us_to_percent(us):
    return (int(us) * 100 + PERIOD_US // 2) // PERIOD_US


def us_to_ns(us):
    return int(us) * 1000


def make_pwm():
    fpioa = FPIOA()
    fpioa.set_function(PAN_GPIO, FPIOA.PWM4)
    pwm = PWM(PAN_PWM_CH)
    try:
        pwm.freq(FREQ_HZ)
    except Exception as e:
        print("@PWM_FREQ_ERR,type=%s,msg=%s" % (type(e).__name__, e))
    return pwm


def stop_all(pwm):
    # Try every API at zero during cleanup. Ignore unsupported methods.
    for name in ("duty_u16", "duty", "duty_ns"):
        try:
            getattr(pwm, name)(0)
        except Exception:
            pass


def run_method(label, writer):
    pwm = make_pwm()
    print("@METHOD_START,%s" % label)
    try:
        writer(pwm, PAN_STOP_US)
        print("@WRITE,%s,stop_us=%d" % (label, PAN_STOP_US))
        time.sleep(1.0)

        writer(pwm, PAN_FWD_US)
        print("@WRITE,%s,forward_us=%d,watch_rotate_2s" % (label, PAN_FWD_US))
        time.sleep(2.0)

        writer(pwm, PAN_STOP_US)
        print("@WRITE,%s,stop_us=%d,watch_stop_1s" % (label, PAN_STOP_US))
        time.sleep(1.0)

        writer(pwm, PAN_REV_US)
        print("@WRITE,%s,reverse_us=%d,watch_rotate_2s" % (label, PAN_REV_US))
        time.sleep(2.0)

        writer(pwm, PAN_STOP_US)
        print("@WRITE,%s,stop_us=%d,watch_stop_1s" % (label, PAN_STOP_US))
        time.sleep(1.0)
    except Exception as e:
        print("@METHOD_ERR,%s,type=%s,msg=%s" % (label, type(e).__name__, e))
    finally:
        try:
            writer(pwm, PAN_STOP_US)
            time.sleep(0.2)
        except Exception:
            pass
        stop_all(pwm)
        try:
            pwm.deinit()
        except Exception:
            pass
        print("@METHOD_DONE,%s" % label)
        time.sleep(1.0)


def write_duty_u16(pwm, us):
    pwm.duty_u16(us_to_duty16(us))


def write_duty_percent(pwm, us):
    pwm.duty(us_to_percent(us))


def write_duty_ns(pwm, us):
    pwm.duty_ns(us_to_ns(us))


def main():
    print("PAN_AXIS_TRUTH_TEST_START")
    print("GPIO52/PWM4 only. DS3115 should visibly rotate for 2 seconds.")
    run_method("duty_u16", write_duty_u16)
    run_method("duty_percent", write_duty_percent)
    run_method("duty_ns", write_duty_ns)
    print("PAN_AXIS_TRUTH_TEST_DONE")


if __name__ == "__main__":
    main()
