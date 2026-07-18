"""
K230 CanMV TILT axis truth test for MG996R.

Purpose:
  Confirm the vertical MG996R position servo can move on GPIO42 / PWM0.

Hardware:
  TILT MG996R signal -> GPIO42 / PWM0

Safety:
  No camera, no display, no relay, no laser.
  The test only moves inside a conservative range:
    1500 us -> 1700 us -> 1300 us -> 1500 us

Control rule:
  MG996R is a position servo, unlike the DS3115 pan servo.
  A pulse width means target angle. The servo should move to that angle and
  hold it until another pulse width is written.
"""

import time
from machine import FPIOA, PWM


TILT_GPIO = 42
TILT_PWM_CH = 0
FREQ_HZ = 50
PERIOD_US = 20000

CENTER_US = 1500
UP_TEST_US = 1700
DOWN_TEST_US = 1300
HOLD_MS = 1200


def us_to_duty_u16(us):
    return (int(us) * 65535 + PERIOD_US // 2) // PERIOD_US


def us_to_percent(us):
    return (int(us) * 100 + PERIOD_US // 2) // PERIOD_US


def us_to_ns(us):
    return int(us) * 1000


def make_pwm():
    fpioa = FPIOA()
    fpioa.set_function(TILT_GPIO, FPIOA.PWM0)
    pwm = PWM(TILT_PWM_CH)
    try:
        pwm.freq(FREQ_HZ)
    except Exception as e:
        print("@PWM_FREQ_ERR,type=%s,msg=%s" % (type(e).__name__, e))
    return pwm


def stop_output(pwm):
    for name in ("duty_u16", "duty", "duty_ns"):
        try:
            getattr(pwm, name)(0)
        except Exception:
            pass


def run_method(label, writer):
    pwm = make_pwm()
    print("@METHOD_START,%s" % label)
    try:
        for name, us in (("CENTER", CENTER_US),
                         ("UP_TEST", UP_TEST_US),
                         ("DOWN_TEST", DOWN_TEST_US),
                         ("CENTER", CENTER_US)):
            writer(pwm, us)
            print("@TILT_WRITE,%s,label=%s,us=%d,hold_ms=%d" %
                  (label, name, us, HOLD_MS))
            time.sleep_ms(HOLD_MS)
    except Exception as e:
        print("@METHOD_ERR,%s,type=%s,msg=%s" %
              (label, type(e).__name__, e))
    finally:
        try:
            writer(pwm, CENTER_US)
            time.sleep_ms(300)
        except Exception:
            pass
        stop_output(pwm)
        try:
            pwm.deinit()
        except Exception:
            pass
        print("@METHOD_DONE,%s" % label)
        time.sleep_ms(800)


def write_duty_u16(pwm, us):
    pwm.duty_u16(us_to_duty_u16(us))


def write_duty_percent(pwm, us):
    pwm.duty(us_to_percent(us))


def write_duty_ns(pwm, us):
    pwm.duty_ns(us_to_ns(us))


def main():
    print("TILT_AXIS_TRUTH_TEST_START")
    print("GPIO42/PWM0 only. Watch MG996R tilt movement.")
    run_method("duty_u16", write_duty_u16)
    run_method("duty_percent", write_duty_percent)
    run_method("duty_ns", write_duty_ns)
    print("TILT_AXIS_TRUTH_TEST_DONE")


if __name__ == "__main__":
    main()
