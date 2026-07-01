# K230 Vision UART Protocol

## K230 script

Upload `main.py` to the K230 first. Recommended remote path:

```text
/sdcard/k230_target.py
```

Then run this on the K230:

```python
exec(open("/sdcard/k230_target.py").read())
```

The script only performs vision detection and UART output. It does not control
servos or laser.

## UART wiring

- K230 UART2 TX pin: 11
- K230 UART2 RX pin: 12
- Baud rate: 115200
- Format: 8 data bits, no parity, 1 stop bit

Wire:

- K230 TX -> TI RX
- K230 RX -> TI TX, optional for now
- K230 GND -> TI GND

Use 3.3 V TTL logic. Do not connect motor battery voltage to signal pins.

## Packet

The K230 sends one text line about every 50 ms:

```text
@V,valid,dx,dy,confidence,label\n
```

Example:

```text
@V,1,-10,10,16800,WHITE_TARGET
```

Fields:

- `valid`: `1` means target detected; `0` means no reliable target.
- `dx`: target center x minus image center x, in pixels.
- `dy`: target center y minus image center y, in pixels.
- `confidence`: detection strength, currently white target pixel count or circle score.
- `label`: detection method, usually `WHITE_TARGET`.

Control sign convention:

- `dx > 0`: target is to the right of image center.
- `dx < 0`: target is to the left of image center.
- `dy > 0`: target is below image center.
- `dy < 0`: target is above image center.
