# Replay captures

`ugv02_feedback.jsonl` is a **synthetic** capture, generated to the protocol
documented in `HARDWARE_HAL_PLAN.md` rather than recorded from a robot: 3 s of
feedback at 80 Hz - 1 s stationary, 1 s at 0.30 m/s forward, 1 s pivoting left at
1 rad/s - with 1 g on the accelerometer's Z axis throughout. Median line length
is 134 bytes, matching the ~140 the bandwidth arithmetic assumes.

It exercises framing, parsing, unit conversion and monotonic encoder
accumulation with no hardware, which is all it is meant to do. It cannot catch a
firmware quirk nobody has seen yet - field ordering, wrap behaviour on `odl`/`odr`,
whatever `L` and `R` really are.

**Replace it with a real capture during on-robot bring-up** (phase 5):

```bash
python3 -c "import serial,sys; s=serial.Serial('/dev/ttyAMA0',115200,timeout=1)
[sys.stdout.write(s.readline().decode('utf-8','replace')) for _ in range(2000)]" \
  > ugv02_feedback.jsonl
```

The tests read whatever is in this file, so a real capture drops straight in.
