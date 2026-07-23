#!/usr/bin/env python3
import evdev

for path in evdev.list_devices():
    dev = evdev.InputDevice(path)
    if "stadia" not in dev.name.lower():
        dev.close()
        continue
    print(f"device={path} name={dev.name}")
    for code in (evdev.ecodes.ABS_X, evdev.ecodes.ABS_Y):
        info = dev.absinfo(code)
        print(
            f"code={code} value={info.value} min={info.min} "
            f"max={info.max} flat={info.flat} fuzz={info.fuzz}"
        )
    dev.close()
