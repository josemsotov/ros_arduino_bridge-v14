#!/usr/bin/env python3
import evdev

for path in evdev.list_devices():
    dev = evdev.InputDevice(path)
    if "stadia" not in dev.name.lower():
        dev.close()
        continue
    print(f"device={path} name={dev.name}")
    for code, info in dev.capabilities().get(evdev.ecodes.EV_ABS, []):
        print(
            f"code={code} name={evdev.ecodes.ABS.get(code)} "
            f"value={info.value} min={info.min} "
            f"max={info.max} flat={info.flat} fuzz={info.fuzz}"
        )
    dev.close()
