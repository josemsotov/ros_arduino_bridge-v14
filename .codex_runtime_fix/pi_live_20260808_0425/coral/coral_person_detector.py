#!/usr/bin/env python3
"""Recovering Edge-TPU person detector sidecar for Smart Trolley."""
import io
import json
import os
import time
import urllib.request

from PIL import Image
from pycoral.adapters import common, detect
from pycoral.utils.edgetpu import make_interpreter


MODEL = "/home/josemsotov/coral_runtime/ssd_mobilenet_v2_coco_edgetpu.tflite"
FRAME_URL = "http://127.0.0.1:8080/api/frame/rgb.jpg"
STATE_FILE = "/run/user/1000/coral_person.json"
MIN_SCORE = 0.35
PERIOD_S = 0.20
RECONNECT_S = 5.0


def write_state(payload):
    temporary = STATE_FILE + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, separators=(",", ":"))
    os.replace(temporary, STATE_FILE)


def report_error(stage, exc):
    write_state({
        "ok": False,
        "stamp": time.time(),
        "stage": stage,
        "error": f"{type(exc).__name__}: {exc}",
        "people": [],
    })


def inference_session():
    interpreter = make_interpreter(MODEL)
    interpreter.allocate_tensors()
    input_width, input_height = common.input_size(interpreter)

    while True:
        cycle_started = time.monotonic()
        with urllib.request.urlopen(FRAME_URL, timeout=1.0) as response:
            frame = Image.open(io.BytesIO(response.read())).convert("RGB")
        resized = frame.resize((input_width, input_height))
        common.set_input(interpreter, resized)
        inference_started = time.perf_counter()
        interpreter.invoke()
        inference_ms = (time.perf_counter() - inference_started) * 1000.0

        people = []
        for item in detect.get_objects(interpreter, MIN_SCORE):
            if int(item.id) != 0:
                continue
            box = item.bbox
            people.append({
                "score": round(float(item.score), 4),
                "box": [
                    max(0.0, min(1.0, float(box.xmin) / input_width)),
                    max(0.0, min(1.0, float(box.ymin) / input_height)),
                    max(0.0, min(1.0, float(box.xmax) / input_width)),
                    max(0.0, min(1.0, float(box.ymax) / input_height)),
                ],
            })
        write_state({
            "ok": True,
            "stamp": time.time(),
            "inference_ms": round(inference_ms, 3),
            "people": people,
        })
        remaining = PERIOD_S - (time.monotonic() - cycle_started)
        if remaining > 0:
            time.sleep(remaining)


def main():
    while True:
        try:
            inference_session()
        except Exception as exc:
            report_error("edge_tpu_reconnect", exc)
            time.sleep(RECONNECT_S)


if __name__ == "__main__":
    main()
