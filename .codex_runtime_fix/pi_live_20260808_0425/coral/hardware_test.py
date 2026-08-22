#!/usr/bin/env python3
import json
import time

from PIL import Image
from pycoral.adapters import classify, common
from pycoral.utils.edgetpu import list_edge_tpus, make_interpreter


MODEL = "/home/josemsotov/coral_runtime/test_data/model.tflite"
IMAGE = "/home/josemsotov/coral_runtime/test_data/kinect.jpg"


def main():
    devices = list_edge_tpus()
    if not devices:
        raise RuntimeError("No Edge TPU devices detected")

    interpreter = make_interpreter(MODEL)
    interpreter.allocate_tensors()
    size = common.input_size(interpreter)
    image = Image.open(IMAGE).convert("RGB").resize(size)
    common.set_input(interpreter, image)

    timings_ms = []
    for _ in range(6):
        started = time.perf_counter()
        interpreter.invoke()
        timings_ms.append((time.perf_counter() - started) * 1000.0)

    classes = classify.get_classes(interpreter, top_k=3)
    print(json.dumps({
        "ok": True,
        "devices": devices,
        "input_size": [int(value) for value in size],
        "first_inference_ms": round(timings_ms[0], 3),
        "steady_mean_ms": round(sum(timings_ms[1:]) / len(timings_ms[1:]), 3),
        "top_classes": [
            {"id": int(item.id), "score": round(float(item.score), 4)}
            for item in classes
        ],
    }))


if __name__ == "__main__":
    main()
