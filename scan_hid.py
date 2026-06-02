import pywinusb.hid as hid
import time

STADIA_VID = 0x18D1
STADIA_PID = 0x9400

devices = [d for d in hid.find_all_hid_devices()
           if d.vendor_id == STADIA_VID and d.product_id == STADIA_PID]
if not devices:
    print("Stadia no encontrado (VID=0x18D1 PID=0x9400)")
    exit(1)

# Usar el primero que sea game controller (Usage Page 1, Usage 5)
dev = None
for d in devices:
    print(f"  Encontrado: {d.product_name}  path={d.device_path}")
    if dev is None:
        dev = d

print(f"\nAbriendo: {dev.product_name}")
dev.open()

last = None

def handler(data):
    global last
    if data != last:
        print("  RAW:", " ".join(f"{b:02X}" for b in data))
        last = data

dev.set_raw_data_handler(handler)

print("Leyendo datos crudos del Stadia. Mueve el stick y pulsa botones. Ctrl+C para salir.\n")
try:
    while True:
        time.sleep(0.01)
except KeyboardInterrupt:
    pass
finally:
    dev.close()
    print("Cerrado.")
