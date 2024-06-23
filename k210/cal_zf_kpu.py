import math
import os
import time
import KPU as kpu
import lcd
import sensor
from Maix import GPIO
from fpioa_manager import fm
from machine import UART

print(os.listdir("/"))

lcd.init()
sensor.reset(freq=20000000)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_vflip(1)
clock = time.clock()

fm.register(6, fm.fpioa.UART1_RX, force=True)
fm.register(7, fm.fpioa.UART1_TX, force=True)
fm.register(16, fm.fpioa.GPIO1)
KEY = GPIO(GPIO.GPIO1, GPIO.IN)
uart = UART(UART.UART1, 115200, read_buf_len=4096)
uart.write("Hello pico! K210 start")
anchors = [2.41, 2.62, 1.06, 1.12, 1.94, 2.0, 1.41, 1.53, 0.59, 0.75]
labels = ["red_box", "duck"]
model_addr = "/sd/duck_in224.kmodel"
task = None
task = kpu.load(model_addr)
kpu.init_yolo2(task, 0.5, 0.3, 5, anchors)
f_x = (2.8 / 3.984) * 160
f_y = (2.8 / 2.952) * 120
c_x = 160 * 0.5
c_y = 120 * 0.5

real_dis = 14.2  # cm


def calculate_distance_to_tag(tag_width_pixel):
    focal_length = f_x
    distance = focal_length / tag_width_pixel
    return distance


threshold_list = {
    "red": [(72, 20, 127, 41, 56, 6)],
    "blue": [(51, 80, -29, 1, -32, -12)],
    "yellow": [(94, 53, -10, 17, 54, 27)],
    "green": [(20, 90, -35, -14, 43, 5)],
}

while True:
    # print("stack mem", gc.mem_free() / 1024) # stack mem
    # print("heap mem", Maix.utils.heap_free() / 1024) # heap mem

    img = sensor.snapshot()
    img_size = 224
    img2 = img.resize(img_size, img_size)
    img2.pix_to_ai()
    objects = kpu.run_yolo2(task, img2)
    del img2

    if objects:
        for obj in objects:
            if labels[obj.classid()] == "duck":
                pos = obj.rect()
                pos = (
                    int(pos[0] * img.width() / img_size),
                    int(pos[1] * img.height() / img_size),
                    int(pos[2] * img.width() / img_size),
                    int(pos[3] * img.height() / img_size),
                )

                img.draw_rectangle(pos, color=(0, 255, 255))
                diag = math.sqrt(pos[2] ** 2 + pos[3] ** 2)
                dis = calculate_distance_to_tag(diag)
                zf = real_dis / dis
                img.draw_string(0, 0, "zf:%.2f" % zf, color=(255, 0, 0), scale=2)
                print("zf:%.2f" % zf)
    lcd.display(img)
