import math
import os
import time

import lcd
import sensor
from Maix import GPIO
from fpioa_manager import fm
from machine import UART

print(os.listdir("/"))

real_dis = 14.2  # cm
find_tag_id = "red"

lcd.init()
sensor.reset(freq=20000000)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_vflip(1)
clock = time.clock()

# 映射串口引脚
fm.register(6, fm.fpioa.UART1_RX, force=True)
fm.register(7, fm.fpioa.UART1_TX, force=True)
fm.register(16, fm.fpioa.GPIO1)
KEY = GPIO(GPIO.GPIO1, GPIO.IN)
# 初始化串口
uart = UART(UART.UART1, 115200, read_buf_len=4096)
uart.write("Hello pico! K210 start")
# kpu
task = None
f_x = (2.8 / 3.984) * 160
f_y = (2.8 / 2.952) * 120
c_x = 160 * 0.5
c_y = 120 * 0.5
threshold_list = {
    "red": [(72, 20, 127, 41, 56, 6)],
    "blue": [(51, 80, -29, 1, -32, -12)],
    "yellow": [(94, 53, -10, 17, 54, 27)],
    "green": [(20, 90, -35, -14, 43, 5)],
}


def calculate_distance_to_tag(tag_width_pixel):
    focal_length = f_x
    distance = focal_length / tag_width_pixel
    return distance


while True:
    img = sensor.snapshot()

    current_threshold = threshold_list[find_tag_id]
    blobs = img.find_blobs(current_threshold, pixels_threshold=100)
    if blobs:
        max_pixel = 0
        b_index = 0
        for b in blobs:
            if b[4] > max_pixel:
                max_pixel = b_index
        object_info = blobs[b_index]
        img.draw_rectangle(blobs[b_index][0:4])
        img.draw_cross(
            blobs[b_index].x() + int(object_info[2] / 2),
            blobs[b_index].y() + int(object_info[3] / 2),
        )
        diag = math.sqrt(object_info[2] ** 2 + object_info[3] ** 2)
        dis = calculate_distance_to_tag(diag)
        zf = real_dis / dis
        img.draw_string(0, 0, "zf:%.2f" % zf, color=(255, 0, 0), scale=2)
        print("zf:%.2f" % zf)
    lcd.display(img)
