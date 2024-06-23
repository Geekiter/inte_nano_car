import os

import lcd
import math
import sensor
import time
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

f_x = (2.8 / 3.984) * 160
f_y = (2.8 / 2.952) * 120
c_x = 160 * 0.5
c_y = 120 * 0.5


def degrees(radians):
    return (180 * radians) / math.pi


real_dis = 31.5  # cm, small
# real_dis = 104.5  # cm, big

while True:
    clock.tick()
    img = sensor.snapshot()

    apriltags = img.find_apriltags(
        fx=f_x, fy=f_y, cx=c_x, cy=c_y
    )  # defaults to TAG36H11
    for tag in apriltags:
        img.draw_rectangle(tag.rect(), color=(255, 0, 0))
        img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))

        dis = tag.z_translation()

        zf = - real_dis / dis
        img.draw_string(0, 0, "zf:%.2f" % zf, color=(255, 0, 0), scale=2)
        print("zf:%.2f" % zf)


    lcd.display(img)
