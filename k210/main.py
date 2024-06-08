import gc
import json
import math
import os
import time

import KPU as kpu
import Maix
import lcd
import sensor
from Maix import GPIO
from fpioa_manager import fm
from machine import UART

print(os.listdir("/"))

lcd.init()
sensor.reset(freq=20000000)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 如果分辨率大得多，内存就不够用了……
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
uart.write("Hello pico! K210 restart")

f_x = (2.8 / 3.984) * 160  # find_apriltags 如果没有设置，则默认为这个
f_y = (2.8 / 2.952) * 120  # find_apriltags 如果没有设置，则默认为这个
c_x = 160 * 0.5  # find_apriltags 如果没有设置，则默认为这个 (the image.w * 0.5)
c_y = 120 * 0.5  # find_apriltags 如果没有设置，则默认为这个 (the image.h * 0.5)

# kpu
anchors = [2.41, 2.62, 1.06, 1.12, 1.94, 2.0, 1.41, 1.53, 0.59, 0.75]
# model_addr = "/sd/duck_in224.kmodel"
model_addr = 0x300000
labels = ['left', 'right', 'park', 'stop', 'turning']
# labels = ["red_box", "duck"]

threshold_list = {
    "red": [(72, 20, 127, 41, 56, 6)],
    "blue": [(51, 80, -29, 1, -32, -12)],
    "yellow": [(94, 53, -10, 17, 54, 27)],
    "green": [(20, 90, -35, -14, 43, 5)],
}

img_mode = "find_apriltags"
img_mode = "kpu"
# img_mode = "find_blobs"
task = None
task = kpu.load(model_addr)
kpu.init_yolo2(task, 0.5, 0.3, 5, anchors)

find_tag_id = None


def degrees(radians):
    return (180 * radians) / math.pi


def calculate_distance_to_tag(tag_width_pixel):
    focal_length = f_x
    distance = focal_length / tag_width_pixel
    return distance


while True:
    print("stack mem", gc.mem_free() / 1024)  # stack mem
    print("heap mem", Maix.utils.heap_free() / 1024)  # heap mem
    uart_send_data = {}
    clock.tick()
    uart_json = {}
    img = None

    try:
        read_data = uart.read(256)
        if read_data:
            read_str = read_data.decode("utf-8")
            # 查找{和}第一个出现的位置，然后截取字符串
            if "{" in read_str and "}" in read_str:
                read_str = read_str[read_str.index("{"): read_str.index("}") + 1]
                uart_json = json.loads(read_str)
        print("uart read data:", uart_json)
        if "img_mode" in uart_json:
            img_mode = uart_json["img_mode"]
            if img_mode not in ["kpu", "find_apriltags", 'find_blobs']:
                img_mode = "kpu"

            gc.collect()
            img = None
            print("uart img_mode:", img_mode)
            find_tag_id = uart_json.get("find_tag_id", None)

    except Exception as e:
        print("uart read error:", e)
        uart_json = {}
    uart_send_data["find_tag_id"] = find_tag_id
    uart_send_data['img_mode'] = img_mode
    print("current mode: ", img_mode)
    print("find_tag_id: ", find_tag_id)
    if KEY.value() == 0:
        gc.collect()
        if img_mode == "kpu":
            img_mode = "find_apriltags"
        elif img_mode == "find_apriltags":
            img_mode = "find_blobs"
        else:
            img_mode = "kpu"
        time.sleep(1)
    img = sensor.snapshot()
    if img_mode == "find_apriltags":
        apriltags = img.find_apriltags(
            fx=f_x, fy=f_y, cx=c_x, cy=c_y
        )  # defaults to TAG36H11
        uart_send_data["TagStatus"] = "none"

        for tag in apriltags:
            if tag.id() != find_tag_id:
                continue
            img.draw_rectangle(tag.rect(), color=(255, 0, 0))
            img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))

            uart_send_data["TagStatus"] = "get"
            uart_send_data["TagId"] = tag.id()
            uart_send_data["TagTx"] = tag.x_translation()
            uart_send_data["TagTy"] = tag.y_translation()
            uart_send_data["TagTz"] = - tag.z_translation()
            uart_send_data["TagCx"] = tag.cx()
            uart_send_data["TagCy"] = tag.cy()
            uart_send_data["TagId"] = tag.id()

        apriltags = None
    elif img_mode == "find_blobs":
        img = sensor.snapshot()
        if find_tag_id is None or find_tag_id not in ['red', 'blue', 'yellow', 'green']:
            find_tag_id = 'red'
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
            uart_send_data["ObjectStatus"] = "get"
            uart_send_data["ObjectX"] = object_info.x()
            uart_send_data["ObjectY"] = object_info.y()
            uart_send_data["ObjectWidth"] = object_info[2]
            uart_send_data["ObjectHeight"] = object_info[3]
            uart_send_data['ObjectZ'] = dis
            uart_send_data['ObjectId'] = find_tag_id

    else:
        # if task is None:
        #     task = kpu.load(model_addr)
        #     kpu.init_yolo2(task, 0.5, 0.3, 5, anchors)
        # img = sensor.snapshot()
        if find_tag_id is None or find_tag_id not in labels:
            continue
        img_size = 224
        img2 = img.resize(img_size, img_size)
        img2.pix_to_ai()
        objects = kpu.run_yolo2(task, img2)
        del img2
        if objects:
            for obj in objects:
                classid = labels[obj.classid()]
                if classid != find_tag_id and find_tag_id != "get_all_id":
                    continue

                pos = obj.rect()
                pos = (
                    int(pos[0] * img.width() / img_size),
                    int(pos[1] * img.height() / img_size),
                    int(pos[2] * img.width() / img_size),
                    int(pos[3] * img.height() / img_size),
                )
                diag = math.sqrt(pos[2] ** 2 + pos[3] ** 2)
                dis = calculate_distance_to_tag(diag)
                uart_send_data["ObjectStatus"] = "get"
                uart_send_data["ObjectX"] = pos[0]
                uart_send_data["ObjectY"] = pos[1]
                uart_send_data["ObjectWidth"] = pos[2]
                uart_send_data["ObjectHeight"] = pos[3]
                uart_send_data['ObjectId'] = labels[obj.classid()]
                uart_send_data['ObjectZ'] = dis

                img.draw_rectangle(pos, color=(0, 255, 255))

    lcd.display(img)

    uart_send_data_json = json.dumps(uart_send_data)
    print("uart send data", uart_send_data_json)
    uart.write(uart_send_data_json)  # 数据回传
    # print("uart send data",uart_send_data_json)

    # print(clock.fps())
