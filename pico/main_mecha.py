from machine import Pin, PWM, ADC
from utime import sleep
import re
import json
from umqtt.simple import MQTTClient
import time
import os
from machine import Timer, Pin, PWM, ADC, UART
import utime
import neopixel
import random
import machine
from utime import sleep
import math

# http server
from micropyserver import MicroPyServer
import utils
from motorPCA9685 import MotorDriver

# # ----------------- fixed parameters -----------------
# ## 初始化UART
uart2 = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17))
inb1 = Pin(10, Pin.OUT)
inb2 = Pin(9, Pin.OUT)
pwmb = PWM(Pin(11))

m = MotorDriver()

# ## arm
ina1 = Pin(8, Pin.OUT)
ina2 = Pin(7, Pin.OUT)
pwma = PWM(Pin(6))
pwma.freq(1000)


is_finished = False


# ## grab claw
maxGrabLevel = 70
minGrabLevel = 30
grabLastState = 2
grabState = 0  # 0 unknown, 1, close, 2, open
currentGrabLevel = 70

# # ----------------- functions -----------------


def parse_data(data):
    values_list = []
    lines = data.split("\n")

    for line in lines:
        if line.strip():  # Check if the line is not just whitespace
            values = {}
            data_items = line.split(",")
            for item in data_items:
                key, value = item.strip().split(":")
                try:
                    values[key] = float(value) if "." in value else int(value)
                except ValueError:
                    values[key] = value
            values_list.append(values)

    return values_list


def RotateBCCW(duty):
    inb1.value(0)
    inb2.value(1)
    duty_16 = int((duty * 65536) / 100)
    pwmb.duty_u16(duty_16)


def RotateBCW(duty):
    inb1.value(1)
    inb2.value(0)
    duty_16 = int((duty * 65536) / 100)
    pwmb.duty_u16(duty_16)


def RotateACW(duty):
    ina1.value(1)
    ina2.value(0)
    duty_16 = int((duty * 65536) / 100)
    print(duty_16)
    pwma.duty_u16(duty_16)


def RotateACCW(duty):
    ina1.value(0)
    ina2.value(1)
    duty_16 = int((duty * 65536) / 100)
    pwma.duty_u16(duty_16)


def StopMotor():
    ina1.value(0)
    ina2.value(0)
    pwma.duty_u16(0)
    inb1.value(0)
    inb2.value(0)
    pwmb.duty_u16(0)


def openClaw():
    global grabLastState
    global grabState
    global currentGrabLevel
    global maxGrabLevel
    global minGrabLevel

    if grabLastState != 1:
        currentGrabLevel = maxGrabLevel

    else:
        currentGrabLevel = max(currentGrabLevel - 10, minGrabLevel)

    print(f"close Claw at {currentGrabLevel}")
    RotateBCCW(currentGrabLevel)
    sleep(0.5)
    StopMotor()
    sleep(0.2)
    grabLastState = 1


def closeClaw():
    global grabLastState
    global grabState
    global currentGrabLevel
    global maxGrabLevel
    global minGrabLevel

    if grabLastState != 2:
        currentGrabLevel = maxGrabLevel

    else:
        currentGrabLevel = max(currentGrabLevel - 10, minGrabLevel)

    print(f"open Claw at {currentGrabLevel}")

    RotateBCW(currentGrabLevel)
    sleep(0.5)
    StopMotor()
    sleep(0.2)
    grabLastState = 2


def moveForwardSpd(speedpct1):
    m.MotorRunInstant("MA", "forward", speedpct1)
    m.MotorRunInstant("MB", "forward", speedpct1)
    m.MotorRunInstant("MC", "forward", speedpct1)
    m.MotorRunInstant("MD", "forward", speedpct1)


def moveBackwardSpd(speedpct1):
    m.MotorRunInstant("MA", "backward", speedpct1)
    m.MotorRunInstant("MB", "backward", speedpct1)
    m.MotorRunInstant("MC", "backward", speedpct1)
    m.MotorRunInstant("MD", "backward", speedpct1)


def rotateLeftSpd(speedpct1):
    m.MotorRunInstant("MA", "backward", speedpct1)
    m.MotorRunInstant("MB", "forward", speedpct1)
    m.MotorRunInstant("MC", "backward", speedpct1)
    m.MotorRunInstant("MD", "forward", speedpct1)


def rotateRightSpd(speedpct1):
    m.MotorRunInstant("MA", "forward", speedpct1)
    m.MotorRunInstant("MB", "backward", speedpct1)
    m.MotorRunInstant("MC", "forward", speedpct1)
    m.MotorRunInstant("MD", "backward", speedpct1)


def stopMove():
    m.MotorStop("MA")
    m.MotorStop("MB")
    m.MotorStop("MC")
    m.MotorStop("MD")


def parallelLeft(speedpct):
    m.MotorRunInstant("MA", "forward", speedpct)
    m.MotorRunInstant("MB", "backward", speedpct)
    m.MotorRunInstant("MC", "backward", speedpct)
    m.MotorRunInstant("MD", "forward", speedpct)


def parallelRight(speedpct):
    m.MotorRunInstant("MA", "backward", speedpct)
    m.MotorRunInstant("MB", "forward", speedpct)
    m.MotorRunInstant("MC", "forward", speedpct)
    m.MotorRunInstant("MD", "backward", speedpct)


def armUp(duty_cycle):
    # arm up
    RotateACW(duty_cycle)
    sleep(0.1)
    StopMotor()
    sleep(0.1)


def armDown(duty_cycle):
    # arm down
    RotateACCW(duty_cycle)
    sleep(0.05)
    StopMotor()
    sleep(0.2)


def keepForward(sp):
    moveForwardSpd(sp)
    moveForwardSpd(sp - 5)
    moveForwardSpd(sp - 10)
    stopMove()


def keepTurnLeft(sp):
    rotateLeftSpd(sp)
    rotateLeftSpd(sp - 5)
    rotateLeftSpd(sp - 8)
    rotateLeftSpd(sp - 10)
    stopMove()


def keepTurnRight(sp):
    rotateRightSpd(sp)
    rotateRightSpd(sp - 5)
    rotateRightSpd(sp - 8)
    rotateRightSpd(sp - 10)
    stopMove()


def keepBackward(sp):
    moveBackwardSpd(sp)
    moveBackwardSpd(sp - 5)
    moveBackwardSpd(sp - 10)
    stopMove()


# # ----------------- adjustable parameters -----------------
"""
action status: 

- locate: locate the object by the big tag.
- grab: grab the object by tag
- finish: finish the task
- grab-by-color: grab the object of the specific color
- put-down: put down the object
- grab-by-kpu: grab the object by kpu
"""
target_action = [
    # {"mode": "find_apriltags", "id": 18, "action": "locate"},
    # {"mode": "find_apriltags", "id": 20, "action": "locate"},
    # {"mode": "kpu", "id": "", "action": "grab-by-kpu"},
    {"mode": "find_blobs", "id": "", "action": "grab-by-color"},
    # {"mode": "find_apriltags", "id": 86, "action": "put-down"},
    # {"mode": "kpu", "id": "", "action": "grab-by-kpu"},
    # {"mode": "find_apriltags", "id": 88, "action": "put-down"},
    # {"mode": "kpu", "id": "", "action": "locate-by-kpu"},
    # {"mode": "find_apriltags", "id": 1, "action": "grab-by-kpu-apriltags"},
    # {"mode": "find_apriltags", "id": 88, "action": "put-down"},
    {"mode": "find_blobs", "id": "", "action": "finished"},
]

big_tag_id_list = [
    18,
    20,
]
small_tag_id_list = [86, 88]

small_tag_zoomfactor = 2.53
big_tag_zoomfactor = 11.33
kpu_tag_zf = 1.42
duck_width_zoomfactor_qqvga = 829.6
duck_height_zoomfactor_qqvga = 829.6


target_img_mode = []
for action in target_action:
    target_img_mode.append(action["mode"])

target_id_list = []
for action in target_action:
    target_id_list.append(action["id"])

target_action_list = []
for action in target_action:
    target_action_list.append(action["action"])


target_index = 0

claw_open_len = 13.5  # cm
claw_close_len = 14

test_mode = False
k210_cam_offset = 85 - 160 / 2  # 相机安装在机械臂上的偏移量

claw_range = (90 - 75) / 2
k210_qqvga = (120, 160)
k210_qvga = (240, 320)
current_resolution = k210_qvga
k210_center = current_resolution[1] / 2 + k210_cam_offset  # QQVGA分辨率：120*160
k210_y_center = current_resolution[0] / 2

arm_range = 15  # pixel 上下浮动范围
rotate_in_front_of_obj = 2  # cm 在物体前方允许旋转的距离
locate_stop_dis = 48  # cm
arm_up_speed = 45
arm_down_speed = 10


def get_zf(id):
    if id in big_tag_id_list:
        return big_tag_zoomfactor
    else:
        return small_tag_zoomfactor


claw_grab_len = 11
claw_arm_up_len = 25  # 大于这个高度，需要抬起机械臂
grab_mode = False
grab_attempted = False
grab_attempted_get_count = 0
grab_attempted_all_count = 0
put_down_obj = False
arm_up_len = 2
search_count = 0
discovered_obj = False
# # ----------------- the life cycle of a job -----------------


# ## main
for _ in range(10):
    armDown(arm_down_speed)

for _ in range(8):
    openClaw()


def close_to_obj_action(cx, cy, claw_range_level=1.5):
    if cy < k210_y_center - arm_range:
        print("view is low, arm up")
        armUp(arm_up_speed)
        sleep(0.3)
    elif cy > k210_y_center + arm_range:
        print("view is high, arm down")
        armDown(arm_down_speed)
        sleep(0.3)
    elif cx > k210_center + claw_range_level * claw_range:
        print("right")
        keepTurnRight(45)
    # 如果cx小于k210_center - claw_range，说明物体在左边，左转
    elif cx < k210_center - claw_range_level * claw_range:
        print("left")
        keepTurnLeft(45)
    else:
        print("forward")
        keepForward(45)
        sleep(0.1)


def get_locate_action(tag_x, tag_y, tag_z):
    global target_index
    obj_dis = tag_z
    print(f"tag_x: {tag_x}, tag_y: {tag_y}, tag_z: {tag_z}")
    print(f"obj_dis: {obj_dis}")
    if obj_dis > locate_stop_dis:
        close_to_obj_action(tag_x, tag_y)
    else:
        for _ in range(16):
            armDown(arm_down_speed)
            sleep(0.1)

        print("have located the object")
        next_target()


def put_down_transition():
    for _ in range(10):
        keepBackward(30)

    for _ in range(16):
        armDown(arm_down_speed)
        sleep(0.3)

    for _ in range(4):
        closeClaw()


def grab_transition():
    for _ in range(12):
        armUp(arm_up_speed)
    for _ in range(4):
        keepBackward(30)


def put_down_action(tag_x, tag_y, tag_z):
    global put_down_obj
    global target_index
    if tag_z > 1.2 * (claw_close_len + arm_up_len):
        close_to_obj_action(tag_x, tag_y)
    else:
        for _ in range(3):
            armUp(arm_up_speed)
            sleep(0.3)

        for _ in range(4):
            openClaw()

        put_down_transition()

        next_target()


def kpu_locate_action(x, y, w, h):
    global grab_mode
    global target_index
    cx = x + w / 2
    cy = y + h / 2
    dis_w = duck_width_zoomfactor_qqvga / w
    dis_h = duck_height_zoomfactor_qqvga / h
    print(f"dis_h: {dis_h}")
    print(f"dis_w: {dis_w}")
    obj_dis = max(dis_w, dis_h)
    print(f"obj_dis: {obj_dis}")
    if obj_dis > rotate_in_front_of_obj + claw_open_len:
        close_to_obj_action(cx, cy, claw_range_level=1.5)
    else:
        for _ in range(6):
            armDown(arm_down_speed)
        next_target()


def get_kpu_tag_action(tag_x, tag_y, tag_z):
    print(f"tag_x: {tag_x}, tag_y: {tag_y}, tag_z: {tag_z}")
    global grab_mode
    global is_grabbed
    global target_index
    if tag_x > k210_center + claw_range:
        print("right")
        keepTurnRight(30)
    elif tag_x < k210_center - claw_range:
        print("left")
        keepTurnLeft(30)
    else:
        print("grab mode")
        grab_mode = True

        for _ in range(4):
            armUp(25)



def grab_by_kpu(cx, cy, dis):
    global grab_mode
    global is_grabbed
    if dis > (rotate_in_front_of_obj + claw_open_len):
        close_to_obj_action(cx, cy, claw_range_level=1.5)
    else:
        if cy < k210_y_center:
            armUp(arm_up_speed)
        else:
            grab_mode = True


def grab_mode_in_kpu():
    global grab_forward_count
    for _ in range(1):
        keepBackward(30)
    for _ in range(10):
        armUp(25)
    for _ in range(grab_forward_count):
        keepForward(25)
    for _ in range(12):
        closeClaw()

    for _ in range(6):
        armUp(arm_up_speed)

    for _ in range(2 * (grab_forward_count - 1)):
        keepBackward(25)

    for _ in range(14):
        armDown(arm_down_speed)


def grab_mode_in_color():
    global grab_forward_count
    for _ in range(1):
        keepBackward(30)
    # for _ in range(10):
    #     armUp(25)
    for _ in range(grab_forward_count):
        keepForward(25)
    for _ in range(12):
        closeClaw()

    for _ in range(6):
        armUp(arm_up_speed)

    for _ in range(2 * (grab_forward_count - 1)):
        keepBackward(25)

    for _ in range(14):
        armDown(arm_down_speed)


def grab_by_color(cx, cy, dis):
    global grab_mode
    global is_grabbed
    if dis > (rotate_in_front_of_obj + claw_open_len) and not grab_mode:
        close_to_obj_action(cx, cy, claw_range_level=1.5)
    else:
        grab_mode = True
        if cy > claw_arm_up_len:
            armUp(25)
            sleep(1)
            print("arm up, and h is: ", cy)
        else:
            keepForward(25)


def get_json(uart_data):
    try:
        uart_read = uart_data.read()
        uart_data = uart_read.decode("utf-8")
        # find { and } first appear position, then cut the string
        if "{" in uart_data and "}" in uart_data:
            uart_data = uart_data[uart_data.index("{") : uart_data.index("}") + 1]
            return json.loads(uart_data)
        else:
            return {}
    except Exception as e:
        print("get json error, original data: ", e, uart_read)
        return {}


def discovered_obj_action():
    global search_count
    print("current search count: {}".format(search_count))
    if search_count < 10:
        keepTurnRight(45)
        search_count += 1
    elif 10 <= search_count < 20:
        keepTurnLeft(45)
        search_count += 1
    else:
        keepTurnRight(45)
    sleep(0.3)


def get_search_count(cx, resolution_x):
    resolution_x_unit = resolution_x / 20
    sc = int(cx / resolution_x_unit)
    if sc < 10:
        return 20 - sc
    else:
        return sc - 10


def next_target():
    global target_index
    global discovered_obj
    target_index += 1
    discovered_obj = False


grab_forward_count = 8
grab_forward_count_origin = 8
wait_ct = 0
wait_ct_limit = 10
# 核心逻辑
while is_finished is False and not test_mode:
    if not uart2.any():
        continue
    if target_index < len(target_action_list):
        print(f"current target action is: {target_action_list[target_index]}")
    else:
        print("target action list is empty")
        break

    data = get_json(uart2)

    print(f"current data: {data}")

    k210_img_mode = data.get("img_mode", "N/A")
    find_tag_id = data.get("find_tag_id", None)

    tag_status = "none"
    obj_status = "none"
    if target_action[target_index]["id"] != "":
        tag_id = int(data.get("TagId", "999"))
        tag_status = data.get("TagStatus", "none")
        zoomfactor = get_zf(tag_id)
        tag_z = int(-zoomfactor * float(data.get("TagTz", "999")))
        tag_x = int(zoomfactor * float(data.get("TagTx", "999")))
        tag_y = int(-zoomfactor * float(data.get("TagTy", "999")))
        tag_cx = int(data.get("TagCx", "999"))
        tag_cy = int(data.get("TagCy", "999"))
    else:
        obj_w = data.get("ObjectWidth", 0)
        obj_h = data.get("ObjectHeight", 0)
        obj_x = data.get("ObjectX", 0)
        obj_y = data.get("ObjectY", 0)
        obj_status = data.get("ObjectStatus", "none")

    if k210_img_mode != target_img_mode[target_index]:
        uart_write_dict = {"img_mode": target_img_mode[target_index]}
        uart2.write(json.dumps(uart_write_dict) + "\n")

    # if target_img_mode[target_index] == "find-apriltags" and find_tag_id != target_id_list[target_index]:
    #     uart_write_dict = {"find_tag_id": target_id_list[target_index]}
    #     uart2.write(json.dumps(uart_write_dict) + "\n")
    if tag_status is "get":
        search_count = get_search_count(tag_cx, current_resolution[1])
        discovered_obj = True
    elif obj_status is "get":
        search_count = get_search_count(obj_x, current_resolution[1])
        discovered_obj = True

    if target_action_list[target_index] == "finished":
        is_finished = True
        break
    elif grab_mode and obj_status != "get":
        if target_action_list[target_index] in ["grab-by-kpu", "grab-by-kpu-apriltags"]:
            grab_mode_in_kpu()
        elif target_action_list[target_index] == "grab-by-color":
            grab_mode_in_color()

        # grab_transition()

        grab_mode = False
        grab_attempted = True
        obj_status = "none"
        sleep(2)

    elif grab_attempted:
        if data == {}:
            continue
        print(
            "all count:{}, get count:{}".format(
                grab_attempted_all_count, grab_attempted_get_count
            )
        )
        if obj_status == "get":
            grab_attempted_get_count += 1
            grab_attempted_all_count += 1
        elif obj_status == "none":
            grab_attempted_all_count += 1
        if grab_attempted_all_count > 10:
            if grab_attempted_get_count >= 3:
                grab_forward_count += 1
                grab_mode = False
                grab_attempted = False
                grab_attempted_get_count = 0
                grab_attempted_all_count = 0
                for _ in range(10):
                    openClaw()
            else:
                grab_transition()
                grab_forward_count = grab_forward_count_origin
                grab_attempted = False
                next_target()

    elif (tag_status == "none" and obj_status == "none") or (
        tag_status == "get" and tag_id != target_id_list[target_index]
    ):
        if discovered_obj:
            # discovered_obj_action()
            print(f"wait ct={wait_ct}")
            wait_ct += 1
            if wait_ct >= wait_ct_limit:
                discovered_obj = False
                wait_ct = 0
        else:
            keepTurnRight(30)
            sleep(0.3)
    elif target_action_list[target_index] == "put-down":
        if tag_id == target_id_list[target_index]:
            put_down_action(tag_cx, tag_cy, tag_z)
    elif target_action_list[target_index] == "grab-by-kpu":
        print("obj_w: ", obj_w)
        obj_dis = duck_width_zoomfactor_qqvga / obj_w
        grab_by_kpu(obj_x, obj_y, obj_dis)
    elif target_action_list[target_index] == "grab-by-color":
        obj_dis = duck_width_zoomfactor_qqvga / obj_w
        print(f"obj_dis: {obj_dis}")
        grab_by_color(obj_x, obj_y, obj_dis)
    elif target_action_list[target_index] == "locate":
        if tag_id == target_id_list[target_index]:
            get_locate_action(tag_cx, tag_cy, tag_z)
    elif target_action_list[target_index] == "grab-by-kpu-apriltags":
        get_kpu_tag_action(tag_cx, tag_cy, tag_z)
    elif target_action_list[target_index] == "locate-by-kpu":
        kpu_locate_action(obj_x, obj_y, obj_w, obj_h)

