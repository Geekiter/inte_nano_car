# In this code,
# it tests the following functions:
# 1. main function: line follow
# 2. RGB using regular pin for power
# 3/ battery voltage monitor
# 4. K210 serial port data transmission
# 5. IR remote controller
# 6. K210 go to a big april tag and grab a small tag
# 7. test music

from machine import Pin, PWM, ADC
from utime import sleep

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
# http server
from micropyserver import MicroPyServer
import utils
from motorPCA9685 import MotorDriver
import network
import socket

import re

# WIFI
wifiName = "WOW10086"
wifiPassword = "chenkeyan"

# MQTT setting
myTopic = "nano2"
clientID = "nano2"

serverIP = "192.168.31.92"
port = 1883

nano_ip = ''
nano_port = 0

try:
    with open('nano_ip.txt', 'r') as f:
        content = f.read()
        data = json.loads(content)
        print(f"nano_ip.txt: {data}")

        if data.get("nanoIp", None) is not None:
            nano_ip = data.get("nanoIp", None)
            nano_port = data.get("nanoProt", None)
            print('Got nano ip address:' + nano_ip + ':' + str(nano_port))
except OSError:
    # 文件不存在
    print('文件不存在')

# your device name
machineId = 'nano2'
state_value = 0

# 0, move left, 1, move right, 2, forward, 3 stop, 4 backward
currentDirection = 3
armUpSpd = 35
wheelSpd = 45
# LED
# Led_R = PWM(Pin(20))
# Led_G = PWM(Pin(19))
# Led_B = PWM(Pin(18))

# Led_R = PWM(Pin(19))
# Led_G = PWM(Pin(18))
# Led_B = PWM(Pin(20))

Led_R = PWM(Pin(18))
Led_G = PWM(Pin(14))
Led_B = PWM(Pin(19))
Led_E = Pin(22, Pin.OUT)  # PWM(Pin(22))

led_r1 = Pin(12, Pin.OUT)
led_r1.value(0)

# Initialize ADC
adc = ADC(Pin(27))  # Assuming you're using GPIO 26 for ADC
resistance_ratio = 0.3333
ADC_MAX = 4095  # Maximum ADC value for 12-bit ADC
V_REF = 3.3  # Reference voltage for the Pico's ADC
voltage_limit = 7.6

# infrared remote
IR_PIN = Pin(3, Pin.IN, Pin.PULL_UP)

grabState = 0  # 0 unknown, 1, close, 2, open
grabLastState = 0
maxGrabLevel = 70
minGrabLevel = 30
currentGrabLevel = 70
# exec_cmd
N = 0

# define for arm and claw control

# ina1 = Pin(7,Pin.OUT)
# ina2 = Pin(8, Pin.OUT)
ina1 = Pin(8, Pin.OUT)
ina2 = Pin(7, Pin.OUT)
pwma = PWM(Pin(6))
pwma.freq(1000)

# inb1 = Pin(9,Pin.OUT)
# inb2 = Pin(10, Pin.OUT)
inb1 = Pin(10, Pin.OUT)
inb2 = Pin(9, Pin.OUT)
pwmb = PWM(Pin(11))

pwmb.freq(1000)

timestamp = int(time.time())
filename1 = f"data_log_{timestamp}.csv"

# Record the start time
start_time = utime.ticks_ms()

TrackingPin_C = 15  # IR tracking sensor(Center)
TrackingPin_R = 1  # IR tracking sensor(right)
TrackingPin_L = 2  # IR tracking sensor(left)


def send_message_to_server(message):
    global nano_ip, nano_port
    if nano_ip == '' or nano_port == 0:
        return
    # 创建一个socket对象
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        # 连接到服务器
        client_socket.connect((nano_ip, nano_port))

        # 发送数据
        client_socket.sendall(message.encode('utf-8'))

        # 可以在这里接收服务器的回复，如果服务器有回复的话
        # response = client_socket.recv(1024)
        # print("收到服务器的回应：", response.decode('utf-8'))
        print("sent message to server")
    except:
        print("连接错误")
    finally:
        # 关闭连接
        client_socket.close()


def track_line_setup():
    global Track_L
    global Track_R
    global Track_C
    Track_L = Pin(TrackingPin_L, Pin.IN, Pin.PULL_UP)
    Track_R = Pin(TrackingPin_R, Pin.IN, Pin.PULL_UP)
    Track_C = Pin(TrackingPin_C, Pin.IN, Pin.PULL_UP)


def log_data(data, filename=filename1, is_header=False):
    # # If filename is not provided, create one with the current timestamp
    # if filename is None:
    #     timestamp = int(time.time())
    #     filename = f"data_log_{timestamp}.csv"

    # Check if the file exists to determine if we need to write the header
    # file_exists = os.path.isfile(filename)

    with open(filename, 'a') as file:
        # If the file does not exist and is_header is True, write the header
        if is_header:
            header = ','.join(data) + '\n'
            file.write(header)
        else:
            # Write the data
            line = ','.join(str(item) for item in data) + '\n'
            file.write(line)


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


def RotateBCW(duty):
    inb1.value(1)
    inb2.value(0)
    duty_16 = int((duty * 65536) / 100)
    pwmb.duty_u16(duty_16)


def RotateBCCW(duty):
    inb1.value(0)
    inb2.value(1)
    duty_16 = int((duty * 65536) / 100)
    pwmb.duty_u16(duty_16)


def StopMotor():
    ina1.value(0)
    ina2.value(0)
    pwma.duty_u16(0)
    inb1.value(0)
    inb2.value(0)
    pwmb.duty_u16(0)


# motor
# motor1 = PWM(Pin(10))
# motor2 = PWM(Pin(11))
# motor3 = PWM(Pin(12))
# motor4 = PWM(Pin(13))
speed = 65534
speedpct = 30

# voice
buzzer = PWM(Pin(0))
Tone = [0, 392, 440, 494, 523, 587, 659, 698, 784]

Song = [Tone[3], Tone[3], Tone[3], Tone[3], Tone[3], Tone[3],
        Tone[3], Tone[5], Tone[1], Tone[2], Tone[3],
        Tone[4], Tone[4], Tone[4], Tone[4], Tone[4], Tone[3], Tone[3], Tone[3],
        Tone[5], Tone[5], Tone[4], Tone[2], Tone[1], Tone[8]]

Beat = [1, 1, 2, 1, 1, 2,
        1, 1, 1.5, 0.5, 4,
        1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 2, 2]

Song2 = [Tone[3], Tone[3]]
Beat2 = [1, 1]

# MQTT
mqtt_client = 1
run = False


def RGB(R, G, B, E):
    if E == 1:
        if R < 0:
            R = random.randint(0, 65535)
        elif R >= 65535:
            R = 65534
        if G < 0:
            G = random.randint(0, 65535)
        elif G >= 65535:
            G = 65534
        if B < 0:
            B = random.randint(0, 65535)
        elif B >= 65535:
            B = 65534
        # print(R,G,B)
        Led_R.duty_u16(65534 - R)
        Led_G.duty_u16(65534 - G)
        Led_B.duty_u16(65534 - B)
        Led_E.value(1)
    else:
        Led_R.duty_u16(0)
        Led_G.duty_u16(0)
        Led_B.duty_u16(0)
        Led_E.value(0)


def setup():
    global m
    m = MotorDriver()


def moveForward():
    m.MotorRunInstant('MA', 'forward', speedpct)
    m.MotorRunInstant('MB', 'forward', speedpct)
    m.MotorRunInstant('MC', 'forward', speedpct)
    m.MotorRunInstant('MD', 'forward', speedpct)


def moveBackward():
    m.MotorRunInstant('MA', 'backward', speedpct)
    m.MotorRunInstant('MB', 'backward', speedpct)
    m.MotorRunInstant('MC', 'backward', speedpct)
    m.MotorRunInstant('MD', 'backward', speedpct)


def moveSmoothForward(desired_duty_cycle):
    global start_time
    global currentDirection
    time2 = utime.ticks_ms()
    elapsed_time = utime.ticks_diff(time2, start_time)
    # still keep the same direction and command is within last 500 ms
    if elapsed_time < 1000 and currentDirection == 2:
        print('forward 1')
        speedpct1 = desired_duty_cycle
        m.MotorRunInstant('MA', 'forward', speedpct1)
        m.MotorRunInstant('MB', 'forward', speedpct1)
        m.MotorRunInstant('MC', 'forward', speedpct1)
        m.MotorRunInstant('MD', 'forward', speedpct1)
    else:
        print('forward 2')
        speedpct1 = 0
        while speedpct1 < desired_duty_cycle:
            speedpct1 += 5  # Increment by 1% or a suitable value
            m.MotorRunInstant('MA', 'forward', speedpct1)
            m.MotorRunInstant('MB', 'forward', speedpct1)
            m.MotorRunInstant('MC', 'forward', speedpct1)
            m.MotorRunInstant('MD', 'forward', speedpct1)
            utime.sleep(0.01)

    currentDirection = 2
    start_time = time2


def moveSmoothBackward(desired_duty_cycle):
    global start_time
    global currentDirection
    time2 = utime.ticks_ms()
    elapsed_time = utime.ticks_diff(time2, start_time)
    print(f'elapsed_time={elapsed_time}, currentDirection={currentDirection}')
    # still keep the same direction and command is within last 500 ms
    if elapsed_time < 1000 and currentDirection == 4:
        print('backward 1')
        speedpct1 = desired_duty_cycle
        m.MotorRunInstant('MA', 'backward', speedpct1)
        m.MotorRunInstant('MB', 'backward', speedpct1)
        m.MotorRunInstant('MC', 'backward', speedpct1)
        m.MotorRunInstant('MD', 'backward', speedpct1)
    else:
        print('backward 2')
        speedpct1 = 0
        while speedpct1 < desired_duty_cycle:
            speedpct1 += 5  # Increment by 1% or a suitable value
            m.MotorRunInstant('MA', 'backward', speedpct1)
            m.MotorRunInstant('MB', 'backward', speedpct1)
            m.MotorRunInstant('MC', 'backward', speedpct1)
            m.MotorRunInstant('MD', 'backward', speedpct1)
            utime.sleep(0.01)

    currentDirection = 4
    start_time = time2


def moveForwardSpd(speedpct1):
    m.MotorRunInstant('MA', 'forward', speedpct1)
    m.MotorRunInstant('MB', 'forward', speedpct1)
    m.MotorRunInstant('MC', 'forward', speedpct1)
    m.MotorRunInstant('MD', 'forward', speedpct1)


def moveBackwardSpd(speedpct1):
    m.MotorRunInstant('MA', 'backward', speedpct1)
    m.MotorRunInstant('MB', 'backward', speedpct1)
    m.MotorRunInstant('MC', 'backward', speedpct1)
    m.MotorRunInstant('MD', 'backward', speedpct1)


def stopMove():
    m.MotorStop('MA')
    m.MotorStop('MB')
    m.MotorStop('MC')
    m.MotorStop('MD')


def rotateLeft():
    m.MotorRunInstant('MA', 'backward', speedpct)
    m.MotorRunInstant('MB', 'forward', speedpct)
    m.MotorRunInstant('MC', 'backward', speedpct)
    m.MotorRunInstant('MD', 'forward', speedpct)


#     motor(0,speed3,speed4,0)
#     motor2(0,speed3,speed4,0)


def rotateRight():
    m.MotorRunInstant('MA', 'forward', speedpct)
    m.MotorRunInstant('MB', 'backward', speedpct)
    m.MotorRunInstant('MC', 'forward', speedpct)
    m.MotorRunInstant('MD', 'backward', speedpct)


def rotateSmoothLeft(desired_duty_cycle):
    global start_time
    global currentDirection
    time2 = utime.ticks_ms()
    elapsed_time = utime.ticks_diff(time2, start_time)
    # still keep the same direction and command is within last 500 ms
    if elapsed_time < 1000 and currentDirection == 0:
        print('left 1')
        speedpct1 = desired_duty_cycle
        m.MotorRunInstant('MA', 'backward', speedpct1)
        m.MotorRunInstant('MB', 'forward', speedpct1)
        m.MotorRunInstant('MC', 'backward', speedpct1)
        m.MotorRunInstant('MD', 'forward', speedpct1)
    else:
        print('left 2')
        speedpct1 = 0
        while speedpct1 < desired_duty_cycle:
            speedpct1 += 5  # Increment by 1% or a suitable value
            m.MotorRunInstant('MA', 'backward', speedpct1)
            m.MotorRunInstant('MB', 'forward', speedpct1)
            m.MotorRunInstant('MC', 'backward', speedpct1)
            m.MotorRunInstant('MD', 'forward', speedpct1)
            utime.sleep(0.01)
    currentDirection = 0
    start_time = time2


def rotateSmoothRight(desired_duty_cycle):
    global start_time
    global currentDirection
    time2 = utime.ticks_ms()
    elapsed_time = utime.ticks_diff(time2, start_time)
    # still keep the same direction and command is within last 500 ms
    if elapsed_time < 1000 and currentDirection == 1:
        print('right 1')
        speedpct1 = desired_duty_cycle
        m.MotorRunInstant('MA', 'forward', speedpct1)
        m.MotorRunInstant('MB', 'backward', speedpct1)
        m.MotorRunInstant('MC', 'forward', speedpct1)
        m.MotorRunInstant('MD', 'backward', speedpct1)
    else:
        print('right 2')
        speedpct1 = 0
        while speedpct1 < desired_duty_cycle:
            speedpct1 += 5  # Increment by 1% or a suitable value
            m.MotorRunInstant('MA', 'forward', speedpct1)
            m.MotorRunInstant('MB', 'backward', speedpct1)
            m.MotorRunInstant('MC', 'forward', speedpct1)
            m.MotorRunInstant('MD', 'backward', speedpct1)
            utime.sleep(0.01)
    currentDirection = 1
    start_time = time2


def rotateRightSpd(speedpct1):
    m.MotorRunInstant('MA', 'forward', speedpct1)
    m.MotorRunInstant('MB', 'backward', speedpct1)
    m.MotorRunInstant('MC', 'forward', speedpct1)
    m.MotorRunInstant('MD', 'backward', speedpct1)


def rotateLeftSpd(speedpct1):
    m.MotorRunInstant('MA', 'backward', speedpct1)
    m.MotorRunInstant('MB', 'forward', speedpct1)
    m.MotorRunInstant('MC', 'backward', speedpct1)
    m.MotorRunInstant('MD', 'forward', speedpct1)


def parallelLeft():
    m.MotorRunInstant('MA', 'forward', speedpct)
    m.MotorRunInstant('MB', 'backward', speedpct)
    m.MotorRunInstant('MC', 'backward', speedpct)
    m.MotorRunInstant('MD', 'forward', speedpct)


#     motor(0,speed3,speed4,0)
#     motor2(0,speed3,speed4,0)


def parallelRight():
    m.MotorRunInstant('MA', 'backward', speedpct)
    m.MotorRunInstant('MB', 'forward', speedpct)
    m.MotorRunInstant('MC', 'forward', speedpct)
    m.MotorRunInstant('MD', 'backward', speedpct)


# def motor(A1, A2, B1, B2):
#     motor1.duty_u16(A1)
#     motor2.duty_u16(A2)
#     motor3.duty_u16(B1)
#     motor4.duty_u16(B2)


def moveSpeed(leftSpeed, rightSpeed):
    margin = 3000
    speedpct1 = 30
    speedpct2 = 30
    if leftSpeed > rightSpeed + margin:
        # move to left
        # m.MotorRunInstant('MA', 'backward', speedpct)
        m.MotorRunInstant('MB', 'forward', speedpct1)
        # m.MotorRunInstant('MC', 'backward', speedpct)
        m.MotorRunInstant('MD', 'forward', speedpct1)
    elif leftSpeed < rightSpeed - margin:

        # move to right
        m.MotorRunInstant('MA', 'forward', speedpct1)
        # m.MotorRunInstant('MB', 'backward', speedpct)
        m.MotorRunInstant('MC', 'forward', speedpct1)
        # m.MotorRunInstant('MD', 'backward', speedpct)
    else:
        # move straight
        m.MotorRunInstant('MA', 'forward', speedpct2)
        m.MotorRunInstant('MB', 'forward', speedpct2)
        m.MotorRunInstant('MC', 'forward', speedpct2)
        m.MotorRunInstant('MD', 'forward', speedpct2)


def moveSpeed2(tz, tx, last_direction):
    margin = 20
    z_limit = 220
    speedpct1 = 25
    speedpct2 = 25
    current_direction = last_direction
    print(f'**last_direction:{last_direction}**')
    # last_direction 0, move left, 1, move right, 2, straight, 3 stop
    if tz < z_limit:
        ratio = tz / z_limit
        speedpct1 = int(float(speedpct1) * ratio)
        speedpct2 = int(float(speedpct2) * ratio)

    if tx < -margin and last_direction != 0:
        # move to left
        m.MotorStop('MA')
        m.MotorRunInstant('MB', 'forward', speedpct1)
        m.MotorStop('MC')
        m.MotorRunInstant('MD', 'forward', speedpct1)
        current_direction = 0
        print(f'move left ------------{speedpct1}')
    elif tx > margin and last_direction != 1:

        # move to right
        m.MotorRunInstant('MA', 'forward', speedpct1)
        m.MotorStop('MB')
        m.MotorRunInstant('MC', 'forward', speedpct1)
        m.MotorStop('MD')
        print(f'{speedpct1} +++++++++++++++  move right')
        current_direction = 1
        # m.MotorRunInstant('MD', 'backward', speedpct)
    elif tx <= margin and tx >= -margin and last_direction != 2:
        # move straight
        m.MotorRunInstant('MA', 'forward', speedpct2)
        m.MotorRunInstant('MB', 'forward', speedpct2)
        m.MotorRunInstant('MC', 'forward', speedpct2)
        m.MotorRunInstant('MD', 'forward', speedpct2)
        print(f' =========={speedpct2}===============')
        current_direction = 2
    elif tz < z_limit and last_direction != 3:
        stopMove()
        current_direction = 3
    return current_direction


def moveSpeed_track_line(last_direction):
    Track = Track_L.value() * 2 + Track_R.value()
    print(f'Track={Track}')
    speedpct1 = 20
    current_direction = 3
    # sleep(0.01)
    # last_direction 0, move left, 1, move right, 2, straight, 3 stop
    if Track == 0 and last_direction != 3:  # No black lines detected on the left and right
        stopMove()
        current_direction = 3
    # Only the black line is detected on the right side
    elif Track == 1 and last_direction != 1:
        # move to right
        m.MotorRunInstant('MA', 'forward', speedpct1)
        m.MotorStop('MB')
        m.MotorRunInstant('MC', 'forward', speedpct1)
        m.MotorStop('MD')
        print(f'{speedpct1} +++++++++++++++  move right')
        current_direction = 1
    elif Track == 2 and last_direction != 0:  # Only the black line is detected on the left side
        # move to left
        m.MotorStop('MA')
        m.MotorRunInstant('MB', 'forward', speedpct1)
        m.MotorStop('MC')
        m.MotorRunInstant('MD', 'forward', speedpct1)
        current_direction = 0
        print(f'move left ------------{speedpct1}')
    # Black lines are detected on both the left and right
    elif Track == 3 and last_direction != 2:
        # move straight
        m.MotorRunInstant('MA', 'forward', speedpct1)
        m.MotorRunInstant('MB', 'forward', speedpct1)
        m.MotorRunInstant('MC', 'forward', speedpct1)
        m.MotorRunInstant('MD', 'forward', speedpct1)
        print(f' =========={speedpct1}===============')
        current_direction = 2

    return current_direction


stop_time = 0.05
stop_time2 = 0.1
stopEn = False
speedInc = 30
speedr = 0
speedl = 0
speedf = 0


def moveFSpeed(speedpct1, last_direction):
    global speedf
    global stopEn
    global stop_time
    global stop_time2
    global speedInc
    # move straight
    if last_direction == 2:  # last move is straight, gradually move forward
        speedf += speedInc
        speedf = min(speedpct1, speedf)
    else:
        speedf = speedInc

    logic1 = not ((not stopEn) and last_direction == 2)

    print(f' logicf= {logic1}')
    # if not(stopEn and last_direction ==2):
    if stopEn or not ((not stopEn) and last_direction == 2):
        m.MotorRunInstant('MA', 'forward', speedf)
        m.MotorRunInstant('MB', 'forward', speedf)
        m.MotorRunInstant('MC', 'forward', speedf)
        m.MotorRunInstant('MD', 'forward', speedf)
        print(f' =========={speedf}===============')
    else:
        print(f' =========={speedf} no command===============')

    if stopEn:
        sleep(stop_time)
        stopMove()
        sleep(stop_time2)


def moveLSpeed(speedpct1, last_direction):
    global speedl
    global stopEn
    global stop_time
    global stop_time2
    global speedInc
    # move straight
    if last_direction == 0:  # last move is left, gradually move left
        speedl += speedInc
        speedl = min(speedpct1, speedl)
    else:
        speedl = speedInc
    logic1 = not ((not stopEn) and last_direction == 0)
    print(f' logicl= {logic1}')
    if stopEn or not ((not stopEn) and last_direction == 0):

        m.MotorRunInstant('MA', 'backward', speedl)
        # m.MotorStop('MA')
        m.MotorRunInstant('MB', 'forward', speedl)
        m.MotorRunInstant('MC', 'backward', speedl)
        # m.MotorStop('MC')
        m.MotorRunInstant('MD', 'forward', speedl)
        print(f'move left ------------{speedl}')
    else:
        print(f'move left ------------{speedl} no command')

    if stopEn:
        sleep(stop_time)
        stopMove()
        sleep(stop_time2)


def moveRSpeed(speedpct1, last_direction):
    global speedr
    global stopEn
    global stop_time
    global stop_time2
    global speedInc
    # move straight
    if last_direction == 1:  # last move is left, gradually move left
        speedr += speedInc
        speedr = min(speedpct1, speedr)
    else:
        speedr = speedInc
    logic1 = not ((not stopEn) and last_direction == 1)
    print(f' logicr= {logic1}')
    if stopEn or not ((not stopEn) and last_direction == 1):
        m.MotorRunInstant('MA', 'forward', speedr)
        m.MotorRunInstant('MB', 'backward', speedr)
        # m.MotorStop('MB')
        m.MotorRunInstant('MC', 'forward', speedr)
        m.MotorRunInstant('MD', 'backward', speedr)
        print(f'{speedr} +++++++++++++++  move right')
    else:
        print(f'{speedr} no command +++++++++++++++  move right')
        # m.MotorStop('MD')

    if stopEn:
        sleep(stop_time)
        stopMove()
        sleep(stop_time2)


def moveSpeed_track_line2(last_direction):
    global track_ct
    left = Track_L.value()
    right = Track_R.value()
    center = Track_C.value()

    print(f'left={left}, center={center}, right={right},ct={track_ct}')
    speedpct1 = 30
    current_direction = 3

    # sleep(0.01)
    # last_direction 0, move left, 1, move right, 2, straight, 3 stop
    # No black lines detected on the left and right
    if center == 0 and left == 0 and right == 0:
        stopMove()
        current_direction = 3
    # Black lines are detected on both the left and right
    elif center == 1 and left == 0 and right == 0:
        # move straight
        moveFSpeed(speedpct1, last_direction)
        current_direction = 2
    elif right == 1 and left == 0:  # Only the black line is detected on the right side
        # move to right
        moveRSpeed(speedpct1, last_direction)
        current_direction = 1
    elif left == 1 and right == 0:  # Only the black line is detected on the left side
        # move to left
        moveLSpeed(speedpct1, last_direction)
        current_direction = 0

    # Only the black line is detected on the left side
    elif left == 1 and center == 1 and right == 1:
        if last_direction == 1:
            # move right
            moveRSpeed(speedpct1, last_direction)
            current_direction = 1
        if last_direction == 0:
            # move to left
            moveLSpeed(speedpct1, last_direction)
            current_direction = 0

    return current_direction


def moveSpeed_track_line3(last_direction):
    Track = Track_L.value() * 2 + Track_R.value()
    print(f'Track={Track}')
    speedpct1 = 30
    speedpct2 = 30
    current_direction = 3
    stop_time = 0.05
    stop_time2 = 0.2
    # sleep(0.01)
    # last_direction 0, move left, 1, move right, 2, straight, 3 stop
    if Track == 0:  # No black lines detected on the left and right
        stopMove()
        current_direction = 3
    elif Track == 1:  # Only the black line is detected on the right side
        # move to right
        moveRSpeed(speedpct1, last_direction)
        current_direction = 1
    elif Track == 2:  # Only the black line is detected on the left side
        # move to left
        moveLSpeed(speedpct1, last_direction)
        current_direction = 0

    elif Track == 3:  # Black lines are detected on both the left and right
        # move straight
        moveFSpeed(speedpct1, last_direction)
        current_direction = 2

    return current_direction


def moveContiuously(leftSpeed, rightSpeed):
    leftpct = int(leftSpeed / speed * 100)
    rightpct = int(rightSpeed / speed * 100)
    print(f'drive pct: {leftpct}, {rightpct}')
    m.MotorRunInstant('MA', 'forward', rightpct)
    m.MotorRunInstant('MB', 'forward', leftpct)
    m.MotorRunInstant('MC', 'forward', rightpct)
    m.MotorRunInstant('MD', 'forward', leftpct)


def moveParallel(leftSpeed, rightSpeed):
    margin = 2000
    speedpct1 = 30
    speedpct2 = 30
    if leftSpeed > rightSpeed + margin:
        # move to parallel left
        m.MotorRunInstant('MA', 'forward', speedpct2)
        m.MotorRunInstant('MB', 'backward', speedpct2)
        m.MotorRunInstant('MC', 'backward', speedpct1)
        m.MotorRunInstant('MD', 'forward', speedpct1)
    elif leftSpeed < rightSpeed - margin:

        # move to parallel right
        m.MotorRunInstant('MA', 'backward', speedpct2)
        m.MotorRunInstant('MB', 'forward', speedpct2)
        m.MotorRunInstant('MC', 'forward', speedpct1)
        m.MotorRunInstant('MD', 'backward', speedpct1)


def moveRotate(z_dis, x_dis, speedpct1):
    z_limit = 200
    # nano margin
    # margin1=20
    # mv210 margin
    margin1 = 10
    alignment = True
    if z_dis > z_limit:
        margin = margin1 * z_dis / z_limit  # 2 cm proportion to z_dis
    else:
        margin = margin1

    if x_dis < -margin:
        # move to rotate left
        m.MotorRunInstant('MA', 'backward', speedpct1)
        m.MotorRunInstant('MB', 'backward', speedpct1)
        m.MotorRunInstant('MC', 'forward', speedpct1)
        m.MotorRunInstant('MD', 'forward', speedpct1)
        alignment = False
    elif x_dis > margin:

        # move to rotate right
        m.MotorRunInstant('MA', 'forward', speedpct1)
        m.MotorRunInstant('MB', 'forward', speedpct1)
        m.MotorRunInstant('MC', 'backward', speedpct1)
        m.MotorRunInstant('MD', 'backward', speedpct1)
        alignment = False
    return alignment


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
    # sleep(0.2)
    stopMove()


def keepTurnLeft(sp):
    rotateLeftSpd(sp)
    rotateLeftSpd(sp - 5)
    rotateLeftSpd(sp - 8)
    rotateLeftSpd(sp - 10)
    # sleep(0.2)
    stopMove()


def keepTurnRight(sp):
    rotateRightSpd(sp)
    rotateRightSpd(sp - 5)
    rotateRightSpd(sp - 8)
    rotateRightSpd(sp - 10)
    # sleep(0.2)
    stopMove()


def keepBackward(sp):
    moveBackwardSpd(sp)
    moveBackwardSpd(sp - 5)
    moveBackwardSpd(sp - 10)
    # sleep(0.2)
    stopMove()


def moveUpDown(z_dis, y_dis, speedpct1):
    z_limit = 200
    # nano margin
    # margin1=20
    # mv210 margin
    margin1 = 30
    margin2 = 10
    alignment = True
    if z_dis > z_limit:
        marginUp = margin1 * z_dis / z_limit  # 2 cm proportion to z_dis
        marginDown = margin2 * z_dis / z_limit
    else:
        marginUp = margin1
        marginDown = margin2

    if y_dis < -marginDown:
        # move up
        armUp(speedpct1)
        alignment = False
        print('move up up up up')
    elif y_dis > marginUp:

        # arm down
        # armDown(speedpct1-30)
        armDown(speedpct1 - 35)
        alignment = False
        print('move down down down')
    return alignment


def openClaw():
    #  open claw
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

    RotateBCCW(currentGrabLevel)
    sleep(0.5)
    StopMotor()
    sleep(0.2)
    grabLastState = 2


def closeClaw():
    # close claw
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
    RotateBCW(currentGrabLevel)
    sleep(0.5)
    StopMotor()
    sleep(0.2)
    grabLastState = 1


# def getDistance():
#     trig = Pin(4, Pin.OUT)
#     trig.value(0)
#     utime.sleep_us(2)
#     trig.value(1)
#     utime.sleep_us(20)
#     trig.value(0)
#     echo = Pin(5, Pin.IN)
#     while echo.value() == 0:
#         start = utime.ticks_us()
#     while echo.value() == 1:
#         end = utime.ticks_us()
#     d = (end - start) * 0.0343 / 2
#     # print("d: ", end="")
#     # print(d)
#     return d


trig = 4
numLimit = 1000


def getDistance(trig):
    global numLimit
    trig = Pin(4, Pin.OUT)
    trig.value(0)
    utime.sleep_us(2)
    trig.value(1)
    utime.sleep_us(10)
    trig.value(0)
    trig = Pin(5, Pin.IN)
    num = 0
    start = 0
    end = 0
    valid = True  # to prevent stuck in loop forever
    while trig.value() == 0 and valid:
        start = utime.ticks_us()
        num = num + 1
        if num > numLimit:
            valid = False
    if not valid:
        d = 0
    else:
        num = 0
        while trig.value() == 1 and valid:
            end = utime.ticks_us()
            num = num + 1
            if num > numLimit:
                valid = False
        if not valid:
            d = 0
        else:
            # print("end= %s" % end)
            # print("start= %s" % start)
            d = (end - start) * 0.0343 / 2
            d = max(d, 0)
    return d


# def Obstacle_Avoidance():
#   motor(speed,0,0,speed) # turn left
#   utime.sleep(0.2)
#   motor(0,0,0,0) #stop
#   utime.sleep(0.5)
#   distance_left = int(getDistance(trig))
#   motor(0,speed,speed,0) # turn right
#   utime.sleep(0.4)
#   motor(0,0,0,0)
#   utime.sleep(0.5)
#   distance_right = int(getDistance(trig))
#   if distance_left > 20 and distance_right > 20:
#     if distance_left > distance_right:
#       motor(speed,0,0,speed) # turn left
#       utime.sleep(0.4)
#     else:
#       motor(speed,0,speed,0) # go backwards
#       utime.sleep(0.2)
#   else:
#     motor(0,speed,speed,0) # turn right
#     utime.sleep(0.5)


def Dodge():
    speedpct1 = 30
    current_direction = 3
    stop_time = 0.05
    stop_time2 = 0.2
    distance_limit = 20

    distance = getDistance(trig)
    if distance < distance_limit and distance > 0:
        #     motor(0, 800, 800, 0) # turn right
        #     utime.sleep(0.5)
        #     motor(0, 0, 0, 0)
        m.MotorRunInstant('MA', 'forward', speedpct1)
        m.MotorRunInstant('MB', 'forward', speedpct1)
        m.MotorRunInstant('MC', 'backward', speedpct1)
        m.MotorRunInstant('MD', 'backward', speedpct1)
        sleep(stop_time)
        stopMove()
        sleep(stop_time2)
        print(f'{speedpct1} +++++++++++++++  move right')
    else:
        # motor(0, 800, 0, 800) # go forward

        m.MotorRunInstant('MA', 'forward', speedpct1)
        m.MotorRunInstant('MB', 'forward', speedpct1)
        m.MotorRunInstant('MC', 'forward', speedpct1)
        m.MotorRunInstant('MD', 'forward', speedpct1)
        sleep(stop_time)
        stopMove()
        sleep(stop_time2)
        print(f' ==========move forward {speedpct1}===============')


# def Obstacle_Avoidance():
#     motor(speed, 0, 0, speed)
#     utime.sleep(0.2)
#     motor(0, 0, 0, 0)
#     utime.sleep(0.5)
#     distance_left = int(getDistance())
#     motor(0, speed, speed, 0)
#     utime.sleep(0.4)
#     motor(0, 0, 0, 0)
#     utime.sleep(0.5)
#     distance_right = int(getDistance())
#     if distance_left > 20 and distance_right > 20:
#         if distance_left > distance_right:
#             motor(speed, 0, 0, speed)
#             utime.sleep(0.4)
#         else:
#             motor(speed, 0, speed, 0)
#             utime.sleep(0.2)
#     else:
#         motor(0, speed, speed, 0)
#         utime.sleep(0.5)


# def Advance():
#     distance = getDistance(trig)
# if distance < 20:
#     # Obstacle_Avoidance()
# else:
#     # motor(0, speed, 0, speed)


# def Dodge():
#     distance = getDistance()
#     if distance < 20:
#         motor(0, speed, speed, 0)
#         utime.sleep(0.5)
#         motor(0, 0, 0, 0)
#     else:
#         motor(0, speed, 0, speed)


def Music():
    global buzzer
    for i in range(0, len(Song)):
        buzzer.duty_u16(2000)
        buzzer.freq(Song[i])
        for j in range(1, Beat[i] / 0.1):
            time.sleep_ms(25)
        buzzer.duty_u16(0)
        time.sleep(0.01)


def Greeting():
    global buzzer
    for i in range(0, len(Song2)):
        buzzer.duty_u16(2000)
        buzzer.freq(Song2[i])
        for j in range(1, Beat2[i] / 0.1):
            time.sleep_ms(25)
        buzzer.duty_u16(0)
        time.sleep(0.01)


def exec_cmd(key_val):
    global state_value
    global mode, firstLoop
    if (key_val == 0x18):
        #         print("Button ^")
        moveForwardSpd(30)
        time.sleep(1)
        stopMove()
    #     # motor(0, speed, 0, speed)  # Go forward
    # elif (key_val == 0x08):
    #         print("Button <")
    #     motor(speed, 0, 0, speed)  # Turn left
    # elif (key_val == 0x5a):
    #     #         print("Button >")
    #     motor(0, speed, speed, 0)  # Turn right
    elif (key_val == 0x52):
        #         print("Button V")
        moveBackwardSpd(30)
        time.sleep(1)
        stopMove()
    #     motor(speed, 0, speed, 0)  # Go back
    # elif (key_val == 0x45):
    #     #         print("Button 1")
    #     RGB(65534, 65534, 65534)
    # elif (key_val == 0x46):
    #     #         print("Button 2")
    #     Led_R.duty_u16(0)
    #     Led_G.duty_u16(0)
    #     Led_B.duty_u16(0)
    # elif (key_val == 0x47):
    #     #         print("Button 3")     buzzer
    #     RGB(0, 0, 0)
    # elif (key_val == 0x44):
    #     #         print("Button 4")     buzzer
    #     print("music")
    #     buzzer.duty_u16(2000)
    #     buzzer.freq(587)
    # elif (key_val == 0x40):
    #     # print("Button 5")     #buzzer
    #     # Advance()
    #     state_value = 2
    # elif (key_val == 0x43):
    #     #         print("Button 6")     buzzer
    #     # Tracking()
    #     state_value = 3
    # elif (key_val == 0x07):
    #     #         print("Button 7")     buzzer
    #     # Find_light()
    #     state_value = 4
    # elif (key_val == 0x15):
    #     #         print("Button 8")     buzzer
    #     # Dodge()
    #     state_value = 5
    # elif (key_val == 0x09):
    #     #         print("Button 9")     buzzer
    #     Music()
    # elif (key_val == 0x19):
    #     #         print("Button 0")     buzzer
    #     #         buzzer.duty_u16(2000)
    #     #         buzzer.freq(587)
    #     mode = True
    #     firstLoop = 0
    #     print(mode)
    # elif (key_val == 0x0d):
    #     #         print("Button 0")     buzzer
    #     state_value = 0
    # else:
    #     motor(0, 0, 0, 0)  # Stop
    #     buzzer.duty_u16(0)


#         print("STOP")


# WIFI 连接函数
wifi_wait_time = 0


def log(message, level="INFO"):
    timestamp = utime.localtime(utime.time())
    formatted_time = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
        *timestamp)
    print("[{}] [{}] {}".format(formatted_time, level, message))


def release_wifi():
    sta_if = network.WLAN(network.STA_IF)

    if sta_if.isconnected():
        # 如果已连接，先断开连接
        sta_if.disconnect()
        log("Disconnected from Wi-Fi network.", "INFO")

    # 禁用STA接口
    sta_if.active(False)
    log("STA interface disabled. Wi-Fi cache released.", "INFO")


def do_connect():
    global wifi_wait_time, wifi_connect

    # 调用释放Wi-Fi缓存的函数
    release_wifi()

    sta_if = network.WLAN(network.STA_IF)

    if not sta_if.isconnected():
        try:
            log('Connecting to network...', 'INFO')
            sta_if.active(True)
            sta_if.connect(wifiName, wifiPassword)
            while not sta_if.isconnected():
                utime.sleep(1)
                wifi_wait_time += 1
                if wifi_wait_time >= 10:
                    raise Exception("Connection timeout")
            wifi_connect = True
            log('Connected to network successfully', 'INFO')
            log(str(sta_if.ifconfig()), 'INFO')
        except Exception as e:
            log("Connection error: " + str(e), 'ERROR')
    else:
        log(str(sta_if.ifconfig()), 'INFO')
        wifi_connect = True


def connect_and_subscribe():
    try:
        client = MQTTClient(client_id=clientID,
                            server=serverIP, port=port, keepalive=6000)
        client.set_callback(MsgOK)
        client.connect()
        client.subscribe(myTopic)
        log("Connected to MQTT server at {}:{}".format(serverIP, port), 'INFO')
        return client
    except Exception as e:
        log("Failed to connect to MQTT server: " + str(e), 'ERROR')
        restart_and_reconnect()


def restart_and_reconnect():
    print('Failed to connect to MQTT broker. Reconnecting...')
    time.sleep(10)
    machine.reset()


def connect_show_params(client, request):
    global mqtt_client
    global run
    global serverIP
    global port
    global machineId
    global server
    global state_value
    ''' request handler '''
    params = utils.get_request_query_params(request)
    print(params)
    ips = params['mqtt_ip'].split(":")
    serverIP = ips[0]
    port = ips[1]
    ''' will return {"param_one": "one", "param_two": "two"} '''
    server.send(client, "HTTP/1.0 200 OK\r\n")
    server.send(client, "Content-Type: text/html\r\n\r\n")
    if machineId != params['machineid']:
        return server.send(client, "Not this car")
    if run == True:
        return server.send(client, "mqtt is connected!")
    try:
        mqtt_client = connect_and_subscribe()
        server.send(client, "ok")
        run = True
        state_value = 12
        # server.stop()
    except OSError as e:
        server.send(client, "failed")


def stop_show_params(client, request):
    global mqtt_client
    global run
    global machineId
    ''' request handler '''
    params = utils.get_request_query_params(request)
    print(params)
    server.send(client, "HTTP/1.0 200 OK\r\n")
    server.send(client, "Content-Type: text/html\r\n\r\n")
    if machineId != params['machineid']:
        return server.send(client, "Not this car")
    if run != True:
        return server.send(client, "No mqtt connected!")
    try:
        mqtt_client.disconnect()
        server.send(client, "ok")
        run = False
    except OSError as e:
        server.send(client, "failed")


def status_show_params(client, request):
    global run, serverIP, port, machineId, server
    ''' request handler '''
    params = utils.get_request_query_params(request)
    print(params)
    if machineId != params['machineid']:
        server.send(client, "HTTP/1.0 200 OK\r\n")
        server.send(client, "Content-Type: text/html\r\n\r\n")
        return server.send(client, "Not this car")
    json_str = json.dumps(
        {"run": run, "mqtt_ip": "{}:{}".format(serverIP, port)})
    server.send(client, "HTTP/1.0 200 OK\r\n")
    server.send(client, "Content-Type: application/json\r\n\r\n")
    server.send(client, json_str)


def reconstruct_data(data, tagsList):
    # 使用 eval() 替代 ast.literal_eval()，注意安全风险
    tags_dict = eval(tagsList)
    data_list = eval(data)

    print(data_list)

    data_list = [[y, x] if 'action' in y else [x, y]
                 for x, y in data_list]  # 简化条件判断
    print(data_list)

    target_id_list = []
    target_action_list = []

    # 迭代处理 data_list
    for item in data_list:
        action = item[0]
        tag = item[1]

        # 处理标签非空的情况
        if tag:
            if tag.startswith("tag"):
                tag_id = int(tag.replace("tag", ""))
                target_id_list.append(tag_id)
            else:
                target_id_list.append("")
        else:
            target_id_list.append("")

        # 根据 action 和 tag 找到对应的操作描述
        if action in tags_dict.values():
            action_desc = {v: k for k, v in tags_dict.items()}[action]
            target_action_list.append(action_desc)
        elif tag and tag in tags_dict.values():
            action_desc = {v: k for k, v in tags_dict.items()}[tag]
            target_action_list.append(action_desc)

    target_action_list.append("finished")
    target_id_list.append("")

    return (target_id_list, target_action_list)


def send_msg_to_k210():
    global target_index, target_id_list, target_action_list
    if target_index < len(target_id_list) and target_action_list[target_index] != 'finished':
        td = str(target_id_list[target_index]) if str(target_id_list[target_index]) != '' else grab_color
        message = target_action_list[target_index] + ":" + td + ":" + str(target_index)  # + '\n'
        print(message)
        # r = uart.write(target_action_list[target_index] + ":" + str(target_id_list[target_index]) + '\n')
        # print(r)
        # if r > 0:
        #     print("Data sent successfully")
        # else:
        #     print("No data was sent")
        send_message_to_server(message)


# ---------- key program to call functions in mqtt mode, when receiving message
def MsgOK(topic, msg):
    log("Received message: {} on topic: {}".format(msg, topic), 'INFO')
    global state_value
    global speed
    global mqtt_client
    global mode, firstLoop
    global grabAngle, releaseAngle
    global target_id_list, target_action_list
    global target_index
    global uart
    if topic == myTopic.encode():
        pattern = r'^actions:'
        if re.search(pattern, msg.decode('utf-8')):
            result = re.sub(pattern, '', msg.decode('utf-8'))
            # actions:[['','action1'],['action2','tag11'],['action1',' '],['action2','tag11']]
            (target_id_list, target_action_list) = reconstruct_data(result, tagsList)
            print(target_id_list)
            print(target_action_list)
            target_index = 0
            # state_value = 12
            send_msg_to_k210()
        elif msg == b'auto':  # when receive auto, it will reject all others
            state_value = 12
        elif state_value != 12:
            if msg == b'backward':
                #             speedpct=20
                #             moveBackward()
                state_value = 8

                # motor(0, 0, 0, 0)
            elif msg == b'forward':
                # speedpct=20
                # moveForward()
                state_value = 7

                # motor(0, 0, 0, 0)
            elif msg == b'down50':
                speedpct = 50
                moveBackward()
                # motor(speed, 0, speed, 0)
                utime.sleep(1)
                state_value = 0
                stopMove()
                # motor(0, 0, 0, 0)
            elif msg == b'up50':
                speedpct = 50
                moveForward()
                # motor(0, speed, 0, speed)
                utime.sleep(1)
                state_value = 0
                stopMove()
            elif msg == b'down70':
                speedpct = 70
                moveBackward()
                # motor(speed, 0, speed, 0)
                utime.sleep(1)
                state_value = 0
                stopMove()
                # motor(0, 0, 0, 0)
            elif msg == b'up70':
                speedpct = 70
                moveForward()
                # motor(0, speed, 0, speed)
                utime.sleep(1)
                state_value = 0
                stopMove()
            elif msg == b'down90':
                speedpct = 90
                moveBackward()
                # motor(speed, 0, speed, 0)
                utime.sleep(1)
                state_value = 0
                stopMove()
                # motor(0, 0, 0, 0)
            elif msg == b'up90':
                speedpct = 90
                moveForward()
                # motor(0, speed, 0, speed)
                utime.sleep(1)
                state_value = 0
                stopMove()
            elif msg == b'down100':
                speedpct = 100
                moveBackward()
                # motor(speed, 0, speed, 0)
                utime.sleep(1)
                state_value = 0
                stopMove()
                # motor(0, 0, 0, 0)
            elif msg == b'up100':
                speedpct = 100
                moveForward()
                # motor(0, speed, 0, speed)
                utime.sleep(1)
                state_value = 0
                stopMove()
            elif msg == b'left':
                state_value = 9

                # motor(speed, 0, 0, speed)
            #             speedpct=15
            #             rotateLeft()
            #             utime.sleep(0.1)
            #
            #             stopMove()
            # motor(0, 0, 0, 0)
            elif msg == b'right':
                state_value = 10

                # motor(0, speed, speed, 0)
            #             speedpct=15
            #             rotateRight()
            #             utime.sleep(0.1)
            #
            #             stopMove()
            # motor(0, 0, 0, 0)
            elif msg == b'middle':
                state_value = 11

            elif msg == b'armUp':
                state_value = 0
                armUp(armUpSpd)
            elif msg == b'armDown':
                state_value = 0
                armDown(10)
            elif msg == b'openClaw':
                state_value = 5
            elif msg == b'closeClaw':
                state_value = 6

            # if msg == b'down':
            #     #motor(speed, 0, speed, 0)
            #     moveDown()
            # elif msg == b'up':
            #     #motor(0, speed, 0, speed)
            #     print('move up')
            #     moveUp()
            #     print('complete move up')
            # elif msg == b'left':
            #     #motor(speed, 0, 0, speed)
            #     moveLeft()
            # elif msg == b'right':
            #     #motor(0, speed, speed, 0)
            #     moveRight()
            # elif msg == b'stop':
            #     state_value = 0
            #     motor(0, 0, 0, 0)
            elif msg == b'turnon':
                RGB(65534, 65534, 65534, 1)
                mqtt_client.publish("Car Light", "light on")
            elif msg == b'turnoff':
                RGB(0, 0, 0, 0)
                mqtt_client.publish("Car Light", "light off")
            elif msg == b'random':
                print("run random")
                RGB(-1, -1, -1, 1)
            elif msg == b'dodge':
                print("run dodge")
                state_value = 3
            elif msg == b'music':
                print("run music")
                state_value = 4
            elif msg == b'ungrab':
                print("ungrab")
                state_value = 5
            elif msg == b'grab':
                print("grab")
                state_value = 6
            elif msg == b'greeting':
                print("greeting")
                Greeting()
            elif msg == b'flashlight':
                print("flash light")
                state_value = 7
            elif msg == b'iRemote':  # switch to remote controller mode
                mode = False
                firstLoop = 0


server = MicroPyServer()
''' add route '''
server.add_route("/connect", connect_show_params)
server.add_route("/stop", stop_show_params)
server.add_route("/status", status_show_params)


def monitor_wifi_connection():
    global wifi_connect

    sta_if = network.WLAN(network.STA_IF)
    if sta_if.isconnected():
        log("WiFi status: 已连接")
        log("信号强度: " + str(sta_if.status('rssi')))
        r, msg = test_connectivity()
        log(msg)
        if r == False:
            wifi_connect = False
    else:
        log("WiFi status: 未连接")
        wifi_connect = False


def test_connectivity():
    sta_if = network.WLAN(network.STA_IF)

    try:
        if sta_if.isconnected():
            addr_info = socket.getaddrinfo(sta_if.ifconfig()[2], 80)
            addr = addr_info[0][-1]

            s = socket.socket()
            s.settimeout(5)
            s.connect(addr)
            s.close()
            return True, "Successfully connected to LAN!"
        else:
            return False, "WIFI can't connected!"
    except OSError as e:
        return False, "Unable to connect to LAN：" + str(e)


def parse_data(data):
    values_list = []
    lines = data.split('\n')

    for line in lines:
        if line.strip():  # Check if the line is not just whitespace
            values = {}
            data_items = line.split(',')
            for item in data_items:
                key, value = item.strip().split(':')
                try:
                    values[key] = float(value) if '.' in value else int(value)
                except ValueError:
                    values[key] = value
            values_list.append(values)

    return values_list


# 初始化小车状态

# motor(0, 0, 0, 0)
setup()
stopMove()
buzzer.duty_u16(0)
# mode = 'uart'
firstLoop = 0
wifi_connect = False
state_value = 0
# 初始化UART
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
# uart = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17))

# RGB(65534, 65534, 65534)
# RGB(0, 65534, 0)
# RGB(0, 0, 65534)
RGB(0, 0, 0, 0)
xunzhao = 0
xunzhao1 = 0
utime.sleep(1)
count = 0
count2 = 0
count_limit = 10
count_start = 3
startRunct = 6
maxSpeed = 65000
last_z = 0
last_x = 0
last_y = 0
start_fine_ct = 6
x_max = 200
z_max = 200
y_max = 200
almost_there_ct = 0
# for nano
# come_in_step=7
# for mv210
come_in_step = 4
got_there = False
grabbed = True
grab_limit = 3
grab_ct = 0
release_wait_ct = 0
release_num = 50
z_limit = 200
z_close_limit = 200
target_id = 33
grab_id = 1
received_id = 0
received_id2 = 0
zoomfactor = 40
z_target = 220
z_grab_target = 220
z_range = 10
# log_data(['location',"x","y","z","count2","almost_there_ct","grab_ct"], is_header=True)
# log_data(['time',"supplyVoltage","voltageAtADC"], is_header=True)
# log_data(['time',"L","C","R","Direction"], is_header=True)

last_direction = 3
grabbed_found = False
got_grabbed_area = False
rotateRight_ct = 0
rotateRight_limit = 15

track_line_setup()
led_ct = 0
led_ct2 = 0
led_gap = 10
led_gap2 = 240

firstLoop = 0

# related to k210 find target
xunzhao = 0
xunzhao1 = 0
utime.sleep(1)
count = 0
count2 = 0
count_limit = 10
count_start = 3
startRunct = 6
maxSpeed = 65000
last_z = 0
last_x = 0
last_y = 0
start_fine_ct = 6
x_max = 200
z_max = 200
y_max = 200
almost_there_ct = 0
# for nano
# come_in_step=7
# for mv210
come_in_step = 4
got_there = False
grabbed = True
grab_limit = 3
grab_ct = 0
release_wait_ct = 0
release_num = 50
z_limit = 200
z_close_limit = 200
target_id = 33
grab_id = 1
received_id = 0
received_id2 = 0
zoomfactor = 40
z_target = 220
z_grab_target = 220
z_range = 10
# log_data(['location',"x","y","z","count2","almost_there_ct","grab_ct"], is_header=True)
# log_data(['time',"supplyVoltage","voltageAtADC"], is_header=True)
last_direction = 3
grabbed_found = False
got_grabbed_area = False
rotateRight_ct = 0
rotateRight_limit = 15
track_ct = 0

mode = True
firstLoop = 0
wifi_connect = False
first_run_mqtt = True
first_run_web = True

# # ----------------- adjustable parameters -----------------
tagsList = "{'grab-by-color':'action1', 'put-down':'action2', 'apple':'tag11'}"

target_index = 0
target_id_list = ["", 11, "", 11, ""]

target_action_list = [
    "grab-by-color",
    "put-down",
    "grab-by-color",
    "put-down",
    "finished",
]
"""
action status: 

- locate: locate the object by the big tag.
- grab: grab the object by tag
- finish: finish the task
- grab-by-color: grab the object of the specific color
- put-down: put down the object
"""

grab_color = "red"

test_mode = False

big_tag_id_list = [
    21,
]
small_tag_id_list = [33, 11]
small_tag_width = 6.2
small_tag_zoomfactor = 0.28 * small_tag_width / 0.0003
big_tag_zoomfactor = 78 / 6.2

object_width = 4  # 3.4  # cm
color_obj_zoomfactor = 0.28 * object_width / 0.0003
object_height = 4  # 3.4  # cm
color_obj_zoomfactor_h = 0.28 * object_height / 0.0003
max_width = 10 * object_width

k210_cam_offset = 85 - 160 / 2  # 相机安装在机械臂上的偏移量
claw_range = (90 - 75) / 2
k210_qqvga = (240, 360)
k210_center = k210_qqvga[1] / 2 + k210_cam_offset  # QQVGA分辨率：120*160
k210_y_center = k210_qqvga[0] / 2
arm_range = 10  # pixel 上下浮动范围
rotate_in_front_of_obj = 5  # cm 在物体前方允许旋转的距离
last_w = None
last_h = None


def get_zf(id):
    if id in big_tag_id_list:
        return big_tag_zoomfactor
    else:
        return small_tag_zoomfactor


claw_open_len = 24  # 14  # cm
claw_close_len = 35  # 15

claw_grab_len = 2  # 11
claw_arm_up_len = 2  # 20  # 大于这个高度，需要抬起机械臂
grab_mode = False
put_down_obj = False
arm_up_len = 2
sp = 50
is_finished = False

gx = 0
gy = 0
gz = 0


# # ----------------- the life cycle of a job -----------------

# 判断当前应该前进还是后退，还是转弯，还是抓取


def get_action(cx, cy, w, h):
    global is_finished
    global grab_mode
    global target_index
    global uart
    if w > 0:
        obj_dis_w = color_obj_zoomfactor / w
    else:
        obj_dis_w = 0  # 或者设定一个合理的默认值

    if h > 0:
        obj_dis = color_obj_zoomfactor_h / h
    else:
        obj_dis = 0  # 或者设定一个合理的默认值
    print(f"obj_dis: {obj_dis}")
    print(f"obj_dis_w: {obj_dis_w}")
    # 如果cx大于k210_center + claw_range，说明物体在右边，右转

    # 如果obj_dis > rotate_in_front_of_obj + claw_open_len，说明物体在前方，调整角度
    if (
            obj_dis_w > 30 and not grab_mode
    ):
        if cy < k210_y_center - 5 * arm_range + 40:
            print("view is low, arm up")
            armUp(armUpSpd)
            # sleep(0.1)
        elif cy > k210_y_center + 5 * arm_range + 40:
            print("view is high, arm down")
            armDown(25)
            # sleep(0.1)
        elif cx > k210_center + claw_range:
            print("right")
            keepTurnRight(wheelSpd)

        # 如果cx小于k210_center - claw_range，说明物体在左边，左转
        elif cx < k210_center - claw_range:
            print("left")
            keepTurnLeft(wheelSpd)
        else:
            print("forward")
            for _ in range(1):
                keepForward(wheelSpd)
            # sleep(1)
    else:
        grab_mode = True
        if obj_dis > 100:
            armUp(armUpSpd)
            sleep(1)
            print("arm up, and h is: ", h)
        else:
            print("forward to grab")
            # moveForwardSpd(30)
            for _ in range(9):
                keepForward(wheelSpd)
                sleep(0.3)
            # armDown(25)
        if obj_dis <= 100:
            print("grab")
            for _ in range(3):
                keepForward(wheelSpd)
            for _ in range(2):
                armUp(armUpSpd)
            for _ in range(6):
                closeClaw()
            for _ in range(15):
                keepBackward(wheelSpd)
            target_index += 1
            send_msg_to_k210()
            sleep(3)
            # is_finished = True
        else:
            grab_mode = True
            if obj_dis > claw_arm_up_len:
                armUp(armUpSpd)
                sleep(1)
                print("arm up, and h is: ", h)
            else:
                print("forward to grab")
                # moveForwardSpd(30)
                for _ in range(2):
                    keepForward(wheelSpd)


def get_tag_action(tag_x, tag_y, tag_z):
    global put_down_obj
    global target_index
    global gx
    global gy
    global gz
    if tag_x == gx and tag_y == gy and tag_z == gz:
        print("right")
        # keepTurnLeft(30)
        keepTurnRight(30)
        # sleep(0.1)
        return
    else:
        gx = tag_x
        gy = tag_y
        gz = tag_z
    print(
        f"tag_x: {tag_x}, tag_y: {tag_y}, tag_z: {tag_z}")
    print(tag_y > 0 + 1.5 * arm_range, tag_y < 0 - 1.5 * arm_range, tag_x > 0 + 1.5 * claw_range,
          tag_x < 0 - 1.5 * claw_range, )
    if tag_z > claw_close_len + arm_up_len + 150:
        if tag_y > 0 + 1.5 * arm_range - 10:
            print("view high is low, arm up")
            armUp(armUpSpd)
            sleep(0.3)
        elif tag_y < 0 - 1.5 * arm_range - 10:
            print("view high is high, arm down")
            armDown(30)
            sleep(0.3)
        elif tag_x > 0 + 5 * claw_range:
            print("left")
            # keepTurnRight(30)
            keepTurnLeft(wheelSpd)
            sleep(0.3)
        elif tag_x < 0 - 5 * claw_range:
            print("right")
            # keepTurnLeft(30)
            keepTurnRight(wheelSpd)
            sleep(0.3)
        else:
            print("keep forward")
            for _ in range(1):
                keepForward(wheelSpd)
    else:
        for _ in range(3):
            armUp(armUpSpd)
            sleep(0.3)

        # for _ in range(8):
        #     keepForward(30)

        for _ in range(4):
            openClaw()

        for _ in range(10):
            keepBackward(wheelSpd)

        for _ in range(10):
            armDown(10)
            sleep(0.3)

        target_index += 1
        send_msg_to_k210()


server.start()

# keepForward(35)
# armDown(50)
# for _ in range(4):
#     armUp(armUpSpd)

for _ in range(4):
    openClaw()

# for _ in range(4):
#     keepForward(wheelSpd)

while True:
    firstLoop += 1
    if mode:  # mqtt mode
        # start to connect to Wifi
        if wifi_connect == False:
            # wifi_connect = True
            do_connect()
            monitor_wifi_connection()

        if run:  # when run=True, mqtt connection established
            first_run_web = True
            if mqtt_client:  # turn off light after mqtt connection established
                if first_run_mqtt:
                    RGB(0, 0, 0, 0)
                first_run_mqtt = False
                try:
                    mqtt_client.check_msg()
                except OSError as e:  # if error, restart
                    run = False
                    print('Failed to connect to MQTT broker. waiting...')
                    # restart_and_reconnect()
        else:
            first_run_mqtt = True
            # green light is on waiting for mqtt connection
            if wifi_connect:
                if first_run_web:
                    RGB(0, 65534, 0, 1)
                first_run_web = False
        server.loop()
        utime.sleep(0.1)

    else:  # remote control mode
        if firstLoop == 1:
            RGB(65534, 0, 0, 1)
        if firstLoop == 10000:
            RGB(0, 0, 0, 0)

        if IR_PIN.value() == 0:
            print(IR_PIN.value())
            count = 0
            while IR_PIN.value() == 0 and count < 200:
                count += 1
                utime.sleep_us(60)
            count = 0
            while IR_PIN.value() == 1 and count < 80:
                count += 1
                utime.sleep_us(60)
            idx = 0
            cnt = 0
            data = [0, 0, 0, 0]
            for i in range(0, 32):
                count = 0
                while IR_PIN.value() == 0 and count < 15:
                    count += 1
                    utime.sleep_us(60)
                count = 0
                while IR_PIN.value() == 1 and count < 40:
                    count += 1
                    utime.sleep_us(60)
                if count > 8:
                    data[idx] |= 1 << cnt
                if cnt == 7:
                    cnt = 0
                    idx += 1
                else:
                    cnt += 1
            if data[0] + data[1] == 0xFF and data[2] + data[3] == 0xFF:
                print("Retrieve key: 0x%02x" % data[2])
                N = data[2]
        if IR_PIN.value() == 1:
            stopMove()  # motor(0,0,0,0)     # Stop
            buzzer.duty_u16(0)
        else:
            exec_cmd(N)
            print('exec_cmd: ', end="")
            print(N)

    if uart.any():
        # print(f"current target id is: {target_id_list[target_index]}")
        try:
            uart2_data = uart.readline().decode("utf-8")
            data = json.loads(uart2_data)
            print(f"uart2_data: {data}")
            if data.get("nanoIp", None) is not None:
                print(data)
                nano_ip = data.get("nanoIp", None)
                nano_port = data.get("nanoProt", None)
                send_message_to_server("connected")
                with open('nano_ip.txt', 'w') as f:
                    f.write(uart2_data)
        except Exception as e:
            print("urat2 data error", e)
    # Run functions taking some time
    if state_value == 3:
        Dodge()
    elif state_value == 4:
        Music()
        state_value = 0
    elif state_value == 5:
        openClaw()
        openClaw()
        state_value = 0
    elif state_value == 6:
        closeClaw()
        closeClaw()
        state_value = 0
    elif state_value == 7:
        # print('move forward1')
        moveForwardSpd(25)
        state_value = 0
    elif state_value == 8:
        moveBackwardSpd(25)
        state_value = 0
    elif state_value == 9:
        rotateLeftSpd(40)
        state_value = 0
    elif state_value == 10:
        rotateRightSpd(40)
        state_value = 0
    elif state_value == 11:
        stopMove()
        state_value = 0

    #     # read voltage
    #     adc_value = adc.read_u16() >> 4  # Scale from 16-bit to 12-bit
    #     # Convert ADC value to voltage
    #     voltage_at_adc = (adc_value / ADC_MAX) * V_REF
    #     # Calculate the original voltage
    #     original_voltage = voltage_at_adc /resistance_ratio

    #     # Convert ADC value to voltage
    #     voltage_at_adc = (adc_value / ADC_MAX) * V_REF
    #     # Calculate the original voltage
    #     original_voltage = voltage_at_adc /resistance_ratio
    #
    #     batteryvoltage_show=False
    #     if batteryvoltage_show:
    #         print(f"battery voltage:{original_voltage}")
    #
    #     led_ct+=1
    #     # provide voltage warning
    #     if original_voltage< voltage_limit and led_ct>led_gap:
    #         led_ct=0
    #         led_r1.value(not led_r1.value())

    # k210 test
    grab_test = True
    is_finished = False
    target_index = 0
    grab_mode = False
    put_down_obj = False
    # armUp(10)
    # print('arm up')
    # moveForward()
    # keepTurnLeft(35)
    # state_value = 12
    errorcount = 0
    if grab_test and state_value == 12:
        for _ in range(10):
            armDown(10)

        for _ in range(8):
            openClaw()
        while is_finished is False and not test_mode:
            # 开启运行中也接受mqtt数据
            try:
                if mqtt_client:
                    mqtt_client.check_msg()
            except OSError as e:  # if error, restart
                pass
            if uart.any():
                # print(f"current target id is: {target_id_list[target_index]}")
                try:
                    uart2_data = uart.read().decode("utf-8").split("\n")[0]
                    # print(f"uart2_data: {uart2_data}")
                    # 查找}第一个出现的位置，然后截取字符串
                    json_end_index = uart2_data.find("}")
                    if json_end_index == -1:
                        continue
                    else:
                        uart2_data = uart2_data[: uart2_data.find("}") + 1]
                        uart2_data = re.sub(r'\{\}', '', uart2_data)
                        # print(f"uart2_data: {uart2_data}")
                        data = json.loads(uart2_data)
                        # print(f"uart2_data: {data}")

                        if data.get("nanoIp", None) is not None:
                            print(data)
                            nano_ip = data.get("nanoIp", None)
                            nano_port = data.get("nanoProt", None)
                            send_message_to_server("connected")
                            with open('nano_ip.txt', 'w') as f:
                                f.write(uart2_data)
                except Exception as e:
                    # print("urat2 data error", e)
                    errorcount += 1
                    if errorcount > 20:
                        data = {}
                        errorcount = 0
                    else:
                        continue

                # 同步运行状态，如何不同步，发送当前状态给nano进行同步，跳过下面步骤
                if data.get('mode', '') != str(target_index) and (
                        data.get("TagId", '') != '' or data.get("ObjectStatus", '') != ''):
                    send_msg_to_k210()
                    continue

                if target_index < len(target_action_list):
                    print(
                        f"current target action is: {target_action_list[target_index]}")
                else:
                    print("target action list is empty")
                    break

                if target_action_list[target_index] == "put-down":
                    tag_id = data.get("TagId", "N/A")
                    print(f"tag_id: {tag_id}")
                    if tag_id == target_id_list[target_index]:
                        zoomfactor = get_zf(tag_id)
                        # print(f"received tag id:{tag_id}")
                        # print(
                        #     f"x: {data.get('TagTx', '999')}, y: {data.get('TagTy', '999')}, z: {data.get('TagTz', '999')}"
                        # )
                        tag_z = int(abs(
                            float(data.get("TagTz", "999"))))
                        tag_x = int(
                            zoomfactor * float(data.get("TagTx", "999")))
                        tag_y = int(-zoomfactor *
                                    float(data.get("TagTy", "999")))
                        tag_cx = int(data.get("TagCx", "999"))
                        tag_cy = int(data.get("TagCy", "999"))
                        # print(
                        #     f"tag_z: {tag_z}, tag_cx: {tag_cx}, tag_cy: {tag_cy}")
                        get_tag_action(tag_cx, tag_cy, tag_z)
                    else:
                        keepTurnRight(30)
                        sleep(0.3)
                elif target_action_list[target_index] == "grab-by-color":
                    obj_status = data.get("ObjectStatus", "N/A")
                    print("obj_status:" + obj_status)

                    if obj_status == "get":

                        def get_obj_data(key):
                            global data
                            try:
                                return int(data.get(key, 0) if obj_status == "get" else 0)
                            except Exception as e:
                                print("get obj data err:", e)
                                print(data.get(key, 0))
                                return 0


                        obj_w = get_obj_data("ObjectWidth")
                        obj_h = get_obj_data("ObjectHeight")
                        obj_x = get_obj_data("ObjectX")
                        obj_y = get_obj_data("ObjectY")
                        obj_cx = get_obj_data("ObjectCX")
                        obj_cy = get_obj_data("ObjectCY")
                        print(
                            f"obj_status: {obj_status}, obj_w: {obj_w}, obj_h: {obj_h}, obj_x: {obj_x}, obj_y: {obj_y}, obj_cx: {obj_cx}, obj_cy: {obj_cy}"
                        )
                        try:
                            get_action(obj_cx, obj_cy, obj_w, obj_h)
                        except Exception as e:
                            print(e)
                    if obj_status == "none":
                        if grab_mode:
                            # for _ in range(5):
                            for _ in range(15):
                                closeClaw()
                            for _ in range(5):
                                armUp(armUpSpd)
                            for _ in range(8):
                                keepBackward(30)
                            # for _ in range(15):
                            #     armDown(10)
                            target_index += 1
                            grab_mode = False
                        else:
                            keepTurnRight(30)
                            sleep(0.3)

                elif target_action_list[target_index] == "finished":
                    is_finished = True
                    print('job finished')
                    state_value = 0
                    break
