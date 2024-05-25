import os
import time

import lcd
import sensor
from Maix import GPIO
from fpioa_manager import fm

print(os.listdir("/"))

qvga_lcd = (320, 240)
qqvga_lcd = (160, 120)
lcd.init(freq=15000000)  # 初始化屏幕显示
sensor.reset()  # 复位并初始化摄像头
sensor.set_pixformat(sensor.RGB565)  # 设置摄像头输出格式为 RGB565
sensor.set_framesize(sensor.QVGA)  # 设置摄像头输出大小为 QVGA (320x240)
# sensor.set_auto_gain(False)  # 必须关闭此功能，以防止图像冲洗…
# sensor.set_auto_whitebal(False)  # 必须关闭此功能，以防止图像冲洗…
#sensor.set_hmirror(1)  # 设置摄像头水平镜像
sensor.set_vflip(1)  # 设置摄像头垂直翻转

# sensor.set_windowing((160, 120))
sensor.run(1)
sensor.skip_frames(30)
fm.register(16, fm.fpioa.GPIO1)
KEY = GPIO(GPIO.GPIO1, GPIO.IN)

video_recording = False


def get_file_name():
    files = os.listdir("/sd")
    files = [f for f in files if f.startswith("cap_")]
    try:
        files.sort(key=lambda f: int(f[4:]))
        if files:
            file_num = int(files[-1][4:]) + 1
        else:
            file_num = 0
    except Exception as e:
        file_num = 0
    print("folder_num:", file_num)
    folder_name = "/sd/cap_%d" % file_num
    # create folder
    os.mkdir(folder_name)
    return folder_name


v_folder = None

while True:
    img = sensor.snapshot()
    lcd.display(img)
    if KEY.value() == 0:
        video_recording = not video_recording
        print("video_recording:", video_recording)
        lcd.draw_string(10, 10, "video_recording: %s" % video_recording, lcd.RED, lcd.WHITE)
        time.sleep(1)

    if video_recording:
        if not v_folder:
            v_folder = get_file_name()
        image_name = v_folder + "/%d.jpg" % time.ticks_ms()
        img.save(image_name)
        lcd.draw_string(10, 10, "save%s" % image_name, lcd.RED, lcd.WHITE)
        print("save image")
        time.sleep(2)
    else:
        v_folder = None
        lcd.draw_string(10, 10, "not recording", lcd.RED, lcd.WHITE)
        time.sleep(1)
        continue
