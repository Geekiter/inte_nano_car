import cv2
import os
from tkinter import *
from PIL import Image, ImageTk

# 获取原文件夹里文件夹的数量
save_path_base = './video_images'
if not os.path.exists(save_path_base):
    os.makedirs(save_path_base)
save_path_num = len(os.listdir(save_path_base))
save_path = os.path.join(save_path_base, f'images_{save_path_num+1}')
if not os.path.exists(save_path):
    os.makedirs(save_path)

# 初始化摄像头
cap = cv2.VideoCapture(0)

# 检查摄像头是否成功打开
if not cap.isOpened():
    raise ValueError("Can't find camera")

# 是否正在保存图片
is_saving = False
# 保存图片的计数器
frame_count = 0

# 创建Tkinter窗口
root = Tk()
root.title("Preview")

# 创建一个Label用于显示摄像头帧
lmain = Label(root)
lmain.pack()

# 更新帧的函数
def show_frame():
    global is_saving, frame_count
    # 从摄像头读取帧
    ret, frame = cap.read()
    frame = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))
    # 如果正在保存图片,每隔30帧保存一张
    if is_saving:
        frame_count += 1
        if frame_count % 10 == 0:
            img_name = os.path.join(save_path, f'image_{frame_count//10}.jpg')
            cv2.imwrite(img_name, frame)
    # 将帧转换为Tkinter可以使用的格式
    frame = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)
    # 每隔10毫秒更新一次帧
    lmain.after(10, show_frame)

# 开始保存图片
def start_saving():
    global is_saving, frame_count
    print(f'Start saving images')
    is_saving = True
    frame_count = 0

# 停止保存图片
def stop_saving():
    global is_saving
    print(f'Stop saving images')
    is_saving = False

# 创建保存图片按钮
btn_start_saving = Button(root, text="Start", width=50, command=start_saving)
btn_start_saving.pack(anchor=CENTER, expand=True)

# 创建停止保存图片按钮
btn_stop_saving = Button(root, text="Stop", width=50, command=stop_saving)
btn_stop_saving.pack(anchor=CENTER, expand=True)

show_frame()
# 启动Tkinter事件循环
# 设置窗口大小为320x240
root.geometry('320x240')
root.mainloop()

# 释放摄像头资源
cap.release()
cv2.destroyAllWindows()
