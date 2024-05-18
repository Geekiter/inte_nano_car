import os
from tkinter import *

import cv2
from PIL import Image, ImageTk

# 创建保存图片的目录
save_path = './pic'
if not os.path.exists(save_path):
    os.makedirs(save_path)

# 初始化摄像头
cap = cv2.VideoCapture(0)

# 检查摄像头是否成功打开
if not cap.isOpened():
    raise ValueError("Unable to open camera")


# 截取图片并保存的函数
def capture_image():
    # 读取当前帧
    ret, frame = cap.read()
    if ret:
        # 构造图片保存的路径
        img_name = os.path.join(save_path, 'capture_{}.png'.format(len(os.listdir(save_path))))
        # 保存图片
        cv2.imwrite(img_name, frame)
        print(f'Image has been saved as {img_name}')


# 创建Tkinter窗口
root = Tk()
root.title("Preview")

# 创建一个Label用于显示摄像头帧
lmain = Label(root)
lmain.pack()

# 创建截取按钮
btn_capture = Button(root, text="Capture", width=50, command=capture_image)
btn_capture.pack(anchor=CENTER, expand=True)


# 更新帧的函数
def show_frame():
    # 从摄像头读取帧
    _, frame = cap.read()
    frame = cv2.resize(frame, (frame.shape[1] // 4, frame.shape[0] // 4))
    # 将帧转换为Tkinter可以使用的格式
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)
    # 每隔10毫秒更新一次帧
    lmain.after(10, show_frame)


# 开始显示帧
show_frame()
# 启动Tkinter事件循环
# 设置窗口大小为320x240
root.geometry('320x240')
root.mainloop()

# 释放摄像头资源
cap.release()
cv2.destroyAllWindows()
