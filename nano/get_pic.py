import os
from tkinter import *

import cv2
from PIL import Image, ImageTk

save_path = './pic'
if not os.path.exists(save_path):
    os.makedirs(save_path)

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise ValueError("Unable to open camera")


def capture_image():
    ret, frame = cap.read()
    if ret:
        img_name = os.path.join(save_path, 'capture_{}.png'.format(len(os.listdir(save_path))))
        cv2.imwrite(img_name, frame)
        print(f'Image has been saved as {img_name}')


root = Tk()
root.title("Preview")

lmain = Label(root)
lmain.pack()

btn_capture = Button(root, text="Capture", width=50, command=capture_image)
btn_capture.pack(anchor=CENTER, expand=True)


# 更新帧的函数
def show_frame():
    _, frame = cap.read()
    frame = cv2.resize(frame, (frame.shape[1] // 4, frame.shape[0] // 4))
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)
    lmain.after(10, show_frame)


show_frame()
root.geometry('320x240')
root.mainloop()

# 释放摄像头资源
cap.release()
cv2.destroyAllWindows()
