import cv2
import os
from tkinter import *
from PIL import Image, ImageTk

save_path_base = './video_images'
if not os.path.exists(save_path_base):
    os.makedirs(save_path_base)
save_path_num = len(os.listdir(save_path_base))
save_path = os.path.join(save_path_base, f'images_{save_path_num+1}')
if not os.path.exists(save_path):
    os.makedirs(save_path)

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise ValueError("Can't find camera")

is_saving = False
frame_count = 0

root = Tk()
root.title("Preview")

lmain = Label(root)
lmain.pack()

def show_frame():
    global is_saving, frame_count
    ret, frame = cap.read()
    frame = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))
    if is_saving:
        frame_count += 1
        if frame_count % 10 == 0:
            img_name = os.path.join(save_path, f'image_{frame_count//10}.jpg')
            cv2.imwrite(img_name, frame)
    frame = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)
    lmain.after(10, show_frame)

def start_saving():
    global is_saving, frame_count
    print(f'Start saving images')
    is_saving = True
    frame_count = 0

def stop_saving():
    global is_saving
    print(f'Stop saving images')
    is_saving = False

btn_start_saving = Button(root, text="Start", width=50, command=start_saving)
btn_start_saving.pack(anchor=CENTER, expand=True)

btn_stop_saving = Button(root, text="Stop", width=50, command=stop_saving)
btn_stop_saving.pack(anchor=CENTER, expand=True)

show_frame()
root.geometry('320x240')
root.mainloop()

cap.release()
cv2.destroyAllWindows()
