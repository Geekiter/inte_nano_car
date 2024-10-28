# 画面

from maix import camera, display, image, nn, uart, touchscreen

mode = 'dis'
real_dis = 11  # cm, small
zf = 1.82

# 常量
families = image.ApriltagFamilies.TAG36H11
find_tag_id = None
img = None
# 常量：不固定
img_mode = 'kpu'
detector = nn.YOLOv5(model="/root/models/maixhub/156565/model_156565.mud", dual_buff=True)
all_img_mode = ['kpu', 'find_apriltags', 'find_blobs']
# 初始化：外设
ts = touchscreen.TouchScreen()

width = 480
height = 280
k210_width = 160
k210_height = 120
cam = camera.Camera(width=width, height=height, fps=60)
width_weight = k210_width / width
height_weight = k210_height / height

cam.skip_frames(30)  # 跳过开头的30帧,图像采集还没稳定出现奇怪的画面
cam.vflip(True)  # 垂直翻转
cam.hmirror(True)  # 水平翻转
disp = display.Display()
ports = uart.list_devices()  # 列出当前可用的串口
print('uart support ports:', ports)
device = ports[0]  # 选择第一个串口

serial = uart.UART(device, 115200)

while 1:
    # 摄像头画面读取
    img = cam.read().lens_corr(strength=1.5)  # 调整strength的值直到画面不再畸变
    apriltags = img.find_apriltags(families=families)
    for a in apriltags:
        corners = a.corners()
        for i in range(4):
            img.draw_line(corners[i][0], corners[i][1], corners[(i + 1) % 4][0], corners[(i + 1) % 4][1],
                          image.COLOR_GREEN, 2)

        cx = (a.x() + a.w() / 2) * width_weight
        cy = 120 - (a.y() + a.h() / 2) * height_weight
        cz = a.z_translation()

        if mode == 'zf':
            zf = - real_dis / cz
            print("zf:%.2f" % zf)
            img.draw_string(a.x(), a.y(), "id: " + str(a.id()) + " zf:%.2f" % zf, color=image.COLOR_RED)
        else:
            real_dis = - zf * cz
            print("real_dis:%.2f" % real_dis)
            img.draw_string(a.x(), a.y(), "id: " + str(a.id()) + " real_dis:%.2f" % real_dis, color=image.COLOR_RED)

    disp.show(img)
