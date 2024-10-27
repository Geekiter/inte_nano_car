# 画面
import json

from maix import camera, display, image, nn, uart, touchscreen, app

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
k210_width =160
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


def on_uart_received(serial: uart.UART, data: bytes):
    global img_mode, find_tag_id
    uart_json = {}
    read_str = data.decode("utf-8")
    if "{" in read_str and "}" in read_str:
        read_str = read_str[read_str.index("{"): read_str.index("}") + 1]
        uart_json = json.loads(read_str)
    print("uart received data: ", data)

    if "img_mode" in uart_json:
        img_mode = uart_json["img_mode"]
        if img_mode not in all_img_mode:
            img_mode = all_img_mode[0]

        print("uart img_mode:", img_mode)
    if "find_tag_id" in uart_json:
        find_tag_id = uart_json.get("find_tag_id", None)


serial.set_received_callback(on_uart_received)
# 屏幕
exit_label = "< Exit"
size = image.string_size(exit_label)
exit_btn_pos = [0, 0, 8 * 2 + size.width(), 12 * 2 + size.height()]


def is_in_button(x_t, y_t, btn_pos):
    return btn_pos[0] < x_t < btn_pos[0] + btn_pos[2] and btn_pos[1] < y_t < btn_pos[1] + btn_pos[3]


while 1:
    # TODO 内存管理
    uart_send_data = {}
    # 摄像头画面读取
    img = cam.read()
    img = img.lens_corr(strength=1.5)  # 调整strength的值直到画面不再畸变
    img.draw_string(8, 12, exit_label, image.COLOR_WHITE)
    img.draw_rect(exit_btn_pos[0], exit_btn_pos[1], exit_btn_pos[2], exit_btn_pos[3], image.COLOR_WHITE, 2)
    # 按键管理
    x, y, pressed = ts.read()
    print("x, y, pressed", x, y, pressed)
    if is_in_button(x, y, exit_btn_pos):
        app.set_exit_flag(True)
    # apriltag
    uart_send_data["find_tag_id"] = find_tag_id
    uart_send_data['img_mode'] = img_mode
    if img_mode == 'find_apriltags':
        apriltags = img.find_apriltags(families=families)
        for a in apriltags:
            if a.id() != find_tag_id:
                continue
            corners = a.corners()
            for i in range(4):
                img.draw_line(corners[i][0], corners[i][1], corners[(i + 1) % 4][0], corners[(i + 1) % 4][1],
                              image.COLOR_GREEN, 2)
            img.draw_string(a.x(), a.y(), "id: " + str(a.id()), color=image.COLOR_RED)

            uart_send_data['TagStatus'] = 'get'
            uart_send_data['TagId'] = a.id()
            uart_send_data['TagCx'] = (a.x() + a.w() / 2) * width_weight
            uart_send_data['TagCy'] = (a.y() + a.h() / 2) * height_weight
            uart_send_data['TagTz'] = - a.z_translation()
            uart_send_data['TagWidth'] = a.w() * width_weight
            uart_send_data['TagHeight'] = a.h() * height_weight

    elif img_mode == 'kpu':
        # TODO duck
        objs = detector.detect(img, conf_th=0.5, iou_th=0.45)
        for obj in objs:
            img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_RED)
            msg = f'{detector.labels[obj.class_id]}: {obj.score:.2f}'
            img.draw_string(obj.x, obj.y, msg, color=image.COLOR_RED)
            uart_send_data['ObjectId'] = detector.labels[obj.class_id]
            uart_send_data['ObjectX'] = (obj.x + obj.w / 2) * width_weight
            uart_send_data['ObjectY'] = (obj.y + obj.h / 2) * height_weight
            uart_send_data['ObjectWidth'] = obj.w * width_weight
            uart_send_data['ObjectHeight'] = obj.h * height_weight
            uart_send_data['ObjectZ'] = 1 / obj.w * width_weight
            uart_send_data['ObjectStatus'] = 'get'
    uart_send_data_json = json.dumps(uart_send_data)
    print("uart send data: ", uart_send_data_json)
    serial.write_str(uart_send_data_json)
    disp.show(img)
