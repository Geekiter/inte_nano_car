import json
# windows导入pupil_apriltags库
import platform
import queue
import time

import numpy as np

from yolov5_obj import YOLOv5Detector

if platform.system() == 'Windows':
    import pupil_apriltags as apriltag
# linux导入apriltag库
elif platform.system() == 'Linux':
    import apriltag

from parse_json import SerialPort

import threading
import cv2
import subprocess


def kill_all_video_processes():
    try:
        result = subprocess.run(['lsof', '+D', '/dev/'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if result.returncode == 0 and result.stdout:
            lines = result.stdout.splitlines()
            killed_pids = set()
            for line in lines:
                parts = line.split()
                if 'video' in parts[8]:
                    pid = parts[1]
                    if pid not in killed_pids:
                        try:
                            subprocess.run(['kill', '-9', pid], check=True)
                            killed_pids.add(pid)
                            print(f"Process {pid} using {parts[8]} has been killed.")
                        except subprocess.CalledProcessError:
                            print(f"Failed to kill process {pid}.")
        else:
            print("No video device processes found.")
    except Exception as e:
        print(f"Error occurred: {e}")


class CameraCapture:
    def __init__(self, camera_index=0, max_frames=10):
        self.max_frames = max_frames
        self.cap = cv2.VideoCapture(-1, cv2.CAP_V4L)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.frame_queue = queue.Queue(maxsize=max_frames)
        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.running = False
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def capture_frames(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                # self.switch_camera()  # 如果读取失败，尝试切换摄像头
                # continue
                break
            if not self.frame_queue.full():
                self.frame_queue.put(frame)
            while self.frame_queue.qsize() > 1:
                self.frame_queue.get()

    def start_capture(self):
        self.running = True
        self.capture_thread.start()

    def get_latest_frame(self):
        if self.frame_queue.empty():
            for _ in range(self.max_frames):
                ret, frame = self.cap.read()
                if ret:
                    self.frame_queue.put(frame)
        if not self.frame_queue.empty():
            return self.frame_queue.get()
        else:
            self.switch_camera()
            return None

    def stop_capture(self):
        self.running = False
        self.capture_thread.join()
        self.cap.release()
        cv2.destroyAllWindows()

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def read(self):
        return self.get_latest_frame()

    def switch_camera(self):

        self.cap.release()
        time.sleep(1)
        kill_all_video_processes()
        time.sleep(1)
        self.cap = cv2.VideoCapture(-1, cv2.CAP_V4L)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        time.sleep(1)
        # 测试是否切换成功
        if self.cap.isOpened():
            for i in range(self.max_frames):
                ret, frame = self.cap.read()
                if ret:
                    self.frame_queue.put(frame)
            # 重启线程
            self.capture_thread.join()
            self.capture_thread = threading.Thread(target=self.capture_frames)
            self.capture_thread.start()

        else:
            print(f"Failed to open camera")

        print(f"Switched to camera index")


class Nano:
    def __init__(self):
        serial_port = '/dev/ttyTHS1'
        baud_rate = 115200  # 波特率
        self.uart = SerialPort(serial_port, baud_rate)

        self.camera_params = np.array(
            [[1.43172312e+03, 0.00000000e+00, 6.07848210e+02], [0.00000000e+00, 1.43099923e+03, 3.28374663e+02],
             [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        self.camera = CameraCapture(camera_index=0, max_frames=10)
        self.camera.start_capture()

        t1 = threading.Thread(target=self.uart.read_data)
        t1.setDaemon(True)
        t1.start()

        # 定义橘红色的HSV范围
        self.lower_orange = np.array([0, 70, 50])
        self.upper_orange = np.array([15, 255, 255])

        # 红色的HSV范围
        # 第一个区间，接近0度
        self.lower_red1 = np.array([0, 70, 50])
        self.upper_red1 = np.array([10, 255, 255])

        # 第二个区间，接近180度
        self.lower_red2 = np.array([170, 70, 50])
        self.upper_red2 = np.array([180, 255, 255])
        # 存储最近几次检测到的位置
        self.smoothed_rectangles = []
        self.smooth_factor = 0.5  # 平滑因子
        self.smooth_max = 10  # 最大平滑次数
        self.detector = None

    def get_pico_data(self):
        json_byte = self.uart.frame_process()
        if json_byte != -1 and json_byte != -2:
            pico_data = json.loads(json_byte)
            return pico_data
        else:
            return {}

    def find_apriltags(self, img):
        detector = apriltag.Detector()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        return detector.detect(gray)

    def find_approx_red_squares(self, img):

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask_red1, mask_red2)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

        max_contour = max(contours, key=cv2.contourArea) if contours else None
        max_rectangle = None
        if max_contour is not None:
            x, y, w, h = cv2.boundingRect(max_contour)
            if w / h < 0.8 or w / h > 1.2:
                self.smoothed_rectangles = []
                return None

            if self.smoothed_rectangles:
                prev_x, prev_y, prev_w, prev_h = self.smoothed_rectangles[-1]
                w = int(self.smooth_factor * w + (1 - self.smooth_factor) * prev_w)
                h = int(self.smooth_factor * h + (1 - self.smooth_factor) * prev_h)

            self.smoothed_rectangles.append([x, y, w, h])
            if len(self.smoothed_rectangles) > self.smooth_max:
                self.smoothed_rectangles.pop(0)

            max_rectangle = [x, y, w, h]

        return max_rectangle

    def cal_zf_from_kpu(self, real_dis=13.6, manual_kpu_target=None):
        if manual_kpu_target is None:
            kpu_target = "class0"
        self.detector = YOLOv5Detector(weights='yolov5_best.engine')

        while True:
            img = self.camera.read()
            if img is None:
                print("No frame in queue")
                continue
            resize_w = img.shape[1] * 640 // img.shape[0]
            frame = cv2.resize(img, (resize_w, 640))
            frame = frame[:, (resize_w - 640) // 2:(resize_w + 640) // 2]
            predictions = self.detector.predict(frame)
            img = cv2.resize(img, (320, 240))

            x = 0
            y = 0
            w = 0
            h = 0
            conf = 0
            for pred in predictions:
                tx, ty, tw, th, tconf, cls, label = pred
                if label != kpu_target:
                    continue
                if tconf > conf:
                    conf = tconf
                    x = tx
                    y = ty
                    w = tw
                    h = th
            if conf == 0:
                print("no object")
            else:
                x = int(x * 320)
                y = int(y * 240)
                w = int(w * 320)
                h = int(h * 240)
                cv2.rectangle(img, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2), (0, 255, 0), 1)
                cv2.putText(img, kpu_target, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(img, f"conf: {conf:.2f}", (x, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                diag = np.sqrt(w ** 2 + h ** 2)
                dis = self.calculate_distance_to_tag(diag)
                zf = real_dis / dis
                cv2.putText(img, f"zf: {zf:.2f}", (x, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            cv2.imshow('USB Camera', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def cal_zf_from_color(self, real_dis=13.6):
        time.sleep(3)
        while True:
            img = self.camera.read()
            if img is None:
                print("No frame in queue")
                continue
            rect = self.find_approx_red_squares(img)
            if rect is not None:
                x, y, w, h = rect
                cx = x + w / 2
                cy = y + h / 2
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)
                x_zf = 320 / img.shape[1]
                y_zf = 240 / img.shape[0]
                width = w * x_zf
                height = h * y_zf
                diag = np.sqrt(width ** 2 + height ** 2)
                dis = self.calculate_distance_to_tag(diag)
                zf = real_dis / dis
                cv2.putText(img, f"zf: {zf:.2f}", (x + w + 10, y + h + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0),
                            1)

            else:
                print("no red object")
            cv2.imshow('USB Camera', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.camera.release()
        cv2.destroyAllWindows()

    def cal_zf_from_tag(self, real_dis=13.6):
        time.sleep(3)
        while True:
            img = self.camera.read()
            if img is None:
                print("No frame in queue")
                continue
            apriltags = self.find_apriltags(img)
            for tag in apriltags:
                cv2.rectangle(img, (tag.corners[0][0].astype(int), tag.corners[0][1].astype(int)),
                              (tag.corners[2][0].astype(int), tag.corners[2][1].astype(int)), (0, 255, 0), 1)

                width = tag.corners[2][0] - tag.corners[0][0]
                height = tag.corners[2][1] - tag.corners[0][1]
                x_zf = 320 / img.shape[1]
                y_zf = 240 / img.shape[0]
                width = width * x_zf
                height = height * y_zf
                diag = np.sqrt(width ** 2 + height ** 2)
                dis = self.calculate_distance_to_tag(diag)
                zf = real_dis / dis
                cv2.putText(img, f"zf: {zf:.2f}", (tag.corners[0][0].astype(int), tag.corners[0][1].astype(int) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow('USB Camera', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.camera.release()
        cv2.destroyAllWindows()

    def get_tag_dis_by_zf(self, zf):
        time.sleep(3)
        while True:
            img = self.camera.read()
            apriltags = self.find_apriltags(img)
            for tag in apriltags:
                cv2.rectangle(img, (tag.corners[0][0].astype(int), tag.corners[0][1].astype(int)),
                              (tag.corners[2][0].astype(int), tag.corners[2][1].astype(int)), (0, 255, 0), 1)

                width = tag.corners[2][0] - tag.corners[0][0]
                height = tag.corners[2][1] - tag.corners[0][1]
                x_zf = 320 / img.shape[1]
                y_zf = 240 / img.shape[0]
                width = width * x_zf
                height = height * y_zf
                diag = np.sqrt(width ** 2 + height ** 2)
                dis = self.calculate_distance_to_tag(diag)
                real_dis = - zf * - dis
                cv2.putText(img, f"real_dis: {real_dis:.2f}",
                            (tag.corners[0][0].astype(int), tag.corners[0][1].astype(int) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow('USB Camera', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.camera.release()
        cv2.destroyAllWindows()

    def get_color_dis_by_zf(self, zf):
        time.sleep(3)
        while True:
            img = self.camera.read()
            rect = self.find_approx_red_squares(img)
            if rect is not None:
                x, y, w, h = rect
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)
                x_zf = 320 / img.shape[1]
                y_zf = 240 / img.shape[0]
                width = w * x_zf
                height = h * y_zf

                diag = np.sqrt(width ** 2 + height ** 2)
                dis = self.calculate_distance_to_tag(diag)
                real_dis = zf * dis
                cv2.putText(img, f"real_dis: {real_dis:.2f}", (x + w + 10, y + h + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 1)

            else:
                print("no red object")
            cv2.imshow('USB Camera', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.camera.release()
        cv2.destroyAllWindows()

    def get_kpu_dis_by_zf(self, zf):
        self.detector = YOLOv5Detector(weights='yolov5_best.engine')
        while True:
            img = self.camera.read()
            resize_w = img.shape[1] * 640 // img.shape[0]
            frame = cv2.resize(img, (resize_w, 640))
            frame = frame[:, (resize_w - 640) // 2:(resize_w + 640) // 2]
            predictions = self.detector.predict(frame)
            img = cv2.resize(img, (320, 240))

            x = 0
            y = 0
            w = 0
            h = 0
            conf = 0
            for pred in predictions:
                tx, ty, tw, th, tconf, cls, label = pred
                if label != "class0":
                    continue
                if tconf > conf:
                    conf = tconf
                    x = tx
                    y = ty
                    w = tw
                    h = th
            if conf == 0:
                print("no object")
            else:
                x = int(x * 320)
                y = int(y * 240)
                w = int(w * 320)
                h = int(h * 240)
                cv2.rectangle(img, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2), (0, 255, 0), 1)
                cv2.putText(img, f"class0", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(img, f"conf: {conf:.2f}", (x, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                diag = np.sqrt(w ** 2 + h ** 2)
                dis = self.calculate_distance_to_tag(diag)
                real_dis = zf * dis
                cv2.putText(img, f"real_dis: {real_dis:.2f}", (x, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0),
                            1)

            cv2.imshow('USB Camera', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.camera.release()
        cv2.destroyAllWindows()

    # 计算相机到tag的距离
    def calculate_distance_to_tag(self, tag_width_pixel):
        focal_length = self.camera_params[0][0]
        distance = focal_length / tag_width_pixel
        return distance

    def run(self, manual_mode=None, manual_tag_id=None, manual_kpu_target=None):
        if manual_kpu_target is None:
            kpu_target = "class0"
        img_mode = "kpu"
        find_tag_id = 20
        self.detector = YOLOv5Detector(weights='yolov5_best.engine')

        while True:
            uart_send_data = {}
            uart_send_data["TagStatus"] = "none"
            uart_send_data["ObjectStatus"] = "none"

            pico_data = self.get_pico_data()
            if pico_data != {}:
                print(pico_data)
            if pico_data.get("img_mode", None) is not None:
                img_mode = pico_data.get("img_mode", None)

            uart_send_data["img_mode"] = img_mode

            if manual_mode is not None:
                img_mode = manual_mode

            img = self.camera.read()
            if img is None:
                print("No frame in queue")
                continue

            if img_mode == "find_apriltags":
                if pico_data.get("find_tag_id", None) is not None:
                    find_tag_id = pico_data.get("find_tag_id", None)
                    uart_send_data["find_tag_id"] = find_tag_id
                if manual_tag_id is not None:
                    find_tag_id = manual_tag_id
                apriltags = self.find_apriltags(img)  # defaults to TAG36H11
                for tag in apriltags:
                    if tag.tag_id != find_tag_id:
                        continue
                    cv2.rectangle(img, (tag.corners[0][0].astype(int), tag.corners[0][1].astype(int)),
                                  (tag.corners[2][0].astype(int), tag.corners[2][1].astype(int)), (0, 255, 0), 1)
                    cv2.putText(img, f"ID: {tag.tag_id}",
                                (tag.corners[0][0].astype(int), tag.corners[0][1].astype(int)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    uart_send_data["TagStatus"] = "get"
                    uart_send_data["TagId"] = tag.tag_id
                    cx = tag.center[0]
                    cy = tag.center[1]
                    width = tag.corners[2][0] - tag.corners[0][0]
                    height = tag.corners[2][1] - tag.corners[0][1]
                    diag = np.sqrt(width ** 2 + height ** 2)

                    x_zf = 320 / img.shape[1]
                    y_zf = 240 / img.shape[0]
                    width = width * x_zf
                    height = height * y_zf
                    cy = cy * y_zf
                    cx = cx * x_zf
                    img = cv2.resize(img, (320, 240))

                    dis = self.calculate_distance_to_tag(diag)

                    uart_send_data["TagCx"] = cx
                    uart_send_data["TagCy"] = cy
                    uart_send_data["TagWidth"] = width
                    uart_send_data["TagHeight"] = height
                    uart_send_data["TagTz"] = dis
            elif img_mode == "find_blobs":
                rect = self.find_approx_red_squares(img)
                if rect is not None:
                    cv2.rectangle(img, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)

                    x_zf = 320 / img.shape[1]
                    y_zf = 240 / img.shape[0]
                    rect = [rect[0] * x_zf, rect[1] * y_zf, rect[2] * x_zf, rect[3] * y_zf]

                    cx = rect[0] + rect[2] / 2
                    cy = rect[1] + rect[3] / 2

                    width = rect[2]
                    height = rect[3]

                    img = cv2.resize(img, (320, 240))

                    diag = np.sqrt(width ** 2 + height ** 2)
                    dis = self.calculate_distance_to_tag(diag)
                    uart_send_data["ObjectZ"] = dis
                    uart_send_data["ObjectStatus"] = "get"
                    uart_send_data["ObjectX"] = cx
                    uart_send_data["ObjectY"] = cy
                    uart_send_data["ObjectZ"] = dis
                    uart_send_data["ObjectWidth"] = rect[2]
                    uart_send_data["ObjectHeight"] = rect[3]
            elif img_mode == "kpu":
                # 高度变为640，宽度按比例缩放，然后居中裁剪成640*640
                resize_w = img.shape[1] * 640 // img.shape[0]
                frame = cv2.resize(img, (resize_w, 640))
                frame = frame[:, (resize_w - 640) // 2:(resize_w + 640) // 2]
                predictions = self.detector.predict(frame)

                img = cv2.resize(img, (320, 240))

                x = 0
                y = 0
                w = 0
                h = 0
                conf = 0
                for pred in predictions:
                    tx, ty, tw, th, tconf, cls, label = pred
                    if label != kpu_target:
                        continue
                    if tconf > conf:
                        conf = tconf
                        x = tx
                        y = ty
                        w = tw
                        h = th
                if conf == 0:
                    uart_send_data["ObjectStatus"] = "none"
                else:

                    x = int(x * 320)
                    y = int(y * 240)
                    w = int(w * 320)
                    h = int(h * 240)
                    cv2.rectangle(img, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2), (0, 255, 0), 1)
                    cv2.putText(img, kpu_target, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(img, f"conf: {conf:.2f}", (x, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    diag = np.sqrt(w ** 2 + h ** 2)
                    dis = self.calculate_distance_to_tag(diag)
                    uart_send_data["ObjectStatus"] = "get"
                    uart_send_data["ObjectX"] = x
                    uart_send_data["ObjectY"] = y
                    uart_send_data["ObjectZ"] = dis
                    uart_send_data["ObjectWidth"] = w
                    uart_send_data["ObjectHeight"] = h

            uart_send_data['ImageWidth'] = img.shape[1]
            uart_send_data['ImageHeight'] = img.shape[0]

            uart_send_data_json = json.dumps(uart_send_data)

            self.uart.port.write(uart_send_data_json.encode())  # 数据回传

            img = cv2.resize(img, (320, 240))
            cv2.imshow('USB Camera', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # 如果按下'q'键，则退出循环
                break

        # 完成所有操作后，释放捕获器
        # self.camera.release()
        cv2.destroyAllWindows()
