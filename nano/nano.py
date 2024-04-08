import json
# windows导入pupil_apriltags库
import platform
import threading

import cv2
import numpy as np

if platform.system() == 'Windows':
    import pupil_apriltags as apriltag
# linux导入apriltag库
elif platform.system() == 'Linux':
    import apriltag

from parse_json import SerialPort


class Nano:
    def __init__(self):
        serial_port = '/dev/ttyTHS1'
        baud_rate = 115200  # 波特率
        self.uart = SerialPort(serial_port, baud_rate)
        fx = 396.943
        fy = 391.173
        cx = 159.771
        cy = 98.5957
        camera_params = [fx, fy, cx, cy]  # 相机焦距和主点

        self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
        # 设置摄像头参数
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # start a thread that constantly reads data from the uart
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

    def cal_zf_from_color(self, real_dis=13.6):
        while True:
            ret, img = self.camera.read()
            rect = self.find_approx_red_squares(img)
            if rect is not None:
                x, y, w, h = rect
                cx = x + w / 2
                cy = y + h / 2
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)
                zf_w = real_dis * w
                zf_h = real_dis * h
                cv2.putText(img, f"zf_w: {zf_w:.2f}", (x + w + 10, y + h + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 1)
                cv2.putText(img, f"zf_h: {zf_h:.2f}", (x + w + 10, y + h + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 1)

            else:
                print("no red object")
            cv2.imshow('USB Camera', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.camera.release()
        cv2.destroyAllWindows()

    def run(self):
        while True:
            uart_send_data = {}
            uart_send_data["TagStatus"] = "none"
            uart_send_data["ObjectStatus"] = "none"

            pico_data = self.get_pico_data()
            print("pico_data: ", pico_data)
            # img_mode = pico_data.get("img_mode", "find_blobs")
            img_mode = "find_blobs"
            find_target_id = pico_data.get("find_target_id", None)

            ret, img = self.camera.read()
            # img = cv2.resize(img, (img.shape[1] // 4, img.shape[0] // 4))

            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                continue

            if img_mode == "find_apriltags":
                apriltags = self.find_apriltags(img)  # defaults to TAG36H11
                for tag in apriltags:
                    # if tag.id() != find_target_id:
                    #     continue

                    cv2.rectangle(img, (tag.corners[0][0], tag.corners[0][1]), (tag.corners[2][0], tag.corners[2][1]),
                                  (0, 255, 0), 2)

                    uart_send_data["TagStatus"] = "get"
                    uart_send_data["TagId"] = tag.tag_id
                    uart_send_data["TagTz"] = tag.z()
                    uart_send_data["TagCx"] = tag.cx()
                    uart_send_data["TagCy"] = tag.cy()
            elif img_mode == "find_blobs":
                rect = self.find_approx_red_squares(img)
                if rect is not None:
                    cv2.rectangle(img, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)
                    cx = rect[0] + rect[2] / 2
                    cy = rect[1] + rect[3] / 2
                    uart_send_data["ObjectStatus"] = "get"
                    uart_send_data["ObjectCx"] = cx
                    uart_send_data["ObjectCy"] = cy
                    uart_send_data["ObjectWidth"] = rect[2]
                    uart_send_data["ObjectHeight"] = rect[3]
            uart_send_data['ImageWidth'] = img.shape[1]
            uart_send_data['ImageHeight'] = img.shape[0]

            uart_send_data_json = json.dumps(uart_send_data)

            self.uart.port.write(uart_send_data_json.encode())  # 数据回传

            cv2.imshow('USB Camera', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # 如果按下'q'键，则退出循环
                break

        # 完成所有操作后，释放捕获器
        self.camera.release()
        cv2.destroyAllWindows()
