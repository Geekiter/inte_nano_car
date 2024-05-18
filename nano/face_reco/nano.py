import json
# windows导入pupil_apriltags库
import platform
import queue
import time

import face_recognition
import numpy as np

if platform.system() == 'Windows':
    import pupil_apriltags as apriltag
# linux导入apriltag库
elif platform.system() == 'Linux':
    import apriltag

from parse_json import SerialPort

import threading
import cv2
import subprocess
import os


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

    def get_pico_data(self):
        json_byte = self.uart.frame_process()
        if json_byte != -1 and json_byte != -2:
            pico_data = json.loads(json_byte)
            return pico_data
        else:
            return {}

    def calculate_distance_to_tag(self, tag_width_pixel):
        focal_length = self.camera_params[0][0]
        distance = focal_length / tag_width_pixel
        return distance

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
        face_known_path = "/home/nvidia/nanocar/face_known/"
        known_face_encodings = []
        known_face_names = []
        for file in os.listdir(face_known_path):
            if file.endswith(".png") or file.endswith(".jpg"):
                image = face_recognition.load_image_file(face_known_path + file)
                face_encoding = face_recognition.face_encodings(image)[0]
                known_face_encodings.append(face_encoding)
                known_face_names.append(file.split(".")[0])

        # Initialize some variables
        face_locations = []
        face_encodings = []
        face_names = []
        process_this_frame = True
        while True:
            if manual_kpu_target is None:
                kpu_target = "cr"
            else:
                kpu_target = manual_kpu_target
            img = self.camera.read()
            frame = img
            if process_this_frame:
                # Resize frame of video to 1/4 size for faster face recognition processing
                # small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
                small_frame = frame
                print(small_frame.shape)

                # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_reco uses)
                rgb_small_frame = cv2.cvtColor(small_frame[:, :, ::-1], cv2.COLOR_BGR2RGB)

                # Find all the faces and face encodings in the current frame of video
                face_locations = face_recognition.face_locations(rgb_small_frame)
                face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

                face_names = []
                for face_encoding in face_encodings:
                    # See if the face is a match for the known face(s)
                    matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                    name = "Unknown"

                    # # If a match was found in known_face_encodings, just use the first one.
                    # if True in matches:
                    #     first_match_index = matches.index(True)
                    #     name = known_face_names[first_match_index]

                    # Or instead, use the known face with the smallest distance to the new face
                    face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                    best_match_index = np.argmin(face_distances)
                    if matches[best_match_index]:
                        name = known_face_names[best_match_index]

                    face_names.append(name)

            process_this_frame = not process_this_frame

            # Display the results
            for (top, right, bottom, left), name in zip(face_locations, face_names):
                # Scale back up face locations since the frame we detected in was scaled to 1/4 size
                if name != kpu_target:
                    continue

                # top *= 4
                # right *= 4
                # bottom *= 4
                # left *= 4

                target_width = 320
                target_height = 240
                current_width = frame.shape[1]
                current_height = frame.shape[0]
                x_zf = target_width / current_width
                y_zf = target_height / current_height

                # Draw a box around the face
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

                # Draw a label with a name below the face
                cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
                if name != kpu_target:
                    continue
                x = (left + right) // 2 * x_zf
                y = (top + bottom) // 2 * y_zf
                w = (right - left) * x_zf
                h = (bottom - top) * y_zf
                diag = np.sqrt(w ** 2 + h ** 2)
                dis = self.calculate_distance_to_tag(diag)
                zf = real_dis / dis
                print(f"zf: {zf}")
                cv2.putText(frame, f"zf: {zf}", (left + 6, bottom + 30), font, 1.0, (255, 255, 255), 1)
            frame = cv2.resize(frame, (320, 240))
            cv2.imshow('USB Camera', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    def get_kpu_dis_by_zf(self, zf):
        face_known_path = "/home/nvidia/nanocar/face_known/"
        known_face_encodings = []
        known_face_names = []
        for file in os.listdir(face_known_path):
            if file.endswith(".png") or file.endswith(".jpg"):
                image = face_recognition.load_image_file(face_known_path + file)
                face_encoding = face_recognition.face_encodings(image)[0]
                known_face_encodings.append(face_encoding)
                known_face_names.append(file.split(".")[0])

        # Initialize some variables
        face_locations = []
        face_encodings = []
        face_names = []
        process_this_frame = True
        while True:
            img = self.camera.read()
            frame = img
            if process_this_frame:
                # Resize frame of video to 1/4 size for faster face recognition processing
                # small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
                small_frame = frame
                print(small_frame.shape)

                # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_reco uses)
                rgb_small_frame = cv2.cvtColor(small_frame[:, :, ::-1], cv2.COLOR_BGR2RGB)

                # Find all the faces and face encodings in the current frame of video
                face_locations = face_recognition.face_locations(rgb_small_frame)
                face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

                face_names = []
                for face_encoding in face_encodings:
                    # See if the face is a match for the known face(s)
                    matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                    name = "Unknown"

                    # # If a match was found in known_face_encodings, just use the first one.
                    # if True in matches:
                    #     first_match_index = matches.index(True)
                    #     name = known_face_names[first_match_index]

                    # Or instead, use the known face with the smallest distance to the new face
                    face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                    best_match_index = np.argmin(face_distances)
                    if matches[best_match_index]:
                        name = known_face_names[best_match_index]

                    face_names.append(name)

            process_this_frame = not process_this_frame

            # Display the results
            for (top, right, bottom, left), name in zip(face_locations, face_names):
                # Scale back up face locations since the frame we detected in was scaled to 1/4 size

                # top *= 4
                # right *= 4
                # bottom *= 4
                # left *= 4

                target_width = 320
                target_height = 240
                current_width = frame.shape[1]
                current_height = frame.shape[0]
                x_zf = target_width / current_width
                y_zf = target_height / current_height

                # Draw a box around the face
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

                # Draw a label with a name below the face
                cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
                if name != "cr":
                    continue
                x = (left + right) // 2 * x_zf
                y = (top + bottom) // 2 * y_zf
                w = (right - left) * x_zf
                h = (bottom - top) * y_zf
                diag = np.sqrt(w ** 2 + h ** 2)
                dis = self.calculate_distance_to_tag(diag)
                real_dis = zf * dis
                print(f"real_dis: {real_dis}")
                cv2.putText(frame, f"real_dis: {real_dis}", (left + 6, bottom + 30), font, 1.0, (255, 255, 255), 1)
            frame = cv2.resize(frame, (320, 240))
            cv2.imshow('USB Camera', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def run(self, manual_mode=None, manual_tag_id=None, manual_kpu_target=None):

        img_mode = "kpu"
        find_tag_id = 20
        # self.detector = YOLOv5Detector(weights='yolov5_best.engine')

        # Load a sample picture and learn how to recognize it.
        face_known_path = "/home/nvidia/nanocar/face_known/"
        known_face_encodings = []
        known_face_names = []
        for file in os.listdir(face_known_path):
            if file.endswith(".png") or file.endswith(".jpg"):
                image = face_recognition.load_image_file(face_known_path + file)
                face_encoding = face_recognition.face_encodings(image)[0]
                known_face_encodings.append(face_encoding)
                known_face_names.append(file.split(".")[0])

        # Initialize some variables
        face_locations = []
        face_encodings = []
        face_names = []
        process_this_frame = True

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
                if pico_data.get("find_tag_id", None) is not None:
                    manual_kpu_target = pico_data.get("find_tag_id", None)
                    uart_send_data["find_tag_id"] = manual_kpu_target
                if manual_kpu_target is None:
                    kpu_target = "messi"
                if manual_kpu_target is not None:
                    kpu_target = manual_kpu_target
                frame = img
                if process_this_frame:
                    # Resize frame of video to 1/4 size for faster face recognition processing
                    # small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
                    small_frame = frame
                    print(small_frame.shape)

                    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_reco uses)
                    rgb_small_frame = cv2.cvtColor(small_frame[:, :, ::-1], cv2.COLOR_BGR2RGB)

                    # Find all the faces and face encodings in the current frame of video
                    face_locations = face_recognition.face_locations(rgb_small_frame)
                    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

                    face_names = []
                    for face_encoding in face_encodings:
                        # See if the face is a match for the known face(s)
                        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                        name = "Unknown"

                        # # If a match was found in known_face_encodings, just use the first one.
                        # if True in matches:
                        #     first_match_index = matches.index(True)
                        #     name = known_face_names[first_match_index]

                        # Or instead, use the known face with the smallest distance to the new face
                        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                        best_match_index = np.argmin(face_distances)
                        if matches[best_match_index]:
                            name = known_face_names[best_match_index]

                        face_names.append(name)

                process_this_frame = not process_this_frame

                # Display the results
                for (top, right, bottom, left), name in zip(face_locations, face_names):
                    # Scale back up face locations since the frame we detected in was scaled to 1/4 size

                    # top *= 4
                    # right *= 4
                    # bottom *= 4
                    # left *= 4

                    target_width = 320
                    target_height = 240
                    current_width = frame.shape[1]
                    current_height = frame.shape[0]
                    x_zf = target_width / current_width
                    y_zf = target_height / current_height

                    # Draw a box around the face
                    cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

                    # Draw a label with a name below the face
                    cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                    font = cv2.FONT_HERSHEY_DUPLEX
                    cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
                    if name != kpu_target:
                        continue
                    x = (left + right) // 2 * x_zf
                    y = (top + bottom) // 2 * y_zf
                    w = (right - left) * x_zf
                    h = (bottom - top) * y_zf
                    diag = np.sqrt(w ** 2 + h ** 2)
                    dis = self.calculate_distance_to_tag(diag)
                    uart_send_data["ObjectStatus"] = "get"
                    uart_send_data["ObjectX"] = x
                    uart_send_data["ObjectY"] = y
                    uart_send_data["ObjectZ"] = dis
                    uart_send_data["ObjectWidth"] = w
                    uart_send_data["ObjectHeight"] = h

                # time.sleep(1)

            img = cv2.resize(img, (320, 240))

            uart_send_data['ImageWidth'] = img.shape[1]
            uart_send_data['ImageHeight'] = img.shape[0]

            uart_send_data_json = json.dumps(uart_send_data)
            print(uart_send_data_json)

            self.uart.port.write(uart_send_data_json.encode())  # 数据回传

            cv2.imshow('USB Camera', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # 如果按下'q'键，则退出循环
                break

        # 完成所有操作后，释放捕获器
        # self.camera.release()
        cv2.destroyAllWindows()
