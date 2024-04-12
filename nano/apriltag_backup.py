#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import select
import rospy
import tf
import numpy as np
import serial
import tf.transformations as tfm
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad
import json
from std_msgs.msg import String
from datetime import datetime
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Initialize the ROS node
rospy.init_node('camera_state_estimator', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()

# Define the robot's parameters
wheel_radius = 1  # Wheel radius in meters
wheel_base = 1  # Distance between wheels in meters

# Define motor speed mapping factors
max_linear_speed = 1.0  # Max linear speed of the robot in m/s
max_angular_speed = np.pi / 4  # Max angular speed of the robot in rad/s

# Linear and angular displacement mapping to motor speed
k_linear = 65535 / max_linear_speed  # For linear displacement
# For angular displacement
k_angular = 65535 / (max_angular_speed * wheel_base)

g_x = 0
g_y = 0
g_z = 0

color = 'red'
tag_id = 3
mode = ""
connected = False

# Initialize the serial port for robot control
ser = serial.Serial(
    port='/dev/ttyTHS1',
    baudrate=115200,
    timeout=1
)


# Function to clamp values within a range
def get_lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return None

def urat_send(urat_send_data_json):
    if ser.isOpen():
        # print(urat_send_data_json)
        bytes_written = ser.write(urat_send_data_json + '\n')
        # if bytes_written > 0:
        #     print("Data sent successfully")
        # else:
        #     print("No data was sent")


# Function to send commands to the robot


# Callback function to handle AprilTag pose messages
def get_camera_to_pose(tagName):
    try:
        (trans, rot) = lr.lookupTransform('/usb_cam', tagName, rospy.Time(0))
        return np.array(trans), np.array(rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        # rospy.loginfo("Camera pose could not be obtained")
        return None, None


# OpenCV和ROS的桥梁
bridge = CvBridge()

# 定义蓝色的HSV范围
lower_blue = np.array([110, 50, 50])
upper_blue = np.array([130, 255, 255])

# 定义橘红色的HSV范围
lower_orange = np.array([0, 70, 50])
upper_orange = np.array([15, 255, 255])

# 红色的HSV范围
# 第一个区间，接近0度
lower_red1 = np.array([0, 70, 50])
upper_red1 = np.array([10, 255, 255])

# 第二个区间，接近180度
lower_red2 = np.array([170, 70, 50])
upper_red2 = np.array([180, 255, 255])

# 图像接收和处理的回调函数
count = 0
accumulated_data = {"ObjectX": 0, "ObjectY": 0, "ObjectCX": 0, "ObjectCY": 0, "ObjectWidth": 0, "ObjectHeight": 0}


def image_callback(ros_image):
    global ser, count, accumulated_data
    global g_x, g_y, g_z, mode
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")

    try:
        if color != '':
            urat_send_data = {}
            # 新增一个检测到的色块计数器
            detected_blocks = 0
            # 将接收到的ROS图像消息转为OpenCV格式
            # cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
            # 图像预处理
            # blurred_image = cv2.GaussianBlur(cv_image, (5, 5), 0)
            # hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

            # 将BGR图像转换为HSV颜色空间
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            if color == 'orange':
                # 根据颜色范围创建掩膜
                mask = cv2.inRange(hsv, lower_orange, upper_orange)
            elif color == 'red':
                # 创建红色掩膜
                mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
                mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)

                # 合并掩膜
                mask = cv2.bitwise_or(mask_red1, mask_red2)
            elif color == 'blue':
                mask = cv2.inRange(hsv, lower_blue, upper_blue)

            # 形态学操作改善掩码
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # 轮廓检测
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

            # 使用轮廓面积找到最大的轮廓
            max_contour = max(contours, key=cv2.contourArea) if contours else None

            # 如果存在最大轮廓，对其进行处理
            if max_contour is not None:
                # 获得色块的矩形边界
                x, y, w, h = cv2.boundingRect(max_contour)
                # if w >= 20 and h >= 20:  # 保证色块的大小超过阈值
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.drawMarker(cv_image, (x + w // 2, y + h // 2), (0, 255, 0), cv2.MARKER_CROSS)

                # 更新累积数据
                accumulated_data["ObjectX"] += x
                accumulated_data["ObjectY"] += y
                accumulated_data["ObjectCX"] += x + int(w / 2)
                accumulated_data["ObjectCY"] += y + int(h / 2)
                accumulated_data["ObjectWidth"] += w
                accumulated_data["ObjectHeight"] += h

                # 检测到的色块计数器只需增加1，因为我们只处理了最大的轮廓
                detected_blocks = 1

            # 如果检测到色块，则更新计数器
            if detected_blocks > 0:
                desktop_path = '/home/nvidia/mqtt_ws/src/mqtt/robot/cv2_image/'
                image_name = 'processed_image.jpg'  # 保存的图片名
                # 完整的保存路径
                save_path = desktop_path + image_name
                cv2.imwrite(save_path, cv_image)
                count += 1

            # 每10次累计后发送平均值
            if count >= 10 and 'ObjectX' in accumulated_data:
                for key in accumulated_data.keys():
                    accumulated_data[key] /= count * detected_blocks  # 计算平均值
                accumulated_data["mode"] = mode
                accumulated_data["ObjectStatus"] = "get"
                urat_send_data_json = json.dumps(accumulated_data)
                urat_send(urat_send_data_json)

                # 重置计数器和累加器
                count = 0
                accumulated_data = {key: 0 for key in accumulated_data}

        if tag_id != -1:
            # print(g_x, g_y, g_z)
            g_x_int = int(160 - g_x)  # 将计算结果转换为整数
            g_y_int = int(120 - g_y)  # 将计算结果转换为整数
            cv2.drawMarker(cv_image, (g_x_int, g_y_int), (0, 255, 0), cv2.MARKER_CROSS)

        # 在ROS节点中处理完成后的图像，例如通过ROS主题发布或者用于导航逻辑
        # 你可以根据你的需要来修改下面的代码
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print(e)


# Main function

nanoIp = get_lan_ip()
nanoPort = 3000

# 创建socket对象
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 绑定端口号
server_socket.bind((nanoIp, nanoPort))
server_socket.setblocking(0)  # Set the socket to non-blocking
# 设置最大连接数，超过后排队
server_socket.listen(5)

def main():
    global target_position, target_orientation
    global g_x, g_y, g_z
    global color, tag_id
    global nanoIp, nanoPort, server_socket, connected, mode
    target_position = np.array([0, 0, 0.025])
    target_orientation = np.array([0.5, -0.5, -0.5, 0.5])

    # Subscribe to the AprilTag pose topic
    # rospy.Subscriber('/apriltags/points', String, rosImageVizCallback)

    # 订阅相机数据主题
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    rospy.sleep(1)  # Wait for subscribers to get ready

    # Set the rate of the loop
    rate = rospy.Rate(10)  # 10Hz

    # # 等待客户端连接
    # client_socket, addr = server_socket.accept()
    # print("连接地址: " + addr)

    while not rospy.is_shutdown():

        read_sockets, _, _ = select.select([server_socket], [], [], 0)
        for sock in read_sockets:
            print(sock)
            if sock is server_socket:
                # print(True)
                client, address = sock.accept()
                # print(client)

                # 接收客户端信息
                data = client.recv(4096) #.decode('utf-8')
                # data = str(data, "utf8")
                print("收到消息：" + data)
                if data == "connected":
                    connected = True
                    print("connected")
                datas = data.split(":")
                if datas[0] == "grab-by-color":
                    color = 'orange' if datas[1] == '' else datas[1]
                    tag_id = -1
                    mode = datas[2]
                elif datas[0] == "put-down":
                    color = ''
                    tag_id = int(datas[1])
                    mode = datas[2]
                    print("/tag_"+str(tag_id))
                print('mode', mode, color, tag_id)


        if not connected:
            urat_send(json.dumps({
                "nanoIp": nanoIp,
                "nanoProt": nanoPort
            }))


        if tag_id == -1:
            continue


        # Get the current camera pose
        current_trans, current_rot = get_camera_to_pose("/tag_"+str(tag_id))
        # print(current_trans)
        if current_trans is None:
            rospy.logerr("/tag_"+str(tag_id)+" Current camera pose could not be obtained.")
            urat_send(json.dumps({}))
            continue

        # Calculate displacements
        displacement = target_position - current_trans

        z_dis = displacement[2] * 1000
        x_dis = displacement[0] * 1000
        y_dis = displacement[1] * 1000
        duration_left = 1
        duration_right = -10000
        # only send the raw data to pico
        # send_to_car(z_dis, x_dis, y_dis, duration_right)

        if g_x == x_dis and g_z == z_dis and g_y == y_dis:
            continue
        else:
            g_x = x_dis
            g_z = z_dis
            g_y = y_dis
        urat_send_data = {}

        urat_send_data["mode"] = mode
        urat_send_data["TagId"] = tag_id
        urat_send_data["TagTx"] = x_dis
        urat_send_data["TagTy"] = y_dis
        urat_send_data["TagTz"] = z_dis
        urat_send_data['TagCx'] = x_dis
        urat_send_data['TagCy'] = y_dis

        urat_send_data_json = json.dumps(urat_send_data)
        urat_send(urat_send_data_json)

        # rospy.loginfo("Driving towards AprilTag...")
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close()
