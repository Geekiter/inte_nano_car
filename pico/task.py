import json
import time
from time import sleep


class Task:
    def __init__(self, car, target_action, **kwargs):

        self.car = car

        self.target_action = target_action
        self.target_index = 0

        self.big_tag_id_list = kwargs.get('big_tag_id_list', [18, 20])
        self.small_tag_id_list = kwargs.get('small_tag_id_list', [86, 88])

        self.small_tag_zoomfactor = kwargs.get('small_tag_zoomfactor', 2.53)
        self.big_tag_zoomfactor = kwargs.get('big_tag_zoomfactor', 11.33)
        self.kpu_tag_zf = kwargs.get('kpu_tag_zf', 1.42)
        self.kpu_obj_zoom_factor = kwargs.get('kpu_obj_zoom_factor', 797.5)
        self.color_obj_zoom_factor = kwargs.get('color_obj_zoom_factor', 2.53)
        self.duck_height_zoom_factor = kwargs.get('duck_height_zoom_factor', 638)

        self.target_img_mode = []
        for action in target_action:
            self.target_img_mode.append(action["mode"])

        self.target_id_list = []
        for action in target_action:
            self.target_id_list.append(action["id"])

        self.target_action_list = []
        for action in target_action:
            self.target_action_list.append(action["action"])

        self.claw_open_len = kwargs.get('claw_open_len', 13.5)  # cm
        self.claw_close_len = kwargs.get('claw_close_len', 14)

        self.test_mode = kwargs.get('test_mode', False)
        self.k210_cam_offset = kwargs.get('k210_cam_offset', 85 - 160 / 2)  # 相机安装在机械臂上的偏移量

        self.claw_range = kwargs.get('claw_range', (90 - 75) / 2)
        self.k210_qqvga = kwargs.get('k210_qqvga', (120, 160))
        self.k210_qvga = kwargs.get('k210_qvga', (240, 320))
        self.current_resolution = kwargs.get('current_resolution', self.k210_qvga)
        self.k210_center = kwargs.get('k210_center',
                                      self.current_resolution[1] / 2 + self.k210_cam_offset)  # QQVGA分辨率：120*160
        self.k210_y_center = kwargs.get('k210_y_center', self.current_resolution[0] / 2)

        self.arm_range = kwargs.get('arm_range', 15)  # pixel 上下浮动范围
        self.rotate_in_front_of_obj = kwargs.get('rotate_in_front_of_obj', 2)  # cm 在物体前方允许旋转的距离
        self.locate_stop_dis = kwargs.get('locate_stop_dis', 48)  # cm

        self.arm_up_speed = kwargs.get('arm_up_speed', 45)
        self.arm_down_speed = kwargs.get('arm_down_speed', 10)
        self.turn_left_speed = kwargs.get('turn_left_speed', 30)
        self.turn_right_speed = kwargs.get('turn_right_speed', 30)
        self.forward_speed = kwargs.get('forward_speed', 40)
        self.backward_speed = kwargs.get('backward_speed', 40)

        self.claw_grab_len = kwargs.get('claw_grab_len', 11)
        self.claw_arm_up_len = kwargs.get('claw_arm_up_len', 25)  # 大于这个高度，需要抬起机械臂
        self.grab_mode = kwargs.get('grab_mode', False)
        self.grab_attempted = kwargs.get('grab_attempted', False)
        self.grab_attempted_get_count = kwargs.get('grab_attempted_get_count', 0)
        self.grab_attempted_all_count = kwargs.get('grab_attempted_all_count', 0)
        self.put_down_obj = kwargs.get('put_down_obj', False)
        self.arm_up_len = kwargs.get('arm_up_len', 2)
        self.grab_arm_up_count = kwargs.get('grab_arm_up_count', 8)
        self.grab_forward_count = kwargs.get('grab_forward_count', 8)
        self.grab_forward_count_origin = kwargs.get('grab_forward_count_origin', 8)
        self.wait_ct = kwargs.get('wait_ct', 0)
        self.wait_ct_limit = kwargs.get('wait_ct_limit', 10)
        self.count_in_sign = kwargs.get('count_in_sign', 10)
        self.grab_center_x = kwargs.get('grab_center_x', 61)
        self.grab_center_y = kwargs.get('grab_center_y', 80)

        self.is_finished = False
        self.search_count = 0
        self.discovered_obj = False
        self.obj_status_none_count = 0

        self.signs_action = {
            "left": self.car.keepTurnLeft(self.turn_left_speed),
            "right": self.car.keepTurnRight(self.turn_right_speed),
        }
        self.follow_obj_timer = None

    def get_zf(self, target_id):
        if target_id in self.big_tag_id_list:
            return self.big_tag_zoomfactor
        else:
            return self.small_tag_zoomfactor

    def close_to_obj_action(self, cx, cy, claw_range_level=1.5):
        t_left = 50
        t_right = 70
        t_bottom = 20
        t_top = 50
        if t_left < cx < t_right and t_bottom < cy < t_top:
            self.car.keepForward(self.forward_speed)
        # 方向控制
        elif cx > t_right:
            print("grab, right")
            self.car.keepTurnRight(self.turn_right_speed)
            self.car.keepForward(self.forward_speed)
        elif cx < t_left:
            print("grab, left")
            self.car.keepTurnLeft(self.turn_left_speed)
            self.car.keepForward(self.forward_speed)
        # 高度控制
        elif cy < t_bottom:
            # up
            print("grab, arm up")
            self.car.armUp(self.arm_up_speed)
            self.car.keepForward(self.forward_speed)
        elif cy > t_top:
            print("grab, arm down")
            self.car.armDown(self.arm_down_speed)
            self.car.keepForward(self.forward_speed)

    def common_action_put_down(self, dis):
        if dis <= 18:
            print("put down !!!")
            for _ in range(10):
                self.car.openClaw()
            self.put_down_transition()
            self.next_target()
        else:
            self.car.keepForward(self.forward_speed)

    def common_action_locate(self, dis):
        if dis < 15:
            for _ in range(10):
                self.car.armDown(self.arm_down_speed)
        else:
            print("locate !!!")
            self.car.keepForward(self.forward_speed)

    def common_action_grab(self, dis):
        if dis <= 13:
            for _ in range(1):
                self.car.keepBackward(self.backward_speed)
            for _ in range(15):
                self.car.closeClaw()
            for _ in range(4):
                self.car.armUp(self.arm_up_speed)

            for _ in range(2 * (self.grab_forward_count - 1)):
                self.car.keepBackward(self.backward_speed)

            self.next_target()
        else:
            self.car.keepForward(self.forward_speed)

    def common_action(self, cx, cy, dis, left=50, right=70, top=20, bottom=50, action="locate"):
        print("common action, dis: ", dis)
        if left < cx < right and top < cy < bottom:
            if action == "locate":
                print("common action: locate")
                self.car.keepForward(self.forward_speed)
            elif action == "put-down":
                print("common action: put down")
                self.common_action_put_down(dis)
            elif action == "grab":
                print("common action: grab by apriltags")
                self.common_action_grab(dis)
            else:
                print("common action error: ", action)

        # 方向控制
        else:
            # 高度控制
            if cy <= top:
                # up
                print("grab, arm up")
                self.car.armUp(self.arm_up_speed)

            elif cy >= bottom:
                print("grab, arm down")
                self.car.armDown(self.arm_down_speed)

            if cx >= right:
                print("grab, right")
                self.car.keepTurnRight(self.turn_right_speed)
                self.car.keepForward(self.forward_speed)

            elif cx <= left:
                print("grab, left")
                self.car.keepTurnLeft(self.turn_left_speed)
                self.car.keepForward(self.forward_speed)



    def get_locate_action(self, tag_x, tag_y, tag_z):
        self.common_action(tag_x, tag_y, tag_z, action="locate", left=self.grab_center_x - 5,
                           right=self.grab_center_y + 5, top=self.grab_center_y - 40, bottom=self.grab_center_y - 30)

    def put_down_transition(self):
        for _ in range(10):
            self.car.keepBackward(self.backward_speed)

        for _ in range(16):
            self.car.armDown(self.arm_down_speed)
            sleep(0.3)

        # for _ in range(4):
        #     self.car.closeClaw()

    def grab_transition(self):
        for _ in range(12):
            self.car.armUp(self.arm_up_speed)
        for _ in range(4):
            self.car.keepBackward(self.backward_speed)

    def put_down_action(self, cx, cy, dis):
        self.common_action(cx, cy, dis, action="put-down", left=self.grab_center_x - 5, right=self.grab_center_y + 5,
                           top=self.grab_center_y - 40, bottom=self.grab_center_y - 30)

    def kpu_locate_action(self, x, y, obj_dis):
        print(f"obj_dis: {obj_dis}")
        if obj_dis > self.rotate_in_front_of_obj + self.claw_open_len:
            self.close_to_obj_action(x, y, claw_range_level=1.5)
        else:
            for _ in range(6):
                self.car.armDown(self.arm_down_speed)
            self.next_target()

    def get_kpu_tag_action(self, tag_x, tag_y, tag_z):
        print(f"tag_x: {tag_x}, tag_y: {tag_y}, tag_z: {tag_z}")
        if tag_x > self.k210_center + self.claw_range:
            print("right")
            self.car.keepTurnRight(self.turn_right_speed)
        elif tag_x < self.k210_center - self.claw_range:
            print("left")
            self.car.keepTurnLeft(self.turn_left_speed)
        else:
            print("grab mode")
            self.grab_mode = True

            for _ in range(4):
                self.car.armUp(self.arm_up_speed)

    def grab_by_kpu(self, cx, cy, dis):
        self.common_action(cx, cy, dis,
                           action="grab", left=self.grab_center_x - 5,
                           right=self.grab_center_y + 5,
                           top=self.grab_center_y,
                           bottom=120 - 20)

    def grab_by_apriltags(self, cx, cy, dis):
        self.common_action(cx, cy, dis,
                           action="grab", left=self.grab_center_x - 5,
                           right=self.grab_center_y + 5,
                           top=self.grab_center_y,
                           bottom=120 - 20)

    def grab_mode_in_kpu(self):
        for _ in range(1):
            self.car.keepBackward(self.backward_speed)
        for _ in range(self.grab_arm_up_count):
            self.car.armUp(self.arm_up_speed)
        for _ in range(self.grab_forward_count):
            self.car.keepForward(self.forward_speed)
        for _ in range(15):
            self.car.closeClaw()

        for _ in range(10):
            self.car.armUp(self.arm_up_speed)

        for _ in range(2 * (self.grab_forward_count - 1)):
            self.car.keepBackward(self.backward_speed)

        for _ in range(14):
            self.car.armDown(self.arm_down_speed)

    def grab_mode_in_color(self):

        for _ in range(1):
            self.car.keepBackward(self.backward_speed)
        # for _ in range(10):
        #     armUp(25)
        for _ in range(self.grab_forward_count):
            self.car.keepForward(self.forward_speed)
        for _ in range(15):
            self.car.closeClaw()

        for _ in range(12):
            self.car.armUp(self.arm_up_speed)

        for _ in range(2 * (self.grab_forward_count)):
            self.car.keepBackward(self.backward_speed)

        for _ in range(14):
            self.car.armDown(self.arm_down_speed)

    def grab_by_color(self, cx, cy, dis):
        self.common_action(cx, cy, dis,
                           action="grab", left=self.grab_center_x - 5,
                           right=self.grab_center_y + 5,
                           top=self.grab_center_y,
                           bottom=120 - 20)

    def discovered_obj_action(self):
        print("current search count: {}".format(self.search_count))
        if self.search_count < 10:
            self.car.keepTurnRight(self.turn_right_speed)
            self.search_count += 1
        elif 10 <= self.search_count < 20:
            self.car.keepTurnLeft(self.turn_left_speed)
            self.search_count += 1
        else:
            self.car.keepTurnRight(self.turn_right_speed)
        sleep(0.3)

    def get_search_count(self, cx, resolution_x):
        resolution_x_unit = resolution_x / 20
        sc = int(cx / resolution_x_unit)
        if sc < 10:
            return 20 - sc
        else:
            return sc - 10

    def next_target(self):
        self.target_index += 1
        self.discovered_obj = False

    def car_init(self):
        for _ in range(10):
            self.car.armDown(self.arm_down_speed)

        for _ in range(8):
            self.car.openClaw()

    def get_json(self, uart_data):
        try:
            uart_read = uart_data.read()
            uart_data = uart_read.decode("utf-8")
            # find { and } first appear position, then cut the string
            if "{" in uart_data and "}" in uart_data:
                uart_data = uart_data[uart_data.index("{"): uart_data.index("}") + 1]
                return json.loads(uart_data)
            else:
                return {}
        except Exception as e:
            print("get json error, original data: ", e, uart_read)
            return {}

    def validate_target_action(self):
        # is array
        if not isinstance(self.target_action, list):
            return False
        # is not empty
        if len(self.target_action) == 0:
            return False
        # item must have mode, id, action
        for item in self.target_action:
            if not isinstance(item, dict):
                return False
            if "mode" not in item or "id" not in item or "action" not in item:
                return False
        return True

    def sign_action(self, obj_id):
        if obj_id is "stop":
            self.car.stopMove()
            self.target_index += 1
        elif obj_id is "left":
            for _ in range(self.count_in_sign):
                self.car.keepTurnLeft(self.turn_left_speed)
        elif obj_id is "right":
            for _ in range(self.count_in_sign):
                self.car.keepTurnRight(self.turn_right_speed)
        elif obj_id is "turning":
            for _ in range(5 * self.count_in_sign):
                self.car.keepTurnRight(self.turn_right_speed)
        elif obj_id is "park":
            self.car.stopMove()
        else:
            print("sign action is invalid")

    def follow_obj_action(self, cx, cy, obj_dis):
        if cy < self.k210_y_center:
            self.car.armUp(self.arm_up_speed)
            sleep(0.3)
        elif cy > self.k210_y_center:
            self.car.armDown(self.arm_down_speed)
            sleep(0.3)
        if cx > self.k210_center - self.claw_range / 2:
            self.car.keepTurnRight(self.turn_right_speed)
        elif cx < self.k210_center - self.claw_range / 2:
            self.car.keepTurnLeft(self.turn_left_speed)

        self.car.keepForward(self.forward_speed)
        sleep(0.1)

    def run(self, only_get_uart_data=False):
        # if not self.validate_target_action():
        #     print("target action is invalid")
        #     return
        if not only_get_uart_data: self.car_init()
        while self.is_finished is False and not self.test_mode:
            if not self.car.uart2.any():
                continue

            if self.target_index < len(self.target_action_list):
                print(f"current target action is: {self.target_action_list[self.target_index]}")
            else:
                print("target action list is empty")
                break

            data = self.get_json(self.car.uart2)

            print(f"current data: {data}")
            if only_get_uart_data:
                continue

            k210_img_mode = data.get("img_mode", "N/A")
            uart_write_dict = {}
            tag_status = "none"
            obj_status = "none"
            tag_z = 0
            tag_cx = 0
            tag_cy = 0
            tag_id = 0
            obj_x = 0
            obj_y = 0
            obj_w = 0
            obj_h = 0
            obj_z = 0
            obj_id = 0
            # if self.target_action[self.target_index]["id"] != "":
            if self.target_img_mode[self.target_index] == "find_apriltags":
                tag_id = int(data.get("TagId", "999"))
                tag_status = data.get("TagStatus", "none")
                if tag_status == "none":
                    self.obj_status_none_count += 1
                else:
                    self.obj_status_none_count = 0
                zoomfactor = self.get_zf(tag_id)
                tag_z = zoomfactor * float(data.get("TagTz", "999"))
                tag_cx = int(data.get("TagCx", "999"))
                tag_cy = int(data.get("TagCy", "999"))
                tag_w = int(data.get("TagWidth", "999"))
                tag_h = int(data.get("TagHeight", "999"))
                find_tag_id = data.get("find_tag_id", None)
                print(f"find tag id: {find_tag_id}, target id: {self.target_id_list[self.target_index]}")
                # if find_tag_id is None or find_tag_id != self.target_id_list[self.target_index]:
                #     uart_write_dict["find_tag_id"] = self.target_id_list[self.target_index]

            else:
                obj_w = data.get("ObjectWidth", 999)
                obj_h = data.get("ObjectHeight", 999)
                obj_x = data.get("ObjectX", 0)
                obj_y = data.get("ObjectY", 0)
                obj_z = data.get("ObjectZ", 0)
                obj_id = data.get("ObjectId", 0)
                obj_status = data.get("ObjectStatus", "none")
                if obj_status == "none":
                    self.obj_status_none_count += 1
                else:
                    self.obj_status_none_count = 0
                find_tag_id = data.get("find_tag_id", None)
                print(f"find tag id: {find_tag_id}, target id: {self.target_id_list[self.target_index]}")
            if find_tag_id is None or find_tag_id != self.target_id_list[self.target_index]:
                uart_write_dict["find_tag_id"] = self.target_id_list[self.target_index]
            if k210_img_mode != self.target_img_mode[self.target_index]:
                uart_write_dict["img_mode"] = self.target_img_mode[self.target_index]

            if uart_write_dict != {}:
                print(f"write data: {uart_write_dict}")
                self.car.uart2.write(json.dumps(uart_write_dict) + "\n")

            # if target_img_mode[target_index] == "find-apriltags" and find_tag_id != target_id_list[target_index]:
            #     uart_write_dict = {"find_tag_id": target_id_list[target_index]}
            #     uart2.write(json.dumps(uart_write_dict) + "\n")
            if tag_status is "get":
                self.search_count = self.get_search_count(tag_cx, self.current_resolution[1])
                self.discovered_obj = True
            elif obj_status is "get":
                self.search_count = self.get_search_count(obj_x, self.current_resolution[1])
                self.discovered_obj = True

            if self.target_action_list[self.target_index] == "finished":
                self.is_finished = True
                for _ in range(10):
                    self.car.closeClaw()
                break
            elif self.grab_mode and self.obj_status_none_count > 10:
                print("grab mode, and obj status is: ", obj_status)
                if self.target_action_list[self.target_index] in ["grab-by-kpu", "grab-by-kpu-apriltags"]:
                    self.grab_mode_in_kpu()
                elif self.target_action_list[self.target_index] in ["grab-by-color", "grab-by-apriltags"]:
                    self.grab_mode_in_color()

                # grab_transition()

                self.grab_mode = False
                self.next_target()
                obj_status = "none"
                sleep(2)

            elif (tag_status == "none" and obj_status == "none") or (
                    tag_status == "get" and tag_id != self.target_id_list[self.target_index]):
                print("tag status is none, or tag id is not equal to target id")
                if self.discovered_obj:
                    # discovered_obj_action()
                    print(f"wait ct={self.wait_ct}")
                    self.wait_ct += 1
                    if self.wait_ct >= self.wait_ct_limit:
                        self.discovered_obj = False
                        self.wait_ct = 0
                else:
                    print("not found, right turn")
                    self.car.keepTurnRight(self.turn_right_speed - 10)
                    sleep(0.3)
            elif self.target_action_list[self.target_index] == "put-down":
                if tag_id == self.target_id_list[self.target_index]:
                    self.put_down_action(tag_cx, tag_cy, tag_z)
            elif self.target_action_list[self.target_index] == "grab-by-kpu":
                obj_dis = self.kpu_obj_zoom_factor * obj_z
                self.grab_by_kpu(obj_x, obj_y, obj_dis)
            elif self.target_action_list[self.target_index] == "grab-by-apriltags":
                self.grab_by_apriltags(tag_cx, tag_cy, tag_z)
            elif self.target_action_list[self.target_index] == "grab-by-color":
                obj_dis = self.color_obj_zoom_factor * obj_z
                self.grab_by_color(obj_x, obj_y, obj_dis)
            elif self.target_action_list[self.target_index] == "locate":
                if tag_id == self.target_id_list[self.target_index]:
                    self.get_locate_action(tag_cx, tag_cy, tag_z)
            elif self.target_action_list[self.target_index] == "grab-by-kpu-apriltags":
                self.get_kpu_tag_action(tag_cx, tag_cy, tag_z)
            elif self.target_action_list[self.target_index] == "locate-by-kpu":
                obj_dis = self.kpu_obj_zoom_factor * obj_z
                self.kpu_locate_action(obj_x, obj_y, obj_dis)
            elif self.target_action_list[self.target_index] == "sign":
                self.sign_action(obj_id)
            elif self.target_action_list[self.target_index] == "follow_obj":
                if self.follow_obj_timer is None:
                    self.follow_obj_timer = time.time()
                # after 5 minutes, target_index + 1
                elif time.time() - self.follow_obj_timer > 5 * 60:
                    self.next_target()
                    self.follow_obj_timer = None
                obj_dis = self.kpu_obj_zoom_factor * obj_z
                self.follow_obj_action(obj_x, obj_y, obj_dis)
