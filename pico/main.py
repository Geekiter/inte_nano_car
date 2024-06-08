from car import Car
from task import Task

if __name__ == '__main__':
    target_action = [
        # {"mode": "find_apriltags", "id": 20, "action": "locate"},
        # {"mode": "find_apriltags", "id": 18, "action": "locate"},
        # {"mode": "kpu", "id": "duck", "action": "grab-by-kpu"},
        # {"mode": "find_blobs", "id": "yellow", "action": "grab-by-color"},
        # {"mode": "find_apriltags", "id": 86, "action": "put-down"},
        # {"mode": "kpu", "id": "", "action": "grab-by-kpu"},
        # {"mode": "find_apriltags", "id": 88, "action": "put-down"},
        # {"mode": "kpu", "id": "", "action": "locate-by-kpu"},
        # {"mode": "kpu", "id": "cr", "action": "locate-by-kpu"},

        # {"mode": "find_apriltags", "id": 1, "action": "grab-by-kpu-apriltags"},
        # {"mode": "find_apriltags", "id": 88, "action": "put-down"},
        {"mode": "kpu", "id": "get_all_id", "action": "sign"},
        {"mode": "find_blobs", "id": "", "action": "finished"},
    ]
    option_params = {
        "wifi_ssid": "WOW10086",
        "wifi_pw": "chenkeyan",
        "mqtt_server": "192.168.31.92",
        "big_tag_id_list": [18, 20],
        "small_tag_id_list": [86, 88],
        "small_tag_zoomfactor": 2.38,
        "big_tag_zoomfactor": 11.8,
        "kpu_tag_zf": 0.19,
        "kpu_obj_zoom_factor": 8.5,
        "color_obj_zoom_factor": 8,
        # "duck_height_zoom_factor": 829.6,
        # "claw_open_len": 13.5,  # cm
        # "claw_close_len": 14,
        # "test_mode": False,
        # "k210_cam_offset": 85 - 160 / 2,  # 相机安装在机械臂上的偏移量
        # "claw_range": (90 - 75) / 2,
        # "k210_qqvga": (120, 160),
        # "k210_qvga": (240, 320),
        "current_resolution": (120, 160),
        "arm_range": 30,  # pixel 上下浮动范围
        "rotate_in_front_of_obj": 5,  # cm 在物体前方允许旋转的距离
        # "locate_stop_dis": 48,  # cm
        "arm_up_speed": 40,
        "arm_down_speed": 20,
        # "claw_grab_len": 11,
        "claw_arm_up_len": 30,  # 大于这个高度，需要抬起机械臂
        # "grab_mode": False,
        # "grab_attempted": False,
        # "grab_attempted_get_count": 0,
        # "grab_attempted_all_count": 0,
        # "put_down_obj": False,
        # "arm_up_len": 2,
        "grab_arm_up_count": 0,
        "grab_forward_count": 4,
        "grab_forward_count_origin": 4,
        # "wait_ct": 0,
        # "wait_ct_limit": 10,
        "forward_speed": 55,
        "count_in_sign": 10
    }

    car = Car(uart_tx=12, uart_rx=13)
    # car.test()
    task = Task(car=car, target_action=target_action, **option_params)
    task.run(only_get_uart_data=False)
