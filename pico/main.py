from car import Car
from task import Task

if __name__ == '__main__':
    target_action = [
        {"mode": "find_apriltags", "id": 86, "action": "grab-by-apriltags"},
        {"mode": "find_apriltags", "id": 94, "action": "put-down"},
        {"mode": "find_apriltags", "id": 86, "action": "grab-by-apriltags"},
        {"mode": "find_apriltags", "id": 94, "action": "put-down"},
        {"mode": "find_apriltags", "id": 86, "action": "grab-by-apriltags"},
        {"mode": "find_apriltags", "id": 94, "action": "put-down"},
        {"mode": "find_apriltags", "id": 86, "action": "grab-by-apriltags"},
        {"mode": "find_apriltags", "id": 94, "action": "put-down"},
        {"mode": "find_apriltags", "id": 86, "action": "grab-by-apriltags"},
        {"mode": "find_apriltags", "id": 94, "action": "put-down"},
        {"mode": "find_blobs", "id": "", "action": "finished"},
    ]
    option_params = {
        "wifi_ssid": "WOW10086",
        "wifi_pw": "chenkeyan",
        "mqtt_server": "192.168.31.92",
        "big_tag_id_list": [18, 20],
        "small_tag_id_list": [86, 88, 94],
        "small_tag_zoomfactor": 1.35,
        "big_tag_zoomfactor": 11.8,
        "kpu_tag_zf": 0.19,
        "kpu_obj_zoom_factor": 8.5,
        "color_obj_zoom_factor": 4.5,

        # "duck_height_zoom_factor": 829.6,
        # "claw_open_len": 13.5,  # cm
        # "claw_close_len": 14,
        # "test_mode": False,
        # "claw_range": (90 - 75) / 2,
        # "k210_qqvga": (120, 160),
        # "k210_qvga": (240, 320),
        "current_resolution": (120, 160),
        # "arm_range": 30,  # pixel range
        # "rotate_in_front_of_obj": 5.5,  # cm allow rotate in front of obj
        # "locate_stop_dis": 48,  # cm
        "arm_up_speed": 40,
        "arm_down_speed": 20,
        # "claw_grab_len": 11,
        "claw_arm_up_len": 30,  # when arm up, the height of claw
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
        "forward_speed": 60,
        "count_in_sign": 10,
        "turn_right_speed": 50,
        "turn_left_speed": 50,
        "grab_center_x": 61,
        "grab_center_y": 84,

    }

    car = Car(uart_tx=12, uart_rx=13)
    # car.test()
    task = Task(car=car, target_action=target_action, **option_params)
    task.run(only_get_uart_data=False)
