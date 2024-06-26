import json
import time

from car import Car
from mqtt_obj import MQTT
from task import Task

if __name__ == '__main__':
    task_list = [
        # {"mode": "find_apriltags", "id": 20, "action": "locate"},
        # {"mode": "find_apriltags", "id": 18, "action": "locate"},
        {"mode": "kpu", "id": "duck", "action": "grab-by-kpu"},
        # {"mode": "find_blobs", "id": "yellow", "action": "grab-by-color"},
        {"mode": "find_apriltags", "id": 86, "action": "put-down"},
        # {"mode": "kpu", "id": "", "action": "grab-by-kpu"},
        # {"mode": "find_apriltags", "id": 88, "action": "put-down"},
        # {"mode": "kpu", "id": "", "action": "locate-by-kpu"},
        # {"mode": "kpu", "id": "cr", "action": "locate-by-kpu"},

        # {"mode": "find_apriltags", "id": 1, "action": "grab-by-kpu-apriltags"},
        # {"mode": "find_apriltags", "id": 88, "action": "put-down"},
        {"mode": "find_blobs", "id": "", "action": "finished"},
    ]
    option_params = {
        "wifi_ssid": "WOW10086",
        "wifi_pw": "chenkeyan",
        "mqtt_server": "192.168.31.92",
        "mqtt_topic": "task_list",
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
        # "k210_cam_offset": 85 - 160 / 2,
        # "claw_range": (90 - 75) / 2,
        # "k210_qqvga": (120, 160),
        # "k210_qvga": (240, 320),
        "current_resolution": (120, 160),
        "arm_range": 30,
        "rotate_in_front_of_obj": 5,
        # "locate_stop_dis": 48,
        "arm_up_speed": 40,
        "arm_down_speed": 20,
        # "claw_grab_len": 11,
        "claw_arm_up_len": 30,
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
        "forward_speed": 55
    }
    mqtt = MQTT(option_params['wifi_ssid'], option_params['wifi_pw'], option_params['mqtt_server'],
                option_params['mqtt_topic'])
    mqtt.init()
    mqtt.publish("msg", "init", qos=1)
    task_list = []

    while True:
        data, status = mqtt.get_data()

        if status:
            task_list = json.loads(data)
            mqtt.publish("msg", "get data", qos=1)
        else:
            mqtt.publish("msg", "no data", qos=1)
            continue

        car = Car(uart_tx=12, uart_rx=13)
        # car.test()
        task = Task(car=car, target_action=task_list, **option_params)
        task.run(only_get_uart_data=False)

        time.sleep(1)
