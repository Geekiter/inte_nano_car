import asyncio

from lib.mqtt_as import MQTTClient, config


class MQTT:
    def __init__(self, ssid, pw, server, topic="task_list"):
        config['ssid'] = ssid
        config['wifi_pw'] = pw
        config['server'] = server
        config['connect_coro'] = self.conn_han
        config['subs_cb'] = self.callback
        self.client = MQTTClient(config)
        MQTTClient.DEBUG = True

        self.task_data = ""
        self.topic = topic

    def init(self):
        asyncio.run(self.client.connect())

    def publish(self, topic, msg, qos=1):
        asyncio.run(self.client.publish(topic, msg, qos=qos))

    def callback(self, topic, msg, retained):
        # print((topic, msg, retained))
        self.task_data = msg.decode()
        print(self.task_data)

    def get_data(self):
        data = self.task_data
        if data:
            self.task_data = ""
            return data, True
        else:
            return data, False

    async def conn_han(self, client):
        await client.subscribe(self.topic, 1)
