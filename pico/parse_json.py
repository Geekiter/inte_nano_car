# -*- coding: utf-8 -*-
"""
Created on Thu Aug  5 08:43:36 2021

@author: huangcw
"""


class SerialPort:
    def __init__(self, uart):
        self.data_bytes = bytearray()
        self.is_exit = False
        self.port = uart  # self.port.close()  # if not self.port.isOpen():  #     self.port.open()

    def read_data(self):
        """
        从串口读取数据，并放到缓冲区中
        """
        while not self.is_exit:
            count = self.port.any()
            if count > 0:
                rec_str = self.port.read(count)
                self.data_bytes.extend(rec_str)

    def frame_process(self):
        """
        提取json格式数据，若收到的数据中包含完整的json数据，每调用一次返回一条json数据。
        注意：仅支持一层 { }的json数据格式。
        """
        self.head = -1
        self.tail = -1
        self.head = self.data_bytes.find(b'{')
        self.tail = self.data_bytes.find(b'}')
        if (self.head != -1):  # 有json的头
            if (self.tail != -1):  # 有尾
                if (self.tail < self.head):  # 尾靠前，说明前面不是一个完整的报文，需要扔掉
                    self.data_bytes[0:self.tail + 1] = b''
                    return -2
                if (self.tail > self.head):  # 有头有尾，且头在尾前面
                    self.rtn = self.data_bytes[self.head:self.tail + 1]
                    self.data_bytes[self.head:self.tail + 1] = b''
                    return self.rtn
            # 有头没有尾，说明还没有接收完一帧，不处理
            else:
                return -1
        # 没头，说明没接收到一帧，不处理
        else:
            return -1
