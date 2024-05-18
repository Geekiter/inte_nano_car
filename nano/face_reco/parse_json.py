import serial


class SerialPort:
    def __init__(self, port, buand):
        self.data_bytes = bytearray()
        self.is_exit = False
        self.port = serial.Serial(port, buand)
        self.port.close()
        if not self.port.isOpen():
            self.port.open()

    def port_open(self):
        if not self.port.isOpen():
            self.port.open()

    def port_close(self):
        self.port.close()

    def send_data(self):
        self.port.write('')

    def read_data(self):
        while not self.is_exit:
            count = self.port.inWaiting()
            if count > 0:
                rec_str = self.port.read(count)
                self.data_bytes = self.data_bytes + rec_str

    def frame_process(self):
        self.head = -1
        self.tail = -1
        self.head = self.data_bytes.find(b'{')
        self.tail = self.data_bytes.find(b'}')
        if (self.head != -1):
            if (self.tail != -1):
                if (self.tail < self.head):
                    self.data_bytes[0:self.tail + 1] = b''
                    return -2
                if (self.tail > self.head):
                    self.rtn = self.data_bytes[self.head:self.tail + 1]
                    self.data_bytes[self.head:self.tail + 1] = b''
                    return self.rtn
            else:
                return -1
        else:
            return -1
