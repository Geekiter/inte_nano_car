from time import sleep

from machine import Pin, PWM, UART

from motorPCA9685 import MotorDriver


class Car:
    def __init__(self, uart_tx=16, uart_rx=17):

        self.target_index = 0

        # claw
        self.inb1 = Pin(10, Pin.OUT)
        self.inb2 = Pin(9, Pin.OUT)
        self.pwmb = PWM(Pin(11))
        self.pwmb.freq(1000)

        self.uart2 = UART(0, baudrate=115200, tx=Pin(uart_tx), rx=Pin(uart_rx))

        self.m = MotorDriver()
        # ## arm
        self.ina1 = Pin(8, Pin.OUT)
        self.ina2 = Pin(7, Pin.OUT)
        self.pwma = PWM(Pin(6))
        self.pwma.freq(1000)

        # ## grab claw
        self.maxGrabLevel = 70
        self.minGrabLevel = 30
        self.grabLastState = 2
        self.grabState = 0  # 0 unknown, 1, close, 2, open
        self.currentGrabLevel = 70

    def RotateBCCW(self, duty):
        self.inb1.value(0)
        self.inb2.value(1)
        duty_16 = int((duty * 65536) / 100)
        self.pwmb.duty_u16(duty_16)

    def RotateBCW(self, duty):
        self.inb1.value(1)
        self.inb2.value(0)
        duty_16 = int((duty * 65536) / 100)
        self.pwmb.duty_u16(duty_16)

    def RotateACW(self, duty):
        self.ina1.value(1)
        self.ina2.value(0)
        duty_16 = int((duty * 65536) / 100)
        self.pwma.duty_u16(duty_16)

    def RotateACCW(self, duty):
        self.ina1.value(0)
        self.ina2.value(1)
        duty_16 = int((duty * 65536) / 100)
        self.pwma.duty_u16(duty_16)

    def StopMotor(self):
        self.ina1.value(0)
        self.ina2.value(0)
        self.pwma.duty_u16(0)
        self.inb1.value(0)
        self.inb2.value(0)
        self.pwmb.duty_u16(0)

    def openClaw(self):

        if self.grabLastState != 1:
            self.currentGrabLevel = self.maxGrabLevel

        else:
            self.currentGrabLevel = max(self.currentGrabLevel - 10, self.minGrabLevel)

        print(f"open Claw at {self.currentGrabLevel}")
        self.RotateBCCW(self.currentGrabLevel)
        sleep(0.5)
        self.StopMotor()
        sleep(0.2)
        self.grabLastState = 1

    def closeClaw(self):
        if self.grabLastState != 2:
            self.currentGrabLevel = self.maxGrabLevel

        else:
            self.currentGrabLevel = max(self.currentGrabLevel - 10, self.minGrabLevel)

        print(f"close Claw at {self.currentGrabLevel}")

        self.RotateBCW(self.currentGrabLevel)
        sleep(0.5)
        self.StopMotor()
        sleep(0.2)
        self.grabLastState = 2

    def moveForwardSpd(self, speedpct1):
        self.m.MotorRunInstant("MA", "forward", speedpct1)
        self.m.MotorRunInstant("MB", "forward", speedpct1)
        self.m.MotorRunInstant("MC", "forward", speedpct1)
        self.m.MotorRunInstant("MD", "forward", speedpct1)

    def moveBackwardSpd(self, speedpct1):
        self.m.MotorRunInstant("MA", "backward", speedpct1)
        self.m.MotorRunInstant("MB", "backward", speedpct1)
        self.m.MotorRunInstant("MC", "backward", speedpct1)
        self.m.MotorRunInstant("MD", "backward", speedpct1)

    def rotateLeftSpd(self, speedpct1):
        self.m.MotorRunInstant("MA", "backward", speedpct1)
        self.m.MotorRunInstant("MB", "forward", speedpct1)
        self.m.MotorRunInstant("MC", "backward", speedpct1)
        self.m.MotorRunInstant("MD", "forward", speedpct1)

    def rotateRightSpd(self, speedpct1):
        self.m.MotorRunInstant("MA", "forward", speedpct1)
        self.m.MotorRunInstant("MB", "backward", speedpct1)
        self.m.MotorRunInstant("MC", "forward", speedpct1)
        self.m.MotorRunInstant("MD", "backward", speedpct1)

    def stopMove(self, ):
        self.m.MotorStop("MA")
        self.m.MotorStop("MB")
        self.m.MotorStop("MC")
        self.m.MotorStop("MD")

    def parallelLeft(self, speedpct):
        self.m.MotorRunInstant("MA", "forward", speedpct)
        self.m.MotorRunInstant("MB", "backward", speedpct)
        self.m.MotorRunInstant("MC", "backward", speedpct)
        self.m.MotorRunInstant("MD", "forward", speedpct)

    def parallelRight(self, speedpct):
        self.m.MotorRunInstant("MA", "backward", speedpct)
        self.m.MotorRunInstant("MB", "forward", speedpct)
        self.m.MotorRunInstant("MC", "forward", speedpct)
        self.m.MotorRunInstant("MD", "backward", speedpct)

    def armUp(self, duty_cycle):
        # arm up
        self.RotateACW(duty_cycle)
        sleep(0.1)
        self.StopMotor()
        sleep(0.1)

    def armDown(self, duty_cycle):
        # arm down
        self.RotateACCW(duty_cycle)
        sleep(0.05)
        self.StopMotor()
        sleep(0.2)

    def keepForward(self, sp, count=3, decrease=5):
        for i in range(count):
            self.moveForwardSpd(sp - i * decrease)
        self.stopMove()

    def keepTurnLeft(self, sp, count=4, decrease=3):
        for i in range(count):
            self.rotateLeftSpd(sp - i * decrease)
        self.stopMove()

    def keepTurnRight(self, sp, count=4, decrease=3):
        for i in range(count):
            self.rotateRightSpd(sp - i * decrease)
        self.stopMove()

    def keepBackward(self, sp, count=3, decrease=5):
        for i in range(count):
            self.moveBackwardSpd(sp - i * decrease)
        self.stopMove()

    def test(self):
        # for _ in range(5):
        #     self.armUp(30)
        # for _ in range(12):
        #     self.armDown(10)
        #
        # for _ in range(5):
        #     self.armUp(30)

        for _ in range(6):
            self.closeClaw()
        for _ in range(6):
            self.openClaw()
        for _ in range(6):
            self.closeClaw()

        # for _ in range(6):
        #     print("right")
        #     self.keepTurnRight(50)
        #     sleep(0.5)
        #
        # for _ in range(6):
        #     print("left")
        #     self.keepTurnLeft(50)
        #     sleep(0.5)
        #
        # for _ in range(6):
        #     print("forward")
        #     self.keepForward(50)
        #     sleep(0.5)
        #
        # for _ in range(6):
        #     print("backward")
        #     self.keepBackward(50)
        #     sleep(0.5)
