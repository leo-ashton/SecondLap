import pigpio
import time

servo = pigpio.pi()  # 舵机
motor = pigpio.pi()  # 电机

if not servo.connected and motor.connected:
    exit()
motor.set_servo_pulsewidth(5, 0)  # 恒定速度


class Actuator:
    def __init__(self, pin1_no):
        """执行器对象的初始化
        Args:
            pin_no (int): 执行器的引脚编号
        """
        servo_gpio = pin1_no
        pass

    def write(self, value):
        """写入执行器

        Args:
            value (int): 写入执行器的值,输入为角度，转换宽度
        """
        pulsewidth = value * 7.8 + 1500
        servo.set_servo_pulsewidth(pulsewidth)
        # raise NotImplementedError("完成执行器的write方法!")
