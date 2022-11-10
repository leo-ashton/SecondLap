import threading
import time

from simple_pid import PID

import actuator
import sensor


class PID_controller(PID):
    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        actuator: actuator.Actuator,
        sensor: sensor.Sensor,
        sample_time: float,
        lower_bound: float,
        upper_bound: float,
    ) -> None:
        """初始化PID控制器

        Args:
            Kp (float): 控制器Kp的值
            Ki (float): 控制器Ki的值
            Kd (float): 控制器Kd的值
            actuator (actuator.Actuator): 执行器对象
            sensor (sensor.Sensor): 传感器对象
            sample_time (float): 采样时间
            lower_bound (float): 输出下限
            upper_bound (float): 输出上限
        """
        PID.__init__(
            self,
            Kp=Kp,
            Ki=Ki,
            Kd=Kd,
            sample_time=sample_time,
            output_limits=(lower_bound, upper_bound),
        )
        self.actuator = actuator
        self.sample_time = sample_time
        self.sensor = sensor
        self.output_pin = actuator
        self.sensor_update_event = threading.Event()

    def sensor_update(self):
        self.feedback = self.sensor.read()

    def main_sensor_update_thread(self):
        while True:
            self.sensor_update()
            self.sensor_update_event.set()
            time.sleep(self.sample_time)

    def actuator_update(self, value):
        self.actuator.write(value)

    def main_actuator_update_thread(self):
        while True:
            if self.sensor_update_event.is_set():
                new_val = self(self.sensor.read())
                print(new_val)
                self.actuator_update(new_val)
                self.sensor_update_event.clear()

            else:
                time.sleep(self.sample_time / 2)

    def run(self):
        self.actuator_update_thread = threading.Thread(
            target=self.main_actuator_update_thread
        )
        self.sensor_update_thread = threading.Thread(
            target=self.main_sensor_update_thread
        )
        self.sensor_update_thread.start()
        self.actuator_update_thread.start()

    def set_target(self, target: float):
        self.setpoint = target
