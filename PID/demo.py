import actuator, sensor, controller, GPS_imu
import os
import serial

path_target = "../GPS/result.txt"  # 储存标记点的文件
if __name__ == "__main__":

    global lon_target, lat_target, ser
    os.system("sudo pigpiod")

    GPS_imu.get_data_lonlat(path_target)

    ser = serial.Serial(
        "/dev/ttyUSB0", 9600, timeout=0.5
    )  # ser = serial.Serial('com7',115200, timeout=0.5)

    fl_motor = actuator.Actuator(pin1_no=12)
    encoder = sensor.Sensor()
    fl_controller = controller.PID_controller(
        Kp=0.5,
        Ki=0,
        Kd=0,
        actuator=fl_motor,
        sensor=encoder,
        sample_time=1,
        lower_bound=-100,
        upper_bound=100,
    )
    fl_controller.run()
    fl_controller.set_target(0)  # 期望永远是车头朝向与目标线为重合
