from GPS_imu import *
import serial
from global_vars import *

number_of_target = 0  # 正常情况只初始化一次


class Sensor:
    def __init__(self):
        self.val = 0

    # 类似于中断中读取数据,得到真实值（反馈）
    def read(self) -> float:

        global lon_target, lat_target
        global number_of_target
        """获取实际值


        1.先从文件,路径为path,中读取经纬度,储存到lon_target,lat_target中
        2.读取当前的经纬度,偏航角yaw
        3.将一组target经纬度和当前经纬度,给到get_two_points_azimuth函数与get_two_points_distance函数
          获得前视距离和方位角
        4.将方位角与yaw角给到get_need_angle函数,得到当前真实偏角,完成read
        """
        global ser, flag, length  # ser = serial.Serial('com5', 9600, timeout=0.5) # ser = serial.Serial('com7',115200, timeout=0.5)
        datahex = ser.read(33)
        if flag:
            lat, lon = ([], [])  # 第二圈清空列表
        direction, yaw, lon, lat = DueData(datahex)
        latitude_now = lat
        longitude_now = lon
        yaw_raw = yaw
        # 从USB口读出新的数据

        # 完成 2
        yaw = anglechange(yaw_raw)
        try:
            length = get_two_points_distance(
                latitude_now[-1],
                longitude_now[-1],
                lat_target[number_of_target],
                lon_target[number_of_target],
            )
            direction = get_two_points_azimuth(
                latitude_now[-1],
                longitude_now[-1],
                lat_target[number_of_target],
                lon_target[number_of_target],
            )

            if length < 5:  # 已经跑进了
                number_of_target + 1  # 跳点,用新点算direction
                if len(lon_target) == number_of_target:
                    number_of_target = 0  # 置零开始循环
            # 如果没有跑进，用旧点跑

            # 完成3
            angle_target = get_need_angle(direction, yaw)  # 完成4

            self.val = angle_target
        except:
            pass
        print(self.val)
        return self.val
