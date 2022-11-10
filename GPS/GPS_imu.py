import math
from global_vars import *

def DueData(inputdata):  # 新增的核心程序，对读取的数据进行划分，各自读到对应的数组里
    global FrameState  # 在局部修改全局变量，要进行global的定义
    global Bytenum
    global CheckSum
    global acc, gyro, Angle, mag, lon, lat
    for data in inputdata:  # 在输入的数据进行遍历
        # Python2软件版本这里需要插入 data = ord(data)*****************************************************************************************************
        if FrameState == 0:  # 当未确定状态的时候，进入以下判断
            if data == 0x55 and Bytenum == 0:  # 0x55位于第一位时候，开始读取数据，增大bytenum
                CheckSum = data
                Bytenum = 1
                continue
            elif data == 0x51 and Bytenum == 1:  # 在byte不为0 且 识别到 0x51 的时候，改变frame
                CheckSum += data
                FrameState = 1
                Bytenum = 2
            elif data == 0x53 and Bytenum == 1:
                CheckSum += data
                FrameState = 3
                Bytenum = 2
            elif data == 0x57 and Bytenum == 1:
                CheckSum += data
                FrameState = 7
                Bytenum = 2

        elif FrameState == 1:  # acc  #已确定数据代表加速度
            if Bytenum < 10:  # 读取8个数据
                ACCData[Bytenum - 2] = data  # 从0开始
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xFF):  # 假如校验位正确
                    acc = get_acc(ACCData)
                CheckSum = 0  # 各数据归零，进行新的循环判断
                Bytenum = 0
                FrameState = 0
        elif FrameState == 3:  # angle
            if Bytenum < 10:
                AngleData[Bytenum - 2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xFF):
                    gyro = get_angle(AngleData)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0
        elif FrameState == 7:  # lonlat
            if Bytenum < 10:
                Lonlatdata[Bytenum - 2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xFF):
                    get_lonlat(Lonlatdata)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0


# 加速度


def get_acc(datahex):
    axl = datahex[0]
    axh = datahex[1]
    ayl = datahex[2]
    ayh = datahex[3]
    azl = datahex[4]
    azh = datahex[5]
    k_acc = 16.0

    acc_x = (axh << 8 | axl) / 32768.0 * k_acc
    acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
    acc_z = (azh << 8 | azl) / 32768.0 * k_acc
    if acc_x >= k_acc:
        acc_x -= 2 * k_acc
    if acc_y >= k_acc:
        acc_y -= 2 * k_acc
    if acc_z >= k_acc:
        acc_z -= 2 * k_acc

    return acc_x, acc_y, acc_z


# 角加速度


def get_gyro(datahex):
    wxl = datahex[0]
    wxh = datahex[1]
    wyl = datahex[2]
    wyh = datahex[3]
    wzl = datahex[4]
    wzh = datahex[5]
    k_gyro = 2000.0

    gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
    gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
    gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
    if gyro_x >= k_gyro:
        gyro_x -= 2 * k_gyro
    if gyro_y >= k_gyro:
        gyro_y -= 2 * k_gyro
    if gyro_z >= k_gyro:
        gyro_z -= 2 * k_gyro
    return gyro_x, gyro_y, gyro_z


# 角度


def get_angle(datahex):
    rxl = datahex[0]
    rxh = datahex[1]
    ryl = datahex[2]
    ryh = datahex[3]
    rzl = datahex[4]
    rzh = datahex[5]
    k_angle = 180.0

    angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >= k_angle:
        angle_z -= 2 * k_angle

    return angle_x, angle_y, angle_z


# 磁力计


def get_mag(datahex):
    hxl = datahex[0]
    hxh = datahex[1]
    hyl = datahex[2]
    hyh = datahex[3]
    hzl = datahex[4]
    hzh = datahex[5]

    mag_x = hxh << 8 | hxl
    mag_y = hyh << 8 | hyl
    mag_z = hzh << 8 | hzl
    if (65565 - mag_x) < mag_x:
        mag_x -= 65535
    if (65565 - mag_y) < mag_y:
        mag_y -= 65535
    if (65565 - mag_z) < mag_z:
        mag_z -= 65535
    return mag_x, mag_y, mag_z


# 经纬度


def get_lonlat(datahex):
    global lat, lon
    Lon0 = datahex[0]
    Lon1 = datahex[1]
    Lon2 = datahex[2]
    Lon3 = datahex[3]
    Lat0 = datahex[4]
    Lat1 = datahex[5]
    Lat2 = datahex[6]
    Lat3 = datahex[7]
    lon_t = (Lon3 << 24) | (Lon2 << 16) | (Lon1 << 8) | Lon0
    lat_t = (Lat3 << 24) | (Lat2 << 16) | (Lat1 << 8) | Lat0
    lat.append(lat_t / 10000000)
    lon.append(lon_t / 10000000)


def get_q(datahex):
    Q0L = datahex[0]
    Q0H = datahex[1]
    Q1L = datahex[2]
    Q1H = datahex[3]
    Q2L = datahex[4]
    Q2H = datahex[5]
    Q3L = datahex[6]
    Q3H = datahex[7]
    q0 = ((Q0H << 8) | Q0L) / 32768
    q1 = ((Q1H << 8) | Q1L) / 32768
    q2 = ((Q2H << 8) | Q2L) / 32768
    q3 = ((Q3H << 8) | Q3L) / 32768
    return q0, q1, q2, q3


def IMU_AHRSupdate_withMagnetic(acc, gyro, mag):
    global yaw
    global Q_info, I_ex, I_ey, I_ez
    global icm_ki, icm_kp, delta_T

    norm = math.sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2])
    ax = acc[0] / norm
    ay = acc[1] / norm
    az = acc[2] / norm

    norm = math.sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2])
    gx = gyro[0] / norm
    gy = gyro[1] / norm
    gz = gyro[2] / norm

    norm = math.sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2])
    mx = mag[0] / norm
    my = mag[1] / norm
    mz = mag[2] / norm

    q0, q1, q2, q3 = (Q_info[0], Q_info[1], Q_info[2], Q_info[3])

    hx = (
        2 * mx * (0.5 - q2 * q2 - q3 * q3)
        + 2 * my * (q1 * q2 - q0 * q3)
        + 2 * mz * (q1 * q3 + q0 * q2)
    )
    hy = (
        2 * mx * (q1 * q2 + q0 * q3)
        + 2 * my * (0.5 - q1 * q1 - q3 * q3)
        + 2 * mz * (q2 * q3 - q0 * q1)
    )
    hz = (
        2 * mx * (q1 * q3 - q0 * q2)
        + 2 * my * (q2 * q3 + q0 * q1)
        + 2 * mz * (0.5 - q1 * q1 - q2 * q2)
    )

    bx = math.sqrt((hx * hx) + (hy * hy))
    bz = hz

    vx = 2 * (q1 * q3 - q0 * q2)
    vy = 2 * (q0 * q1 + q2 * q3)
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

    wx = 2 * bx * (0.5 - q2 * q2 - q3 * q3) + 2 * bz * (q1 * q3 - q0 * q2)
    wy = 2 * bx * (q1 * q2 - q0 * q3) + 2 * bz * (q0 * q1 + q2 * q3)
    wz = 2 * bx * (q0 * q2 + q1 * q3) + 2 * bz * (0.5 - q1 * q1 - q2 * q2)

    ex = (ay * vz - az * vy) + (my * wz - mz * wy)
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz)
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx)

    if ex != 0.0 and ey != 0.0 and ez != 0.0:
        I_ex += delta_T * ex
        I_ey += delta_T * ey
        I_ez += delta_T * ez

        gx = gx + icm_kp * ex + icm_ki * I_ex
        gy = gy + icm_kp * ey + icm_ki * I_ey
        gz = gz + icm_kp * ez + icm_ki * I_ez

    halfT = 0.5 * delta_T
    delta_2 = (
        (2 * halfT * gx) * (2 * halfT * gx)
        + (2 * halfT * gy) * (2 * halfT * gy)
        + (2 * halfT * gz) * (2 * halfT * gz)
    )

    q0 = (1 - delta_2 / 8) * q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT
    q1 = (1 - delta_2 / 8) * q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT
    q2 = (1 - delta_2 / 8) * q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT
    q3 = (1 - delta_2 / 8) * q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT

    norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    Q_info[0] = q0 / norm
    Q_info[1] = q1 / norm
    Q_info[2] = q2 / norm
    Q_info[3] = q3 / norm

    # pitch = math.asin(2 * q0*q2 - 2 * q1*q3) * 180 / math.pi
    # roll = math.atan2(2 * q2*q3 + 2 * q0*q1, -2 * q1*q1 - 2 * q2*q2 + 1) * 180 / math.pi
    yaw = math.degrees(
        math.atan2(
            2 * Q_info[1] * Q_info[2] + 2 * Q_info[0] * Q_info[3],
            -2 * Q_info[2] * Q_info[2] - 2 * Q_info[3] * Q_info[3] + 1,
        )
    )


def get_two_points_distance(latitude1, longitude1, latitude2, longitude2):
    EARTH_RADIUS = 6378137

    rad_latitude1 = math.radians(latitude1)
    rad_latitude2 = math.radians(latitude2)
    rad_longitude1 = math.radians(longitude1)
    rad_longitude2 = math.radians(longitude2)

    a = rad_latitude1 - rad_latitude2
    b = rad_longitude1 - rad_longitude2

    distance = 2 * math.asin(
        math.sqrt(
            pow(math.sin(a / 2), 2)
            + math.cos(rad_latitude1)
            * math.cos(rad_latitude2)
            * pow(math.sin(b / 2), 2)
        )
    )
    return distance * EARTH_RADIUS


def get_two_points_azimuth(latitude1, longitude1, latitude2, longitude2):
    latitude1 = math.radians(latitude1)
    latitude2 = math.radians(latitude2)
    longitude1 = math.radians(longitude1)
    longitude2 = math.radians(longitude2)

    x = math.sin(longitude2 - longitude1) * math.cos(latitude2)
    y = math.cos(latitude1) * math.sin(latitude2) - math.sin(latitude1) * math.cos(
        latitude2
    ) * math.cos(longitude2 - longitude1)
    angle = math.degrees(math.atan2(x, y))
    return angle if (angle > 0) else (angle + 360)


# 文件地址，经度纬度


def save_lonlat(file, lon, lat):
    str1 = ""
    for i in range(len(lon)):
        str1 += str(lon[i]) + "," + str(lat[i]) + "\n"
    with open(file, "w") as f:
        f.write(str1)


# # 例子
# lon = [1,2,3,4,5,6,7]
# lat = [1,2,3,4,5,6,7]
# save_lonlat("1.txt",lon,lat)
