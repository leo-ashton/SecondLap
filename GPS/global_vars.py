Q_info = [1.0, 0.0, 0.0, 0.0]
I_ex, I_ey, I_ez = (0.0, 0.0, 0.0)
icm_kp = 50.0  # 加速度计的收敛速率比例增益
icm_ki = 0.2  # 陀螺仪收敛速率的积分增益
ACCData = [0.0] * 8
GYROData = [0.0] * 8
AngleData = [0.0] * 8
MAGData = [0.0] * 8
Lonlatdata = [0.0] * 8
FrameState = 0  # 通过0x后面的值判断属于哪一种情况
Bytenum = 0  # 读取到这一段的第几位
CheckSum = 0  # 求和校验位


delta_T = 0.002  # 采样周期
acc = [0.0] * 3
gyro = [0.0] * 3
Angle = [0.0] * 3
mag = [0.0] * 3
eulerAngle = [0.0] * 3
# 存放的经纬度列表
lon = []
lat = []
