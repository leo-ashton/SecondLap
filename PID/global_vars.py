import serial

ser = serial.Serial(
    "/dev/ttyUSB0", 9600, timeout=0.5
)  # ser = serial.Serial('com7',115200, timeout=0.5)
# False: 第一圈采点，不清空采点列表lon,lat;
# True: 第二圈跑点，每次解析数据前清空列表lon, lat;
flag = True
# length = 100000
