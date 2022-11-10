import signal

import serial

from GPS_imu import *


def terminate_signal_handler():
    save_lonlat("./result.txt", lon, lat)
    exit(0)


if __name__ == "__main__":
    ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)
    # signal.signal(signal.SIGUSR1, terminate_signal_handler)
    print(ser.is_open)
    try:
        while 1:
            datahex = ser.read(33)
            DueData(datahex)
            #  print(lon,lat)
    except KeyboardInterrupt:
        save_lonlat("./result.txt", lon, lat)
