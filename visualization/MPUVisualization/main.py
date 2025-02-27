import time

from vpython import *
import numpy as np
import serial


def main():
    arduino_serial = serial.Serial('com3', 115200)
    time.sleep(1)

    scene.range = 5
    to_rad = 2*np.pi/360
    to_deg = 1/to_rad
    scene.forward = vector(0, -10, -10)

    scene.width = 600
    scene.height = 600

    angles = label(pos=vec(-10, 10, 1), text='roll: 0\npitch: 0\nyaw: 0', align='center',
                   color=color.white, height=30, depth=0)

    xarrow = arrow(length=10, shaftwidth=.1, color=color.red, axis=vector(1, 0, 0))
    yarrow = arrow(length=10, shaftwidth=.1, color=color.green, axis=vector(0, 1, 0))
    zarrow = arrow(length=10, shaftwidth=.1, color=color.blue, axis=vector(0, 0, 1))

    front_arrow = arrow(length=10, shaftwidth=.1, color=color.purple, axis=vector(1, 0, 0))
    up_arrow = arrow(length=8, shaftwidth=.1, color=color.magenta, axis=vector(0, 1, 0))
    side_arrow = arrow(length=11, shaftwidth=.1, color=color.orange, axis=vector(0, 0, 1))

    arduino = box(length=5, width=6, height=1.5, pos=vector(0, 0, 0),
                  color=color.blue, opacity=.8)
    bp = box(length=3.5, width=4.5, height=1, pos=vector(0, 0.5*arduino.height + 0.5, -0.5),
             color=color.white, opacity=.8)
    mpu = box(length=1.5, width=2.5, height=.3, pos=vector(0, 0.5*arduino.height + bp.height + 0.15, -0.5),
              color=color.cyan, opacity=.8)
    controller = compound([arduino, bp, mpu])
    controller.axis = front_arrow.axis
    controller.up = up_arrow.axis

    while True:
        try:
            while arduino_serial.in_waiting <= 4:
                pass
            data_packet = arduino_serial.readline()
            data_packet = str(data_packet, 'utf-8')
            split_packet = data_packet.split(",")
            roll = float(split_packet[0]) * to_rad
            pitch = float(split_packet[1]) * to_rad
            yaw = float(split_packet[2]) * to_rad

            angles.text = f'roll: {round(roll*to_deg)}\npitch: {round(pitch*to_deg)}\nyaw: {round(yaw*to_deg)}'

            rate(50)
            k = vector(cos(-yaw) * cos(-pitch), sin(-pitch), sin(-yaw) * cos(-pitch))

            y = vector(0, 1, 0)
            s = cross(k, y)
            v = cross(s, k)

            front_arrow.axis = k
            front_arrow.length = 8
            side_arrow.axis = s
            side_arrow.length = 9
            up_arrow.axis = v
            up_arrow.length = 6

            controller.axis = k
            controller.up = v
        except:
            pass


if __name__ == '__main__':
    main()
