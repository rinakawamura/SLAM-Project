#!/usr/bin/env python
################
# Your pwm_gen.py from Assignment #1
# You need to add the CAN-messaging functionality as explained in the lecture note
################
import rospy
import can
import struct

from assignment2_team3.msg import Int16Stamped

def rescale(v):
    # scale from (-100,100) to (0.1*2**16, 0.2*2**16)
    v += 100
    v *= 2**16 / 200.0 / 10.0
    v += 0.1 * 2**16
    return int(v)

throttle = rescale(0)
steering = rescale(0)

def throttle_callback(data):
    global throttle
    throttle = rescale(data.value)


def steering_callback(data):
    global steering
    # NOTE added because the car did not drive straight
    steering = rescale(-data.value) +500
    if steering > 0.2*2**16:
        steering = 0.199*2**16


def listener():
    rospy.init_node('pwm_gen', anonymous=False)
    rospy.Subscriber('drive_throttle', Int16Stamped, throttle_callback)
    rospy.Subscriber('drive_steering', Int16Stamped, steering_callback)

    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=250000)

    frequency = 20  # Hz
    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        rospy.loginfo("throttle: {}; steering: {}".format(throttle, steering))

        # send to CAN bus
        packed = struct.pack('hhI', throttle, steering, 0)
        msg = can.Message(arbitration_id=0x1, data=packed, extended_id=False)
        bus.send(msg)

        rate.sleep()


if __name__ == '__main__':
    listener()
