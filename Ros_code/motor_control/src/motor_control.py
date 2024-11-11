#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32

def motor_control():
    pub = rospy.Publisher('motor_speed', Int32, queue_size=10)
    rospy.init_node('motor_controller', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            speed = int(input("Nhập tốc độ động cơ (-255 đến 255): "))
            if -255 <= speed <= 255:
                rospy.loginfo("Setting motor speed to: {}".format(speed))
                pub.publish(speed)
            else:
                rospy.logwarn("Vui lòng nhập giá trị trong khoảng -255 đến 255.")
        except ValueError:
            rospy.logwarn("Giá trị nhập vào không hợp lệ. Vui lòng nhập một số nguyên.")
        rate.sleep()

if __name__ == '__main__':
    try:
        motor_control()
    except rospy.ROSInterruptException:
        pass
