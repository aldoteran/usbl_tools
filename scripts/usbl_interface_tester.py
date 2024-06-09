#!/usr/bin/env python

import rospy
from std_msgs.msg import Char
from std_msgs.msg import Float64

def main():
    rospy.init_node("usbl_interface_tester")

    usbl_pub = rospy.Publisher("lolo/core/usbl/received", Char, queue_size=100)
    depth_pub = rospy.Publisher("lolo/dr/depth", Float64, queue_size=1)

    msg = "RUNCMD 723486132 1 2 3 4 5\n"

    while not rospy.is_shutdown():
        depth_pub.publish(Float64(15.0))
        for c in msg:
            usbl_pub.publish(Char(ord(c)))
        rospy.loginfo("Relaying message.")
        rospy.sleep(3)

if __name__ == "__main__":
    main()
