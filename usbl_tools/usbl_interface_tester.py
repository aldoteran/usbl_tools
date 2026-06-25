#!/usr/bin/env python

import rospy
from std_msgs.msg import Char
from std_msgs.msg import Float64

import time

def main():
    rospy.init_node("usbl_interface_tester")

    usbl_pub = rospy.Publisher("lolo/core/usbl/received", Char, queue_size=100)
    depth_pub = rospy.Publisher("lolo/dr/depth", Float64, queue_size=1)

    msg = "RUNCMD ~/./usbl_stop.sh"
    stamp = int(time.time())
    msg = msg + " " + str(stamp)

    bytelist = [ord(c) for c in msg]
    checksum = hex(sum(bytearray(bytelist)) % 256)[2:]

    msg = msg + " " + checksum + "\n"

    # msg = "REL 1 2 3 4\n"

    while not rospy.is_shutdown():
        depth_pub.publish(Float64(15.0))
        rospy.loginfo("Relaying message: {0}.".format(msg))
        for c in msg:
            usbl_pub.publish(Char(ord(c)))
        rospy.sleep(3)

if __name__ == "__main__":
    main()
