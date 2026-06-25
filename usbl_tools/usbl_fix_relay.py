#!/usr/bin/env python

import rospy
from sbg_driver.msg import SbgEkfQuat
from dmac.msg import mUSBLFix
from dmac.msg import DMACPayload

from scipy.spatial.transform import Rotation

MODEM_TOPSIDE_ID = 1
MODEM_LOLO_ID = 2

AT_DROP_MSG = "+++ATZ4"

class UsblRelay:

    def __init__(self):

        self.attitude = None
        self.ahrs_init = False

        # Publisher for the transmit message.
        self.transmit_pub = rospy.Publisher(rospy.get_param("/usbl_transmit_topic",
                                                            "/evologics_modem/send"),
                                            DMACPayload, queue_size=1)

       # Subscribe from the get-go.
        rospy.Subscriber(rospy.get_param("/usbl_measurement_topic",
                                        "/evologics_modem/measurement/usbl_fix"),
                         mUSBLFix, self.usbl_fix_callback)
        rospy.Subscriber(rospy.get_param("/sbg_attitude_topic",
                                         "/sbg/ekf_quat"),
                         SbgEkfQuat, self.ahrs_quat_callback)

    def usbl_fix_callback(self, msg):
        """
        Let's just assume that the USBL's AHRS is pretty solid,
        we can then compare wrt the SBG's data.
        """

        fix = [round(msg.relative_position.x, 2),
               round(msg.relative_position.y, 2),
               round(msg.relative_position.z, 2)]

        payload_msg = DMACPayload()
        payload_msg.header.stamp = rospy.Time.now()
        payload_msg.source_address = MODEM_TOPSIDE_ID
        payload_msg.destination_address = MODEM_LOLO_ID
        # TODO: I'm almost sure we don't need to append a newline for the ROS message.
        payload_msg.payload = "REL " + str(fix[0]) + " " + str(fix[1]) + " " + str(fix[2])
        self.transmit_pub.publish(payload_msg)

    def ahrs_quat_callback(self, msg):
        """
        Receives the filtered heading of the user's AHRS in ENU convention?
        """
        self.attitude = Rotation.from_quat([msg.quaternion.x, msg.quaternion.y,
                                            msg.quaternion.z, msg.quaternion.w])

    def drop_buffer(self):
        """
        Send the ATZ4 command to drop the outbound buffer.
        """
        payload_msg = DMACPayload()
        payload_msg.header.stamp = rospy.Time.now()
        payload_msg.payload = AT_DROP_MSG
        self.transmit_pub.publish(payload_msg)

        rospy.loginfo("(UsblRelay) Shutting down, sending ATZ4...")

        rospy.sleep(1)

def main():
    rospy.init_node("usbl_fix_relay")

    # Only instantiating the class should be enough.
    interface = UsblRelay()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

    # Send a drop buffer cmf when closing the node.
    # FIXME: This actually doesn't work. Since ROS has already shut down,
    # the message will not be pulbished. Need to figure out a way to do this
    # cleanly.
    # interface.drop_buffer()

    rospy.loginfo("(UsblRelay) Shutting down, send ATZ4 manually.")

if __name__ == "__main__":
    main()
