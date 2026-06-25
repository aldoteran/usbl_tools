#!/usr/bin/env python

import rospy
from sbg_driver.msg import SbgEkfNav
from dmac.msg import mUSBLFix
from dmac.msg import DMACPayload

MODEM_TOPSIDE_ID = 1
MODEM_LOLO_ID = 2

AT_DROP_MSG = "+++ATZ4"

# Max allowed age of the AHRS lat/lon in [s].
MAX_LATLON_AGE = 5

USBLLONG_ID = 2

class UsblRelay:

    def __init__(self):

        self.lat_str = None
        self.lon_str = None
        self.ahrs_init = False
        self.last_ahrs = 0
        self.ahrs_age = 999

        # Publisher for the transmit message.
        self.transmit_pub = rospy.Publisher(rospy.get_param("/usbl_transmit_topic",
                                                            "/evologics_modem/send"),
                                            DMACPayload, queue_size=1)

       # Subscribe from the get-go.
        rospy.Subscriber(rospy.get_param("/usbl_measurement_topic",
                                        "/evologics_modem/measurement/usbl_fix"),
                         mUSBLFix, self.usbl_fix_callback)
        rospy.Subscriber(rospy.get_param("/sbg_navigation_topic",
                                         "/sbg/ekf_nav"),
                         SbgEkfNav, self.ahrs_nav_callback)

    def usbl_fix_callback(self, msg):
        """
        Get the raw measured range and send the correction to the vehicle.
        The outbound message has the format: "COR lat lon usbl_range measurement_stamp"
        """
        if not self.ahrs_init or (self.ahrs_age > MAX_LATLON_AGE):
            rospy.logwarn("(UsblRelay) AHRS lat/lon not available.")
            return

        if msg.type != USBLLONG_ID:
            rospy.logwarn("(UsblRelay) Not a USBLLONG, skipping...")
            return

        stamp = str(int(msg.header.stamp.to_sec()))
        raw_range = str(round(msg.range,2))

        payload_msg = DMACPayload()
        payload_msg.header.stamp = rospy.Time.now()
        payload_msg.source_address = MODEM_TOPSIDE_ID
        payload_msg.destination_address = MODEM_LOLO_ID

        # TODO: I'm almost sure we don't need to append a newline for the ROS message.
        payload_msg.payload = "COR " + self.lat_str + " " + self.lon_str + " " + raw_range + " " + stamp

        self.transmit_pub.publish(payload_msg)

    def ahrs_nav_callback(self, msg):
        """
        Get lat/lon from the AHRS.
        """
        now = rospy.Time.now().to_sec()

        if not self.ahrs_init:
            self.last_ahrs = now
            self.ahrs_init = True

        self.lat_str = str(round(msg.latitude, 6))
        self.lon_str = str(round(msg.longitude, 6))

        self.ahrs_age = now - self.last_ahrs
        self.last_ahrs = now

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
    rospy.init_node("usbl_correction_relay")

    # Only instantiating the class should be enough.
    interface = UsblRelay()

    rate = rospy.Rate(1)
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
