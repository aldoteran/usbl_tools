/**
 * @file usbl_relay.cpp
 * @brief ROSnode relaying USBL fix from topside unit to LoLo.
 * @date Oct 18, 2022
 * @author aldo terán (aldot@kth.se)
 */

#include "usbl_relay.h"

namespace dmac {

UsblRelay::UsblRelay(ros::NodeHandle& nh){
  // Initialize publisher.
  dmac_to_lolo_pub_ =
      nh.advertise<DMACPayload>("/evologics_modem/send", 1);
}

void UsblRelay::UsblFixCallback(const mUSBLFix &msg) {
  // Some printouts for debugging.
  if (msg.type == 0) {
    ROS_WARN("Got RANGE_ONLY measurement.");
  } else if (msg.type == 1) {
    ROS_WARN("Got AZIMUTH_ONLY measurement.");
  } else if (msg.type == 2) {
    ROS_WARN("Got FULL_FIX measurement.");
  } else {
    ROS_WARN("Got CARTESIAN measurement.");
  }

  // Get relative fix in USBL's inertial frame (i.e NED frame).
  // Gotta rotate the fix into the ENU frame. I'll do it by simply mapping
  // the (x, y, z) coordinate to (x, -y, -z). TODO: (sanity check).
  std::string x_coord =
      std::to_string(std::ceil(msg.relative_position.x * 100.0) / 100.0);
  x_coord.erase(x_coord.find_last_not_of('0') + 1, std::string::npos);
  x_coord.erase(x_coord.find_last_not_of('.') + 1, std::string::npos);
  std::string y_coord =
      std::to_string(-std::ceil(msg.relative_position.y * 100.0) / 100.0);
  y_coord.erase(y_coord.find_last_not_of('0') + 1, std::string::npos);
  y_coord.erase(y_coord.find_last_not_of('.') + 1, std::string::npos);
  std::string z_coord =
      std::to_string(-std::ceil(msg.relative_position.z * 100.0) / 100.0);
  z_coord.erase(z_coord.find_last_not_of('0') + 1, std::string::npos);
  z_coord.erase(z_coord.find_last_not_of('.') + 1, std::string::npos);

  // Populate DMACPayload message with USBL fix and scientist command.
  DMACPayload payload_msg;
  payload_msg.header = msg.header;
  payload_msg.header.stamp = ros::Time::now();
  payload_msg.payload =
      "CAP SENDTOSCIENTIST " + x_coord + "," + y_coord + "," + z_coord;

  ROS_INFO("(UsblRelay) sending to LoLo: %s", payload_msg.payload.c_str());

  // Publish to payload topic.
  dmac_to_lolo_pub_.publish(payload_msg);
}

} // namespace dmac
