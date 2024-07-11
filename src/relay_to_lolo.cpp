/**
 * @file relay_to_lolo.cpp
 * @brief ROSnode relaying USBL fix from topside unit to LoLo.
 * @date Oct 18, 2022
 * @author aldo terán (aldot@kth.se)
 */

#include "usbl_relay.h"

//using namespace dockslam;

int main(int argc, char **argv) {
  ros::init(argc, argv, "relay_USBL_to_lolo");
  ros::NodeHandle nh;

  dmac::UsblRelay usbl_relay(nh);

  ros::Subscriber usbl_sub =
      nh.subscribe("/evologics_modem/measurement/usbl_fix", /*queue_size=*/1,
                   &dmac::UsblRelay::UsblFixCallback, &usbl_relay);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
