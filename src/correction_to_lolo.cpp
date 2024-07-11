/**
 * @file correction_to_lolo.cpp
 * @brief ROSnode correction relay to lolo.
 * @date Oct 18, 2022
 * @author aldo terán (aldot@kth.se)
 */

#include "usbl_correction.h"

//using namespace dockslam;

int main(int argc, char **argv) {
  ros::init(argc, argv, "usbl_correction_to_lolo");
  ros::NodeHandle nh;

  dmac::Usblcorrection usbl_correction(nh);

  ros::Subscriber usbl_sub =
      nh.subscribe("/evologics_modem/measurement/usbl_fix", /*queue_size=*/5,
                   &dmac::Usblcorrection::UsblFixCallback, &usbl_correction);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
