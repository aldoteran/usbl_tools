/**
 * @file visualizer_dmac.cpp
 * @brief ROS node that converts mUSBLFix messages to ROS visualization msgs.
 * @date Oct 21, 2022
 * @author aldo terán (aldot@kth.se)
 */

#include <dmac_translator/dmac_translator.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "dmac_to_marker");
  ros::NodeHandle nh;
  dmac::DmacTranslator::Config config;

  // Get ROS parameters for configuration.
  nh.param<std::string>("dmac_fix_topic", config.dmac_fix_topic,
                        "/evologics_modem/measurement/usbl_fix");
  nh.param<std::string>("marker_fix_topic", config.marker_fix_topic,
                        "/tender/usbl_fix");
  nh.param<std::string>("marker_map_fix_topic", config.marker_map_fix_topic,
                        "/tender/enu/usbl_fix");
  // FIXME: why is this not workinnnnnnggg?!
  //nh.param<std::string>("map_frame_id", config.map_frame_id, "/map");
  config.map_frame_id = "tender/odom";
  nh.param<std::string>("usbl_frame_id", config.usbl_frame_id,
                        "/tender/usbl_link");

  nh.param<bool>("publish_in_map_frame", config.publish_in_map_frame, true);

  dmac::DmacTranslator dmac_translator(config, nh);

  ros::Subscriber usbl_sub =
      nh.subscribe(config.dmac_fix_topic, /*queue_size=*/1,
                   &dmac::DmacTranslator::UsblFixCallback, &dmac_translator);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
