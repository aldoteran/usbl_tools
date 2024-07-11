/**
 * @file dmac_translator.h
 * @brief DMACTranslator class for converting DMAC-specific ROS messages into
 * visualization messages.
 * @date Oct 20, 2022
 * @author aldo terán (aldot@kth.se)
 */

#ifndef DMAC_TRANSLATOR_DMAC_TRANSLATOR_H_
#define DMAC_TRANSLATOR_DMAC_TRANSLATOR_H_

#include <dmac/mUSBLFix.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>

namespace dmac {

class DmacTranslator {
public:
  struct Config {
    std::string dmac_fix_topic = "";    // Incoming USBL topic (DMAC).
    std::string marker_fix_topic = "";  // Outgoing USBL topic (Marker).
    std::string marker_map_fix_topic = "";  // Outgoing USBL topic (Marker).

    std::string map_frame_id = "";      // Map frame name.
    std::string usbl_frame_id = "";     // USBL topside frame name.

    // Whether to publish usbl fix in the map frame as well.
    bool publish_in_map_frame = true;
  };

  // Constructor takes config struct and node handler for ROS.
  DmacTranslator(const Config &config_, ros::NodeHandle &nh);

  // Callback for the USBL measurement in DMAC standard. Converts it
  // into a Marker ROS msg and republishes it.
  void UsblFixCallback(const mUSBLFix& msg);

private:
  Config config_;

  // TF listener.
  tf::TransformListener listener_;

  // ROS publisher for Marker msg in the base frame.
  ros::Publisher marker_fix_pub_;

  // ROS publisher for Marker msg in the map frame.
  ros::Publisher marker_map_fix_pub_;

  // Counter for Marker id.
  int id_ = 0;
};

} // namespace dmac

#endif //DMAC_TRANSLATOR_DMAC_TRANSLATOR_H_
