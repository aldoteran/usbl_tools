#include "dmac_translator.h"

#include <geometry_msgs/Quaternion.h>


namespace dmac {
namespace {

bool IsConfigValid(const DmacTranslator::Config &config) {
  // Check if config has been done successfully.
  if (config.dmac_fix_topic.empty()) {
    ROS_ERROR("(DmacTranslator) Empty 'dmac_fix_topic'.");
    return false;
  }
  if (config.marker_fix_topic.empty()) {
    ROS_ERROR("(DmacTranslator) Empty 'marker_fix_topic'.");
    return false;
  }

  if (config.map_frame_id.empty()) {
    ROS_ERROR("(DmacTranslator) Empty 'map_frame_id'.");
    return false;
  }
  if (config.usbl_frame_id.empty()) {
    ROS_ERROR("(DmacTranslator) Empty 'usbl_frame_id'.");
    return false;
  }

  return true;
}
} // namespace

DmacTranslator::DmacTranslator(const Config &config, ros::NodeHandle &nh)
    : config_(config) {
  if (!IsConfigValid(config_)) {
    ROS_ERROR("(DmacTranslator) Incorrect initial configuration");
    return;
  }
  // Init publishers.
  marker_fix_pub_ =
      nh.advertise<visualization_msgs::Marker>(config_.marker_fix_topic, 1);
  marker_map_fix_pub_ =
      nh.advertise<visualization_msgs::Marker>(config_.marker_map_fix_topic, 1);
}

void DmacTranslator::UsblFixCallback(const mUSBLFix &msg) {
  if (msg.type != 2) {
    ROS_WARN("(DmacTranslator) Didn't get FULL_FIX. Skipping...");
    return;
  }

  // Populate Marker msg.
  visualization_msgs::Marker usbl_marker;
  usbl_marker.header = msg.header;
  usbl_marker.header.frame_id = config_.usbl_frame_id;
  usbl_marker.id = id_;
  usbl_marker.type = visualization_msgs::Marker::SPHERE;
  usbl_marker.action = visualization_msgs::Marker::ADD;

  // Position from USBL message.
  usbl_marker.pose.position.x = msg.relative_position.x;
  usbl_marker.pose.position.y = msg.relative_position.y;
  usbl_marker.pose.position.z = msg.relative_position.z;
  // Orientation we don't care.
  usbl_marker.pose.orientation = geometry_msgs::Quaternion();

  // Color for the marker (green and no transparency).
  usbl_marker.color.g = 1.0;
  usbl_marker.color.a = 1.0;

  // How does scale work?
  usbl_marker.scale.x = 1.1;
  usbl_marker.scale.y = 1.1;
  usbl_marker.scale.z = 1.1;

  id_++;
  marker_fix_pub_.publish(usbl_marker);

  if (config_.publish_in_map_frame) {
    // Get TF from map to tender.
    tf::StampedTransform map_tfm_tender;
    try {
      listener_.waitForTransform(config_.map_frame_id, config_.usbl_frame_id,
                               ros::Time(0), ros::Duration(0.1));
      listener_.lookupTransform(config_.map_frame_id, config_.usbl_frame_id,
                               ros::Time(0), map_tfm_tender);
    } catch (tf::TransformException ex) {
      ROS_WARN("(DmacTranslator) %s", ex.what());
      return;
    }
    usbl_marker.pose.position.x += map_tfm_tender.getOrigin().x();
    usbl_marker.pose.position.y += map_tfm_tender.getOrigin().y();
    usbl_marker.header.frame_id = config_.map_frame_id;
    marker_map_fix_pub_.publish(usbl_marker);
  }
}

} // namespace dmac
