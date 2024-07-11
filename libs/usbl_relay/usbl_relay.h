
#ifndef DMAC_USBL_RELAY_H_
#define DMAC_USBL_RELAY_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <dmac/DMACPayload.h>
#include <dmac/mUSBLFix.h>

namespace dmac {

class UsblRelay {
public:
  // Constructor requires the node handle to initialize internal publishers.
  UsblRelay(ros::NodeHandle &nh);

  //
  void UsblFixCallback(const mUSBLFix &msg);

private:
  // Publisher for the DMAC transmission topic.
  ros::Publisher dmac_to_lolo_pub_;
};

} // namespace dmac

#endif // DMAC_USBL_RELAY_H_
