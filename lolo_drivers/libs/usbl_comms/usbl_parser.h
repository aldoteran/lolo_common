/* USBL parser class for lolo's USBL acoustic modem.
 * This class can be updated to include more USBL message
 * types as needed.
 *
 * @file usbl_parser.h
 * @author aldo terán <aldot@kth.se>
 */

#ifndef USBL_COMMS_USBL_PARSER_H_
#define USBL_COMMS_USBL_PARSER_H_

#include <string>
#include <sstream>
#include <iterator>

#include <ros/console.h>
#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

namespace lolo_drivers {

class UsblParser {
public:
  struct Config {
    std::string usbl_msg_topic = ""; // Topic name for incoming USBL msg.
    std::string usbl_fix_topic = ""; // Topic name for processed USBL fix.
    // TODO: Some examples to add.
    /* std::string usbl_depth_topic;
     * std::string usbl_temp_topic;
     * std::string usbl_action_topic;
     * std::string usbl_latlon_topic;
     */
  };

  // Constructor takes config and node handle for ROS.
  UsblParser(const Config& config, ros::NodeHandle& nh);

  // The USBL message come as a std_msgs String from the captain interface.
  // TODO: we should design some sort of protocol to know what kind of message
  // is coming.
  // FIXME: I'm only gonna publish the fix as I need it for my docking
  // algorithms, so I'll be publishing a Float64MultiArray with [stamp, x, y,
  // z].
  void UsblMessageCallback(const std_msgs::String& msg);

private:
  Config config_;

  // TODO: Add publishers as you see fit.
  ros::Publisher usbl_fix_pub_;
};

} // namespace lolo_drivers

#endif // USBL_COMMS_USBL_PARSER_H_


