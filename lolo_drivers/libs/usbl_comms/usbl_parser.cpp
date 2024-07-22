
#include "usbl_parser.h"

namespace lolo_drivers {
namespace {
bool IsConfigValid(const UsblParser::Config &config) {
  if (config.usbl_fix_topic.empty()) {
    ROS_ERROR("(UsblParser) Empty 'usbl_fix_topic'");
    return false;
  }
  return true;
}
} // namespace

UsblParser::UsblParser(const UsblParser::Config &config, ros::NodeHandle &nh)
    : config_(config) {
  if (!IsConfigValid(config_)) {
    ROS_ERROR("(UsblParser) Incorrect initial configuration.");
    return;
  }
  usbl_fix_pub_ =
      nh.advertise<std_msgs::Float64MultiArray>(config_.usbl_fix_topic, 1);
}

void UsblParser::UsblMessageCallback(const std_msgs::String& msg){
  // TODO: We should have some sort of switch-case scenario depending on the
  // first entry in the String msg.
  std::string message = msg.data;

  std_msgs::Float64MultiArray fix_out;
  fix_out.data.push_back(ros::Time::now().toSec());

  // Slice message where there's a whitespace.
  std::istringstream iss(message);
  std::vector<std::string> entries {
    std::istream_iterator<std::string>{iss},
        std::istream_iterator<std::string> {}
  };

  // Make sure we only got one fix with 3 values (X,Y,Z).
  if (entries.size() > 3) {
      ROS_WARN("(UsblParser) Got a corrupted message, skipping...");
      return;
  }
  // Make sure we don't have only zeros.
  if (std::all_of(entries.begin(), entries.end(), [](std::string s) { return s=="0"; })){
      return;
  }

  // Loop through entries and make sure we only have digits.
  for (std::size_t i = 0; i < entries.size(); i++) {
    fix_out.data.push_back(std::stod(entries[i]));
  }

  usbl_fix_pub_.publish(fix_out);
}

} // namespace lolo_drivers
