
#include <usbl_comms/usbl_parser.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "lolo_usbl_parser");
  ros::NodeHandle nh;
  lolo_drivers::UsblParser::Config config;

  nh.param<std::string>("usbl_message_topic", config.usbl_msg_topic,
                        "/lolo/text");
  nh.param<std::string>("usbl_fix_topic", config.usbl_fix_topic,
                        "/lolo/sim/usbl");

  lolo_drivers::UsblParser usbl_parser(config, nh);

  ros::Subscriber usbl_message_sub = nh.subscribe(
      config.usbl_msg_topic, /*queue_size=*/1,
      &lolo_drivers::UsblParser::UsblMessageCallback, &usbl_parser);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
