#include "ros/ros.h"
#include "std_msgs/String.h"

std::string string_0, string_1;
ros::Publisher pub_0;
ros::Publisher pub_1;

// Subscriber 0 callback
void sub0Cb(const std_msgs::String &msg)
{
  // std::cout << "Changing string_0 to " << msg.data << std::endl;
  string_0 = msg.data;

  // Create the messages
  std_msgs::String msg_0;
  msg_0.data = string_0 + " like it is supposed to.";

  // Publish the messages
  pub_0.publish(msg_0);
}

// Subscriber 1 callback
void sub1Cb(const std_msgs::String &msg)
{
  // std::cout << "Changing string_1 to " << msg.data << std::endl;
  string_1 = msg.data;

  // Create the messages
  std_msgs::String msg_1;
  msg_1.data = string_1 + " properly.";

  // Publish the messages
  pub_1.publish(msg_1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "component_string_2_node");
  ros::NodeHandle nh;

  // Test publishers
  pub_0 = nh.advertise<std_msgs::String>("cs_2_pub_0", 10);
  pub_1 = nh.advertise<std_msgs::String>("cs_2_pub_1", 10);

  // Test subscribers
  ros::Subscriber sub_0 = nh.subscribe( "cs_2_sub_0", 10, sub0Cb);
  ros::Subscriber sub_1 = nh.subscribe( "cs_2_sub_1", 10, sub1Cb);

  std::cout << "component_string_2 READY\n";

  ros::spin();

  return 0;
}
