#include "ros/ros.h"
#include "temoto_component_manager/component_manager_interface.h"

void resourceFailureCallback(temoto_component_manager::LoadComponent load_resource_msg, temoto_resource_registrar::Status status_msgs)
{
  ROS_WARN_STREAM("The following resource stopped unexpectedly\n" << load_resource_msg.request);
}

int main(int argc, char** argv)
{
  TEMOTO_LOG_ATTR.initialize("test_cm_client_node");
  ros::init(argc, argv, TEMOTO_LOG_ATTR.getSubsystemName());
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  //ros::waitForShutdown();

  /*
   * Create Component Manager Interface object that provides a simplified
   * API for communicating with the Component Manager. The boolean "true", that's passed
   * to the constructor of ERM interface tells it whether it should be initialised immediately,
   * or that's done later by the user.
   */
  temoto_component_manager::ComponentManagerInterface cmi(true);

  /*
   * You can register a custom routine (not required) where resource failures are reported.
   */
  cmi.registerComponentStatusCallback(resourceFailureCallback);

  ROS_INFO("Loading 2d_camera");
  auto responded_topics = cmi.startComponent("2d_camera");
  ros::Duration(10).sleep();

  ROS_INFO("Unloading 2d_camera");
  cmi.stopComponent("2d_camera", "", "");

  // ROS_INFO("Loading testpipe");
  // //temoto_core::TopicContainer load_pipe_msg = cmi.startPipe("testpipe");
  // ros::Duration(15).sleep();
}