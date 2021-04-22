#include "ros/ros.h"
#include "temoto_component_manager/component_manager_interface.h"

void resourceFailureCallback(temoto_component_manager::LoadComponent load_resource_msg, temoto_resource_registrar::Status status_msgs)
{
  ROS_WARN_STREAM("The following resource stopped unexpectedly\n" << load_resource_msg.request);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_cm_client_node");
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  //ros::waitForShutdown();

  /*
   * Create External Resource Manager Interface object that provides a simplified
   * API for communicating with the External Resource Manager. The boolean "true", that's passed
   * to the constructor of ERM interface tells it whether it should be initialised immediately,
   * or that's done later by the user.
   */
  temoto_component_manager::ComponentManagerInterface cmi(true);

  /*
   * You can register a custom routine (not required) where resource failures are reported.
   */
  cmi.registerComponentStatusCallback(resourceFailureCallback);

  /*
   * ER Manager allows to invoke ROS executables, ROS launch files and other programs.
   * The "loadRosResource" and "loadSysResource" methods return a "temoto_er_manager::LoadExtResource"
   * object which contains the details regarding the query. This can be later used to unload the resource.
   */

  /*
   * Load the Gnome calculator as an example of a regular system program. Additional
   * arguments can also be passed as a second std::string variable.
   */
  ROS_INFO("Loading 2d_camera");
  auto responded_topics = cmi.startComponent("2d_camera");
  ros::Duration(15).sleep();

  ROS_INFO("Unloading 2d_camera");
  cmi.stopComponent("2d_camera", "", "");
}