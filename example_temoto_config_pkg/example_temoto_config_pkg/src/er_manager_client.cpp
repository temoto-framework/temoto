#include "ros/ros.h"
#include "temoto_er_manager/temoto_er_manager_interface.h"

void resourceFailureCallback(temoto_er_manager::LoadExtResource load_resource_msg, temoto_resource_registrar::Status status_msgs)
{
  ROS_WARN_STREAM("The following resource stopped unexpectedly\n" << load_resource_msg.request);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_er_client_node");
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  //ros::waitForShutdown();

  /*
   * Create External Resource Manager Interface object that provides a simplified
   * API for communicating with the External Resource Manager. The boolean "true", that's passed
   * to the constructor of ERM interface tells it whether it should be initialised immediately,
   * or that's done later by the user.
   */
  temoto_er_manager::ERManagerInterface ermi(true);

  /*
   * You can register a custom routine (not required) where resource failures are reported.
   */
  // ermi.registerUpdateCallback(resourceFailureCallback);

  /*
   * ER Manager allows to invoke ROS executables, ROS launch files and other programs.
   * The "loadRosResource" and "loadSysResource" methods return a "temoto_er_manager::LoadExtResource"
   * object which contains the details regarding the query. This can be later used to unload the resource.
   */

  /*
   * Load the Gnome calculator as an example of a regular system program. Additional
   * arguments can also be passed as a second std::string variable.
   */
  ROS_INFO("Loading gnome-calculator");
  temoto_er_manager::LoadExtResource load_resource_msg = ermi.loadSysResource("gnome-calculator");
  ros::Duration(15).sleep();

  ROS_INFO("Unloading gnome-calculator");
  ermi.unloadResource(load_resource_msg);

  /*
   * Load a ROS program an example of a ROS executable (regularly invoked via 'rosrun'). The first
   * parameter indicates the ROS package name and the second indicates the executable. Additional
   * arguments can also be passed as a third std::string variable. The same method can be used to
   * load ROS launch files
   */
  ROS_INFO("Loading rqt_graph");
  ermi.loadRosResource("rqt_graph", "rqt_graph");
  ros::Duration(5).sleep();
  
  /*
   * Note that this time the "unloadResource" was not invoked, as the destructor of "ermi" automatically
   * unloads all loaded resources.
   */ 
  return 0;
}