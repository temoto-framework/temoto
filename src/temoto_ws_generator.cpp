#include "ros/ros.h"
#include "ros/package.h"
#include "file_template_parser/file_template_parser.h"
#include <boost/filesystem.hpp>

int main(int argc, char **argv)
{
  std::cout << "d1\n";
  // Check if arguments were provided
  if (argc != 2)
  {
    std::cout << "d2\n";
    std::cout << "Invalid number of arguments" << std::endl;
    return 1;
  }

  std::cout << "d3\n";
  // Get the name of the package
  const std::string temoto_ws_name = std::string(argv[1]);
  const std::string base_path = ros::package::getPath(ROS_PACKAGE_NAME);
  const std::string temoto_ws_path = base_path + "/../" + temoto_ws_name + "/";

  // Import the CMakeLists template
  tp::TemplateContainer t_cmakelists = tp::TemplateContainer(base_path + "/templates/temoto_ws_cmakelists.xml");

  // Import the package.xml template
  tp::TemplateContainer t_packagexml = tp::TemplateContainer(base_path + "/templates/temoto_ws_packagexml.xml");

  // Import the action path config template
  tp::TemplateContainer t_action_config = tp::TemplateContainer(base_path + "/templates/temoto_ws_action_config.xml");

  /*
   * CREATE TEMOTO WS PACKAGE DIRECTORY STRUCTURE
   */ 
  boost::filesystem::create_directories(temoto_ws_path + "actions");
  boost::filesystem::create_directories(temoto_ws_path + "config");

  /*
   * GENERATE THE CONTENT
   */
  t_cmakelists.setArgument("temoto_ws_name", temoto_ws_name);
  t_packagexml.setArgument("temoto_ws_name", temoto_ws_name);
  t_action_config.setArgument("temoto_ws_name", temoto_ws_name);

  /*
   * SAVE THE CONTENT
   */
  t_cmakelists.processAndSaveTemplate(temoto_ws_path, "CMakeLists");
  t_packagexml.processAndSaveTemplate(temoto_ws_path, "package");
  t_action_config.processAndSaveTemplate(temoto_ws_path + "/config/", "action_dst");

  return 0;
}
