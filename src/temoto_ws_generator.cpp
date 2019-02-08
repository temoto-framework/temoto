#include "ros/package.h"
#include "file_template_parser/file_template_parser.h"
#include <boost/filesystem.hpp>
#include <iostream>

int main(int argc, char **argv)
{
  // Check if arguments were provided
  if (argc != 2)
  {
    std::cout << "Invalid number of arguments" << std::endl;
    return 1;
  }

  // Get the name of the package
  const std::string temoto_ws_name = std::string(argv[1]);
  const std::string base_path = ros::package::getPath(ROS_PACKAGE_NAME);
  const std::string temoto_ws_path = base_path + "/../../" + temoto_ws_name + "/";
  const std::string temoto_ws_package_path = temoto_ws_path + temoto_ws_name + "/";

  /*
   * IMPORT THE TEMPLATES
   */ 
  tp::TemplateContainer t_cmakelists = tp::TemplateContainer(base_path + "/templates/temoto_ws_cmakelists.xml");
  tp::TemplateContainer t_packagexml = tp::TemplateContainer(base_path + "/templates/temoto_ws_packagexml.xml");
  tp::TemplateContainer t_action_config = tp::TemplateContainer(base_path + "/templates/temoto_ws_action_config.xml");
  tp::TemplateContainer t_nlp_launch = tp::TemplateContainer(base_path + "/templates/temoto_ws_nlp_launch.xml");
  tp::TemplateContainer t_components = tp::TemplateContainer(base_path + "/templates/temoto_ws_components.xml");

  /*
   * CREATE TEMOTO WS PACKAGE DIRECTORY STRUCTURE
   */
  std::cout << "* Creating package folder structure" << std::endl; 
  boost::filesystem::create_directories(temoto_ws_path + "actions");
  boost::filesystem::create_directories(temoto_ws_package_path + "config");
  boost::filesystem::create_directories(temoto_ws_package_path + "launch");

  /*
   * GENERATE THE CONTENT
   */
  std::cout << "* Parsing arguments" << std::endl;
  t_cmakelists.setArgument("temoto_ws_name", temoto_ws_name);
  t_packagexml.setArgument("temoto_ws_name", temoto_ws_name);
  t_action_config.setArgument("temoto_ws_name", temoto_ws_name);
  t_nlp_launch.setArgument("temoto_ws_name", temoto_ws_name);
  t_components.setArgument("temoto_ws_name", temoto_ws_name);

  /*
   * SAVE THE CONTENT
   */
  std::cout << "* Generating the package content" << std::endl;
  t_cmakelists.processAndSaveTemplate(temoto_ws_package_path, "CMakeLists");
  t_packagexml.processAndSaveTemplate(temoto_ws_package_path, "package");
  t_action_config.processAndSaveTemplate(temoto_ws_package_path + "/config/", "action_dst");
  t_nlp_launch.processAndSaveTemplate(temoto_ws_package_path + "/launch/", "temoto_nlp");
  t_components.processAndSaveTemplate(temoto_ws_package_path, "components");

  std::cout << "* Finished generating a TeMoto workspace '" << temoto_ws_name 
            << "' to " << temoto_ws_path << std::endl;

  return 0;
}
