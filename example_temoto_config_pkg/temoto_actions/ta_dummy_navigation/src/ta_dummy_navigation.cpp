
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *
 *  The basis of this file has been automatically generated
 *  by the TeMoto action package generator. Modify this file
 *  as you wish but please note:
 *
 *    WE HIGHLIY RECOMMEND TO REFER TO THE TeMoto ACTION
 *    IMPLEMENTATION TUTORIAL IF YOU ARE UNFAMILIAR WITH
 *    THE PROCESS OF CREATING CUSTOM TeMoto ACTION PACKAGES
 *    
 *  because there are plenty of components that should not be
 *  modified or which do not make sence at the first glance.
 *
 *  See TeMoto documentation & tutorials at: 
 *    https://temoto-telerobotics.github.io
 *
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

/* REQUIRED BY TEMOTO */
#include <class_loader/class_loader.hpp>
#include "ta_dummy_navigation/temoto_action.h"

#include <chrono>
#include <thread>
#include <math.h>

/* 
 * ACTION IMPLEMENTATION of TaDummyNavigation 
 */
class TaDummyNavigation : public TemotoAction
{
public:

/*
 * Function that gets invoked when the action is executed (REQUIRED)
 */
void executeTemotoAction()
{
  getInputParameters();
  
  /*
   * Use the output "current location" to store the robot's intermediate location
   */
  out_param_current_location_x = in_param_current_location_x;
  out_param_current_location_y = in_param_current_location_y;

  /*
   * Calculate the travel distances
   */
  double distance_x = in_param_nav_goal_x - in_param_current_location_x;
  double distance_y = in_param_nav_goal_y - in_param_current_location_y;
  double distance_total = std::sqrt(pow(distance_x, 2) + pow(distance_y, 2));

  /*
   * Move the robot
   */
  TEMOTO_INFO("\nMoving the dummy robot to [%.2f m; %.2f m] ...", in_param_nav_goal_x, in_param_nav_goal_y);
  double time_dist = distance_total / max_speed_linear_;
  if (time_dist > min_feedback_timestep_)
  {
    unsigned int loop_iterations = time_dist / min_feedback_timestep_;
    double delta_dist = distance_total / loop_iterations;
    double angle = std::atan2(distance_y, distance_x);

    for (unsigned int i=0; i<loop_iterations; i++)
    {
      if (!actionOk())
      {
        return;
      }

      printCurrentLocation();
      out_param_current_location_x += delta_dist * std::cos(angle);
      out_param_current_location_y += delta_dist * std::sin(angle);
      std::this_thread::sleep_for(std::chrono::milliseconds(int(min_feedback_timestep_ * 1000)));
    }
  }
  out_param_current_location_x = in_param_nav_goal_x;
  out_param_current_location_y = in_param_nav_goal_y;
  printCurrentLocation();

  /*
   * Done, set the output parameters
   */
  TEMOTO_INFO("Reached the goal!");
  setOutputParameters();
}

void printCurrentLocation()
{
  TEMOTO_INFO("Current location: x=%.2f m; y=%.2f m"
  , out_param_current_location_x
  , out_param_current_location_y);
}

// Default Constructor (REQUIRED)
TaDummyNavigation()
{
  std::cout << __func__ << " constructed\n";
}

// Destructor
~TaDummyNavigation()
{
  TEMOTO_INFO("Action instance destructed");
}

// Loads in the input parameters
void getInputParameters()
{
  in_param_current_location_x = GET_PARAMETER("current_location::x", double);
  in_param_current_location_y = GET_PARAMETER("current_location::y", double);
  in_param_current_location_yaw = GET_PARAMETER("current_location::yaw", double);
  in_param_nav_goal_x = GET_PARAMETER("nav_goal::x", double);
  in_param_nav_goal_y = GET_PARAMETER("nav_goal::y", double);
  in_param_nav_goal_yaw = GET_PARAMETER("nav_goal::yaw", double);
}

// Sets the output parameters which can be passed to other actions
void setOutputParameters()
{
  SET_PARAMETER("current_location::x", "number", out_param_current_location_x);
  SET_PARAMETER("current_location::y", "number", out_param_current_location_y);
  SET_PARAMETER("current_location::yaw", "number", out_param_current_location_yaw);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class members
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Declaration of input parameters
double in_param_current_location_x;
double in_param_current_location_y;
double in_param_current_location_yaw;
double in_param_nav_goal_x;
double in_param_nav_goal_y;
double in_param_nav_goal_yaw;

// Declaration of output parameters
double out_param_current_location_x;
double out_param_current_location_y;
double out_param_current_location_yaw;

// Declaration of intrinsic parameters of the dummy robot
double max_speed_linear_ = 0.7;      // m/s
double max_speed_angular_ = 0.8;     // rad/s
double min_feedback_timestep_ = 0.2; // s


}; // TaDummyNavigation class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaDummyNavigation, ActionBase);
