
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

/* 
 * ACTION IMPLEMENTATION of TaDummyNavigation 
 */
class TaDummyNavigation : public TemotoAction
{
public:

// Constructor. REQUIRED BY TEMOTO
TaDummyNavigation()
{
  std::cout << __func__ << " constructed\n";
}

// REQUIRED BY TEMOTO
void executeTemotoAction()
{
  // Input parameters
  double in_param_position_x = GET_PARAMETER("position::x", double);
  double in_param_position_y = GET_PARAMETER("position::y", double);
  double in_param_position_z = GET_PARAMETER("position::z", double);

  // Declaration of output parameters
  double out_param_position_x;
  double out_param_position_y;
  double out_param_position_z;

  /* * * * * * * * * * * * * * 
   *                          
   * ===> YOUR CODE HERE <===
   *                          
   * * * * * * * * * * * * * */

  // Pass the output parameters to the action engine
  SET_PARAMETER("position::x", "number", out_param_position_x);
  SET_PARAMETER("position::y", "number", out_param_position_y);
  SET_PARAMETER("position::z", "number", out_param_position_z);

}

// Destructor
~TaDummyNavigation()
{
  TEMOTO_INFO("Action instance destructed");
}

}; // TaDummyNavigation class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaDummyNavigation, ActionBase);
