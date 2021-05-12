
#ifndef ta_robot_manager_tests_TEMOTO_ACTION_H
#define ta_robot_manager_tests_TEMOTO_ACTION_H

/* REQUIRED BY TEMOTO */
#include "temoto_core/common/base_subsystem.h"
#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/messaging.h"

#define GET_PARAMETER(name, type) getUmrfNodeConst().getInputParameters().getParameterData<type>(name)
#define SET_PARAMETER(name, type, value) getUmrfNode().getOutputParametersNc().setParameter(name, type, boost::any(value))

/**
 * @brief Class that integrates TeMoto Base Subsystem specific and Action Engine specific codebases.
 * 
 */
class TemotoAction : public ActionBase, public temoto_core::BaseSubsystem
{
public:
  TemotoAction()
  : BaseSubsystem("action_engine", temoto_core::error::Subsystem::TASK, "DEFINED_LATER", "actions")
  {}

  /**
   * @brief Get the Name of the action
   * 
   * @return const std::string& 
   */
  const std::string& getName()
  {
    return getUmrfNodeConst().getFullName();
  }

  /**
   * @brief Wraps the executeTemotoAction and converts TeMoto specific errors to action engine errors.
   * 
   */
  void executeAction()
  {
    try
    {
      /* 
       * Assign the class name. This cannot be done in constructor because class loader works
       * only with default constructors.
       */ 
      class_name_ = getUmrfNodeConst().getFullName();
      executeTemotoAction();
    }
    catch(temoto_core::error::ErrorStack e)
    {
      /*
       * TODO: Do something with TeMoto errors.
       */
      std::cout << e << std::endl;
    }
    catch(TemotoErrorStack e)
    {
      throw FORWARD_TEMOTO_ERROR_STACK(e);
    }
    catch(const std::exception& e)
    {
      throw CREATE_TEMOTO_ERROR_STACK(e.what());
    }
    catch(...)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Caught an unhandled exception");
    }
  }

  virtual void updateParameters(const ActionParameters& parameters_in)
  {
    for (const auto& p_in : parameters_in)
    {
      boost::any param_data;

      if (!getUmrfNodeConst().getInputParameters().hasParameter(p_in))
      {
        throw CREATE_TEMOTO_ERROR_STACK("This action has no parameter '" + p_in.getName() + "'");
      }

      else
      {
        throw CREATE_TEMOTO_ERROR_STACK("No matching data type");
      }

      getUmrfNode().getInputParametersNc().setParameterData(p_in.getName(), param_data);
    }
  }

  /**
   * @brief Has to be implemented by an action.
   * 
   */
  virtual void executeTemotoAction() = 0;
};

#endif
