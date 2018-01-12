/**
@file    variable_names.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jun 27, 2017
@brief   Strings used to identify the optimization variables.
 */

#ifndef TOWR_VARIABLES_VARIABLE_NAMES_H_
#define TOWR_VARIABLES_VARIABLE_NAMES_H_

#include <string>

namespace towr {
namespace id {

static const std::string base_lin_nodes    = "base_lin";
static const std::string base_ang_nodes    = "base_ang";
static const std::string ee_motion_nodes   = "ee_motion_";
static const std::string ee_force_nodes    = "ee_force_";
static const std::string contact_schedule  = "ee_schedule";


static std::string EEMotionNodes(uint ee)
{
  return  ee_motion_nodes + std::to_string(ee);
}

static std::string EEForceNodes(uint ee)
{
  return  ee_force_nodes + std::to_string(ee);
}

static std::string EESchedule(uint ee)
{
  return  contact_schedule + std::to_string(ee);
}

} // namespace id
} // namespace towr



#endif /* TOWR_VARIABLES_VARIABLE_NAMES_H_ */
