/**
@file    variable_names.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jun 27, 2017
@brief   Strings used to identify the optimization variables.
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_VARIABLE_NAMES_H_
#define XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_VARIABLE_NAMES_H_

#include <string>

namespace xpp {
namespace id {

static const std::string base_linear         = "base_lin_";
static const std::string base_angular        = "base_ang_";
static const std::string endeffectors_motion = "ee_motion_";
static const std::string contact_schedule    = "ee_schedule";
static const std::string endeffector_force   = "ee_force_";


static std::string GetEEMotionId(int ee)
{
  return  endeffectors_motion + "_xy_" + std::to_string(ee);
}

static std::string GetEEForceId(int ee)
{
  return  endeffector_force + std::to_string(ee);
}

static std::string GetEEScheduleId(int ee)
{
  return  contact_schedule + std::to_string(ee);
}

} // namespace id
} // namespace xpp



#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_VARIABLE_NAMES_H_ */
