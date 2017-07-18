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
namespace opt {
namespace id {

static const std::string base_linear         = "base_lin_";
static const std::string base_angular        = "base_ang_";
static const std::string endeffectors_motion = "endeffectors_motion_";
static const std::string contact_schedule    = "contact_schedule_";
static const std::string endeffector_force   = "endeffector_force_";

} // namespace id
} // namespace opt
} // namespace xpp



#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_VARIABLE_NAMES_H_ */
