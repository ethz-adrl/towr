/**
@file    variable_names.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Aug 16, 2016
@brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_VARIABLE_NAMES_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_VARIABLE_NAMES_H_

#include <xpp/utils/cartesian_declarations.h>

namespace xpp {
namespace opt {

/** Common strings to use to name optimization variable sets. This is just to
  * avoid misspelling resulting in a set not being found.
  */
struct VariableNames
{
  static constexpr const char* kSplineCoeff        = "spline_coeff";
  static constexpr const char* kFootholds          = "footholds";
  static constexpr const char* kConvexity          = "convexity_lambdas";
  static constexpr const char* kCenterOfPressure   = "center_of_pressure";
};

struct ContactVars {
  static int Index (int id, xpp::utils::Coords3D dim)
  {
    assert(id >= 0); // footholds fixed by start can't be optimized over, so have no index
    return id*xpp::utils::kDim2d + dim;
  }
};


} /* namespace opt */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_VARIABLE_NAMES_H_ */
