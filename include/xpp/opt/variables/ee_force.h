/**
 @file    ee_force.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 15, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_EE_FORCE_H_
#define XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_EE_FORCE_H_

#include "xpp/opt/constraints/composite.h"

namespace xpp {
namespace opt {

/** @brief Parameterizes the continuous force over time of a single endeffector.
 */
class EEForce : public Component {
public:
  EEForce ();
  virtual ~EEForce ();
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_EE_FORCE_H_ */
