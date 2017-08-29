/**
 @file    endeffector_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 29, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_VARIABLES_ENDEFFECTOR_MOTION_H_
#define XPP_OPT_INCLUDE_XPP_VARIABLES_ENDEFFECTOR_MOTION_H_

#include <xpp/endeffectors.h>

#include "node_values.h"
#include "phase_nodes.h"

namespace xpp {
namespace opt {

// spring_clean_ make fact that this is just a ghost component visible through
// parameter. Derive extra class called GHOST_COMPONENT with parameter Set already.
class EndeffectorMotion : public Spline, public Component {
public:
  EndeffectorMotion (EndeffectorID);
  virtual ~EndeffectorMotion ();

//  virtual const StateLinXd GetPoint(double t_global) const override;
//  virtual Jacobian GetJacobian (double t_global,  MotionDerivative dxdt) const override;

private:
  EndeffectorNodes::Ptr xy_motion_;
  EndeffectorNodes::Ptr z_motion_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_VARIABLES_ENDEFFECTOR_MOTION_H_ */
