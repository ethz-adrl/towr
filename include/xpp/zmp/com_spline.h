/**
 @file    com_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_SPLINE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_SPLINE_H_

#include "com_motion.h"

namespace xpp {
namespace zmp {

/** Represents the Center of Mass (CoM) motion as a Spline (sequence of polynomials).
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomes into a CoM position/velocity and acceleration.
  */
class ComSpline : public ComMotion {
public:
  ComSpline ();
  virtual ~ComSpline ();
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_SPLINE_H_ */
