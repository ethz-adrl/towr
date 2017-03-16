/**
 @file    center_of_pressure.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 16, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CENTER_OF_PRESSURE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CENTER_OF_PRESSURE_H_

#include <Eigen/Dense>

namespace xpp {
namespace opt {

/** Parametrizes the motion of the center of pressure over the trajectory.
  *
  * In this case we discretize and represent as piece-wise constant.
  */
class CenterOfPressure {
public:
  using VectorXd = Eigen::VectorXd;

  CenterOfPressure ();
  virtual ~CenterOfPressure ();

  void Init(double dt, double T);

  void SetOptimizationVariables(const VectorXd& x);
  VectorXd GetOptimizationVariables() const;


private:
  VectorXd cop_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CENTER_OF_PRESSURE_H_ */
