/**
 @file    linear_inverted_pendulum.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_INVERTED_PENDULUM_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_INVERTED_PENDULUM_H_

#include <Eigen/Dense>

namespace xpp {
namespace opt {

class LinearInvertedPendulum {
public:
  using ComPos = Eigen::Vector2d;
  using ComVel = Eigen::Vector2d;
  using ComAcc = Eigen::Vector2d;
  using Cop    = Eigen::Vector2d;

  LinearInvertedPendulum ();
  virtual ~LinearInvertedPendulum ();

  void SetCurrent(const ComPos&, const ComVel&, double height);
  ComAcc GetDerivative(const Cop& p) const;

private:
  ComPos pos_;
  ComVel vel_;
  double h_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_INVERTED_PENDULUM_H_ */
