/**
 @file    dynamic_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_SRC_DYNAMIC_CONSTRAINT_H_
#define XPP_XPP_OPT_SRC_DYNAMIC_CONSTRAINT_H_

#include "linear_inverted_pendulum.h"
#include <xpp/a_constraint.h>
#include <memory>

namespace xpp {
namespace opt {

class ComMotion;

class DynamicConstraint : public AConstraint {
public:
  using ComMotionPtrU = std::unique_ptr<ComMotion>;

  DynamicConstraint ();
  virtual ~DynamicConstraint ();

  void Init(const ComMotion&, double dt);

  void UpdateVariables (const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  ComMotionPtrU com_motion_;
  Eigen::VectorXd cop_;
  mutable LinearInvertedPendulum model_;

//  double dt_;
  std::vector<double> dts_;

  Jacobian GetJacobianWrtCop() const;
  Jacobian GetJacobianWrtCom() const;
  double kHeight_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_SRC_DYNAMIC_CONSTRAINT_H_ */
