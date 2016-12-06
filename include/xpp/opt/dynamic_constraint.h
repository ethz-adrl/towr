/**
 @file    dynamic_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_SRC_DYNAMIC_CONSTRAINT_H_
#define XPP_XPP_OPT_SRC_DYNAMIC_CONSTRAINT_H_

#include "a_constraint.h"
#include "motion_structure.h"
#include "linear_inverted_pendulum.h"
#include <memory>

namespace xpp {
namespace opt {

class ComMotion;

class DynamicConstraint : public AConstraint {
public:
  using ComMotionPtrU = std::unique_ptr<ComMotion>;

  DynamicConstraint ();
  virtual ~DynamicConstraint ();

  void Init(const ComMotion&, const MotionStructure&, double com_height);

  void UpdateVariables (const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  MotionStructure motion_structure_;
  ComMotionPtrU com_motion_;
  Eigen::VectorXd cop_;
  mutable LinearInvertedPendulum model_;

  double kHeight_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_SRC_DYNAMIC_CONSTRAINT_H_ */
