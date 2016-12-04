/**
 @file    dynamic_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Declares the DynamicConstraint class
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_CONSTRAINT_H_

#include "a_constraint.h"
#include "motion_structure.h"
#include <memory>

namespace xpp {

namespace hyq { class SupportPolygonContainer; }

namespace opt {

class ComMotion;

/** Ensures physical feasibility of the motion
  */
class DynamicConstraint : public AConstraint {
public:
  using Contacts      = xpp::hyq::SupportPolygonContainer;
  using ComMotionPtrU = std::unique_ptr<ComMotion>;
  using ContactPtrU   = std::unique_ptr<Contacts>;

  DynamicConstraint ();
  virtual ~DynamicConstraint ();

  void Init(const ComMotion&, const Contacts&, const MotionStructure&);

  void UpdateVariables (const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  ComMotionPtrU com_motion_;
  ContactPtrU contacts_;
  MotionStructure motion_structure_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_CONSTRAINT_H_ */
