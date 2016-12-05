/**
 @file    dynamic_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Declares the DynamicConstraint class
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_

#include "a_constraint.h"
#include "motion_structure.h"
#include <memory>

namespace xpp {

namespace hyq { class SupportPolygonContainer; }

namespace opt {

class ComMotion;

/** Ensures constraints related to the area created by the contact points
  */
class SupportAreaConstraint : public AConstraint {
public:
  using Contacts      = xpp::hyq::SupportPolygonContainer;
//  using ComMotionPtrU = std::unique_ptr<ComMotion>;
  using ContactPtrU   = std::unique_ptr<Contacts>;

  SupportAreaConstraint ();
  virtual ~SupportAreaConstraint ();

  void Init(const Contacts&, const MotionStructure&);

  void UpdateVariables (const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
//  ComMotionPtrU com_motion_;
  ContactPtrU contacts_;
  MotionStructure motion_structure_;
  Eigen::VectorXd lambdas_;
  Eigen::VectorXd cop_;

  static constexpr double kWalkingHeight = 0.58; //zmp_ make parameter

  Jacobian GetJacobianWithRespectToLambdas() const;
  Jacobian GetJacobianWithRespectToContacts() const;
//  Jacobian GetJacobianWithRespectToComMotion() const;
  Jacobian GetJacobianWithRespectToCop() const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_ */
