/*
 * range_of_motion_constraint.h
 *
 *  Created on: May 26, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_

#include "a_constraint.h"
#include "motion_structure.h"
#include <memory>

namespace xpp {

namespace hyq {
class SupportPolygonContainer;
}

namespace zmp {

class ComMotion;

class RangeOfMotionConstraint : public AConstraint {
public:
  using Contacts      = xpp::hyq::SupportPolygonContainer;
  using ComMotionPtrU = std::unique_ptr<ComMotion>;
  using ContactPtrU   = std::unique_ptr<Contacts>;
  using PosXY         = Eigen::Vector2d;
  using LegID         = xpp::hyq::LegID;

  RangeOfMotionConstraint ();
  virtual ~RangeOfMotionConstraint () {};

  void Init(const ComMotion&, const Contacts&);
  void UpdateVariables(const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  PosXY GetNominalPositionInBase(LegID leg) const; // move to support polygon

  void SetJacobianWrtContacts();
  void SetJacobianWrtMotion();

  ContactPtrU supp_polygon_container_;
  ComMotionPtrU com_motion_;

  MotionStructure::MotionInfoVec motion_info_;

  Jacobian jac_wrt_contacts_;
  Jacobian jac_wrt_motion_;

};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_ */
