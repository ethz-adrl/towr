/**
 @file    motion_structure.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Declares various Range of Motion Constraint Classes
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_

#include "a_constraint.h"
#include "motion_structure.h"
#include <xpp/utils/eigen_std_conversions.h>
#include <memory>

namespace xpp {
namespace opt {

class ComMotion;
class ARobotInterface;

/** @brief Base class for a constraint between contacts and CoM position.
  *
  * These constraints are necessary to avoid choosing contact locations
  * that are outside the kinematic reach of the robot. The constraint can
  * be defined in terms of joint limits or Cartesian estimates of the
  * reachability.
  */
class RangeOfMotionConstraint : public AConstraint {
public:
  using ComMotionPtrU = std::unique_ptr<ComMotion>;
  using RobotPtrU     = std::unique_ptr<ARobotInterface>;
  using PosXY         = Eigen::Vector2d;

  RangeOfMotionConstraint ();
  virtual ~RangeOfMotionConstraint ();

  void Init(const ComMotion&, const MotionStructure&, RobotPtrU);
  void UpdateVariables(const OptimizationVariables*) final;
  Jacobian GetJacobianWithRespectTo (std::string var_set) const final;

protected:
  utils::StdVecEigen2d footholds_;
  ComMotionPtrU com_motion_;
  MotionStructure motion_structure_;
  RobotPtrU robot_;
  bool first_update_ = true;

private:
  Jacobian jac_wrt_contacts_;
  Jacobian jac_wrt_motion_;

  virtual void SetJacobianWrtContacts(Jacobian&) const = 0;
  virtual void SetJacobianWrtMotion(Jacobian&) const = 0;
};

/** @brief Constrains the contact to lie in a box around the nominal stance
  *
  * This constraint calculates the position of of the contact expressed in the
  * current CoM frame and constrains it to lie in a box around the nominal/
  * natural contact position for that leg.
  */
class RangeOfMotionBox : public RangeOfMotionConstraint {
public:
  virtual VectorXd EvaluateConstraint () const final;
  virtual VecBound GetBounds () const final;

private:
  virtual void SetJacobianWrtContacts(Jacobian&) const final;
  virtual void SetJacobianWrtMotion(Jacobian&) const final;
};

///** @brief Constrains the contact to lie at a fixed position in world frame.
//  *
//  * This constraint places the footholds at predefined positions with no
//  * margin. It is mainly useful for debugging when other features of the optimizer
//  * are being tested.
//  */
//class RangeOfMotionFixed : public RangeOfMotionConstraint {
//public:
//  virtual VectorXd EvaluateConstraint () const final;
//  virtual VecBound GetBounds () const final;
//
//private:
//  const double kStepLength_ = 0.15;
//  virtual void SetJacobianWrtContacts(Jacobian&) const final;
//  virtual void SetJacobianWrtMotion(Jacobian&) const final;
//};


} /* namespace opt */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_ */
