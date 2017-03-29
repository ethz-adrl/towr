/**
 @file    range_of_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Declares various Range of Motion Constraint classes
 */

#ifndef XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_
#define XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_

#include "endeffectors_motion.h"
#include <memory>
#include "../constraint.h"

namespace xpp {
namespace opt {

class BaseMotion;

/** @brief Base class for a constraint between contacts and CoM position.
  *
  * These constraints are necessary to avoid choosing contact locations
  * that are outside the kinematic reach of the robot. The constraint can
  * be defined in terms of joint limits or Cartesian estimates of the
  * reachability.
  */
class RangeOfMotionConstraint : public Constraint {
public:
  using ComMotionPtrU = std::shared_ptr<BaseMotion>;
  using EEMotionPtr   = std::shared_ptr<EndeffectorsMotion>;
  using PosXY         = Eigen::Vector2d;

  RangeOfMotionConstraint ();
  virtual ~RangeOfMotionConstraint ();

  void Init(const ComMotionPtrU& com_motion,
            const EEMotionPtr& ee_motion,
            double dt);

protected:
  ComMotionPtrU com_motion_;
  EEMotionPtr ee_motion_;
  std::vector<double> dts_; ///< discretization of constraint

private:
  virtual void InitializeConstantJacobians() = 0;
};

/** @brief Constrains the contact to lie in a box around the nominal stance
  *
  * This constraint calculates the position of of the contact expressed in the
  * current CoM frame and constrains it to lie in a box around the nominal/
  * natural contact position for that leg.
  */
class RangeOfMotionBox : public RangeOfMotionConstraint {
public:
  using MaxDevXY       = std::array<double,2>;
  using NominalStance  = EEXppPos;

  /** @param dev  How much the endeffector can deviate from the default (x,y)
    * position while still remaining in the range of motion.
    */
  RangeOfMotionBox(const MaxDevXY& dev, const NominalStance& nom);

  void UpdateConstraintValues () override;
  virtual VecBound GetBounds () const override;


private:
  void InitializeConstantJacobians() override;
  void UpdateJacobianWrtEndeffectors();
  void UpdateJacobianWrtBase();

  MaxDevXY max_deviation_from_nominal_;
  NominalStance nominal_stance_;
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

#endif /* XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_ */
