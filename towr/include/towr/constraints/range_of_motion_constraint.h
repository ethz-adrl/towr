/**
 @file    range_of_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Declares various Range of Motion Constraint classes
 */

#ifndef TOWR_CONSTRAINTS_RANGE_OF_MOTION_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_RANGE_OF_MOTION_CONSTRAINT_H_

#include <string>
#include <vector>

//#include <ifopt/composite.h>
//
//#include <xpp_states/endeffectors.h>
//#include <xpp_states/state.h>

#include <towr/models/kinematic_model.h>
#include <towr/optimization_parameters.h>
#include <towr/variables/spline.h>
#include <towr/variables/angular_state_converter.h>
#include <towr/variables/spline_holder.h>

#include "time_discretization_constraint.h"


namespace towr {

/** @brief Constrains the contact to lie in a box around the nominal stance
  *
  * These constraints are necessary to avoid choosing contact locations
  * that are outside the kinematic reach of the robot. The constraint
  * is defined by Cartesian estimates of the reachability of each endeffector.
  *
  * This constraint calculates the position of of the contact expressed in the
  * current CoM frame and constrains it to lie in a box around the nominal/
  * natural contact position for that leg.
  */
class RangeOfMotionBox : public TimeDiscretizationConstraint {
public:
  using EndeffectorID = uint;
  using Vector3d = Eigen::Vector3d;

  RangeOfMotionBox(const KinematicModel::Ptr& robot_model,
                   const OptimizationParameters& params,
                   const EndeffectorID& ee,
                   const SplineHolder& spline_holder);
  virtual ~RangeOfMotionBox() = default;


private:
  void UpdateConstraintAtInstance (double t, int k, VectorXd& g) const override;
  void UpdateBoundsAtInstance (double t, int k, VecBound&) const override;
  virtual void UpdateJacobianAtInstance(double t, int k, Jacobian&, std::string) const override;

  int GetRow(int node, int dimension) const;

  Spline::Ptr base_linear_;
  Spline::Ptr base_angular_;
  Spline::Ptr ee_motion_;

  Eigen::Vector3d max_deviation_from_nominal_;
  Eigen::Vector3d nominal_ee_pos_B_;
  AngularStateConverter converter_;

  EndeffectorID ee_;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_RANGE_OF_MOTION_CONSTRAINT_H_ */
