/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_CONSTRAINTS_RANGE_OF_MOTION_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_RANGE_OF_MOTION_CONSTRAINT_H_

#include <string>
#include <vector>

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
  using EE = uint;
  using Vector3d = Eigen::Vector3d;

  RangeOfMotionBox(const KinematicModel::Ptr& robot_model,
                   const OptimizationParameters& params,
                   const EE& ee,
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

  EE ee_;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_RANGE_OF_MOTION_CONSTRAINT_H_ */
