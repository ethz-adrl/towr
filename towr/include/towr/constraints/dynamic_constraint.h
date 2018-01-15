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

#ifndef TOWR_CONSTRAINTS_DYNAMIC_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_DYNAMIC_CONSTRAINT_H_

#include <string>
#include <vector>

#include <towr/models/dynamic_model.h>
#include <towr/variables/spline.h>
#include <towr/variables/angular_state_converter.h>
#include <towr/variables/spline_holder.h>

#include "time_discretization_constraint.h"

namespace towr {

class DynamicConstraint : public TimeDiscretizationConstraint {
public:
  using VecTimes = std::vector<double>;
  using Vector3d = Eigen::Vector3d;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  DynamicConstraint (const DynamicModel::Ptr& m,
                     const VecTimes& evaluation_times,
                     const SplineHolder& spline_holder);
  virtual ~DynamicConstraint () = default;

private:

  Spline::Ptr base_linear_;
  Spline::Ptr base_angular_;
  std::vector<Spline::Ptr> ee_forces_;
  std::vector<Spline::Ptr> ee_motion_;

  int n_ee_;

  mutable DynamicModel::Ptr model_;
  double gravity_;
  AngularStateConverter converter_;

  int GetRow(int node, Dim6D dimension) const;

  virtual void UpdateConstraintAtInstance(double t, int k, VectorXd& g) const override;
  virtual void UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const override;
  virtual void UpdateJacobianAtInstance(double t, int k, Jacobian&, std::string) const override;

  void UpdateModel(double t) const;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_DYNAMIC_CONSTRAINT_H_ */
