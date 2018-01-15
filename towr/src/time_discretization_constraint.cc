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

#include <towr/constraints/time_discretization_constraint.h>

#include <cmath>
#include <initializer_list>
#include <Eigen/Dense>

namespace towr {


TimeDiscretizationConstraint::TimeDiscretizationConstraint (double T, double dt,
                                                            const std::string& name)
    :ConstraintSet(kSpecifyLater, name)
{
  double t = 0.0;
  dts_ = {t};

  for (int i=0; i<floor(T/dt); ++i) {
    t += dt;
    dts_.push_back(t);
  }

  dts_.push_back(T); // also ensure constraints at very last node/time.
}

TimeDiscretizationConstraint::TimeDiscretizationConstraint (const EvaluationTimes& times,
                                                            const std::string& name)
   :ConstraintSet(kSpecifyLater, name) // just placeholder
{
  dts_ = times;
}

TimeDiscretizationConstraint::~TimeDiscretizationConstraint ()
{
}

int
TimeDiscretizationConstraint::GetNumberOfNodes () const
{
  return dts_.size();
}

TimeDiscretizationConstraint::VectorXd
TimeDiscretizationConstraint::GetValues () const
{
  VectorXd g = VectorXd::Zero(GetRows());

  int k = 0;
  for (double t : dts_)
    UpdateConstraintAtInstance(t, k++, g);

  return g;
}

TimeDiscretizationConstraint::VecBound
TimeDiscretizationConstraint::GetBounds () const
{
  VecBound bounds(GetRows());

  int k = 0;
  for (double t : dts_)
    UpdateBoundsAtInstance(t, k++, bounds);

  return bounds;
}

void
TimeDiscretizationConstraint::FillJacobianBlock (std::string var_set,
                                                  Jacobian& jac) const
{
  int k = 0;
  for (double t : dts_)
    UpdateJacobianAtInstance(t, k++, jac, var_set);
}

} /* namespace towr */


