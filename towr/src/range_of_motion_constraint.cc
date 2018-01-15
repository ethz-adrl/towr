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

#include <towr/constraints/range_of_motion_constraint.h>

#include <memory>
#include <Eigen/Eigen>

#include <towr/variables/variable_names.h>


namespace towr {


RangeOfMotionBox::RangeOfMotionBox (const KinematicModel::Ptr& model,
                                    const OptimizationParameters& params,
                                    const EE& ee,
                                    const SplineHolder& spline_holder)
    :TimeDiscretizationConstraint(params.GetTotalTime(),
                                  params.dt_range_of_motion_,
                                  "RangeOfMotionBox-" + std::to_string(ee))
{
  ee_ = ee;

  base_linear_  = spline_holder.GetBaseLinear();
  base_angular_ = spline_holder.GetBaseAngular();
  ee_motion_    = spline_holder.GetEEMotion(ee);
  converter_    = AngularStateConverter(base_angular_);

  max_deviation_from_nominal_ = model->GetMaximumDeviationFromNominal();
  nominal_ee_pos_B_           = model->GetNominalStanceInBase().at(ee);
  SetRows(GetNumberOfNodes()*k3D);
}

int
RangeOfMotionBox::GetRow (int node, int dim) const
{
  return node*k3D + dim;
}

void
RangeOfMotionBox::UpdateConstraintAtInstance (double t, int k, VectorXd& g) const
{
  Vector3d base_W = base_linear_->GetPoint(t).p();
  AngularStateConverter::MatrixSXd b_R_w = converter_.GetRotationMatrixBaseToWorld(t).transpose();
  Vector3d pos_ee_B = b_R_w*(ee_motion_->GetPoint(t).p() - base_W);

  g.middleRows(GetRow(k, X), k3D) = pos_ee_B;
}

void
RangeOfMotionBox::UpdateBoundsAtInstance (double t, int k, VecBound& bounds) const
{
  using namespace ifopt;

  for (int dim=0; dim<k3D; ++dim) {
    Bounds b;
    b += nominal_ee_pos_B_(dim);
    b.upper_ += max_deviation_from_nominal_(dim);
    b.lower_ -= max_deviation_from_nominal_(dim);
    bounds.at(GetRow(k,dim)) = b;
  }
}

void
RangeOfMotionBox::UpdateJacobianAtInstance (double t, int k, Jacobian& jac,
                                            std::string var_set) const
{
  AngularStateConverter::MatrixSXd b_R_w = converter_.GetRotationMatrixBaseToWorld(t).transpose();
  int row_start = GetRow(k,X);

  if (var_set == id::base_lin_nodes) {
    jac.middleRows(row_start, k3D) = -1*b_R_w*base_linear_->GetJacobianWrtNodes(t, kPos);
  }

  if (var_set == id::base_ang_nodes) {
    Vector3d base_W   = base_linear_->GetPoint(t).p();
    Vector3d ee_pos_W = ee_motion_->GetPoint(t).p();
    Vector3d r_W = ee_pos_W - base_W;
    jac.middleRows(row_start, k3D) = converter_.GetDerivativeOfRotationMatrixRowWrtCoeff(t,r_W, true);
  }

  if (var_set == id::EEMotionNodes(ee_)) {
    jac.middleRows(row_start, k3D) = b_R_w*ee_motion_->GetJacobianWrtNodes(t,kPos);
  }

  if (var_set == id::EESchedule(ee_)) {
    jac.middleRows(row_start, k3D) = b_R_w*ee_motion_->GetJacobianOfPosWrtDurations(t);
  }

}

} /* namespace xpp */

