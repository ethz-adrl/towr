/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/variables/phase_spline.h>
#include <towr/variables/phase_durations.h>

namespace towr {

PhaseSpline::PhaseSpline(
    NodesVariablesPhaseBased::Ptr const nodes,
    PhaseDurations* const phase_durations)
    : NodeSpline(nodes.get(), nodes->ConvertPhaseToPolyDurations(phase_durations->GetPhaseDurations())),
      PhaseDurationsObserver(phase_durations)
{
  phase_nodes_ = nodes;

  UpdatePolynomialDurations();

  // if durations change, the polynomial active at a specified global time
  // changes. Therefore, all elements of the Jacobian could be non-zero
  // and must make sure that Jacobian structure never changes during
  // the iterations.
  // assume every global time time can fall into every polynomial
  for (int i=0; i<nodes->GetPolynomialCount(); ++i)
    FillJacobianWrtNodes(i, 0.0, kPos, jac_wrt_nodes_structure_, true);
}

void
PhaseSpline::UpdatePolynomialDurations()
{
  auto phase_duration = phase_durations_->GetPhaseDurations();
  auto poly_durations = phase_nodes_->ConvertPhaseToPolyDurations(phase_duration);

  for (int i=0; i<cubic_polys_.size(); ++i) {
    cubic_polys_.at(i).SetDuration(poly_durations.at(i));
  }

  UpdatePolynomialCoeff();
}

PhaseSpline::Jacobian
PhaseSpline::GetJacobianOfPosWrtDurations (double t_global) const
{
  VectorXd dx_dT  = GetDerivativeOfPosWrtPhaseDuration(t_global);
  VectorXd xd     = GetPoint(t_global).v();
  int current_phase = GetSegmentID(t_global, phase_durations_->GetPhaseDurations());

  return phase_durations_->GetJacobianOfPos(current_phase, dx_dT, xd);
}

Eigen::VectorXd
PhaseSpline::GetDerivativeOfPosWrtPhaseDuration (double t_global) const
{
  int poly_id; double t_local;
  std::tie(poly_id, t_local) = GetLocalTime(t_global, GetPolyDurations());

  VectorXd vel  = GetPoint(t_global).v();
  VectorXd dxdT = cubic_polys_.at(poly_id).GetDerivativeOfPosWrtDuration(t_local);

  double inner_derivative = phase_nodes_->GetDerivativeOfPolyDurationWrtPhaseDuration(poly_id);
  double prev_polys_in_phase = phase_nodes_->GetNumberOfPrevPolynomialsInPhase(poly_id);

  // where this minus term comes from:
  // from number of polynomials before current polynomial that
  // cause shifting of entire spline
  return inner_derivative*(dxdT - prev_polys_in_phase*vel);
}


} /* namespace towr */
