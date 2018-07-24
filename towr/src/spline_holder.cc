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

#include <towr/variables/spline_holder.h>
#include <towr/variables/phase_spline.h>

namespace towr{

SplineHolder::SplineHolder (NodesVariables::Ptr base_lin_nodes,
                            NodesVariables::Ptr base_ang_nodes,
                            const std::vector<double>& base_poly_durations,
                            std::vector<NodesVariablesPhaseBased::Ptr> ee_motion_nodes,
                            std::vector<NodesVariablesPhaseBased::Ptr> ee_force_nodes,
                            std::vector<PhaseDurations::Ptr> phase_durations,
                            bool durations_change)
{
  base_linear_  = std::make_shared<NodeSpline>(base_lin_nodes.get(), base_poly_durations);
  base_angular_ = std::make_shared<NodeSpline>(base_ang_nodes.get(), base_poly_durations);
  phase_durations_ = phase_durations;

  for (uint ee=0; ee<ee_motion_nodes.size(); ++ee) {
    if (durations_change) {
      // spline that changes the polynomial durations (affects Jacobian)
      ee_motion_.push_back(std::make_shared<PhaseSpline>(ee_motion_nodes.at(ee), phase_durations.at(ee).get()));
      ee_force_.push_back(std::make_shared<PhaseSpline>(ee_force_nodes.at(ee), phase_durations.at(ee).get()));
    } else {
      // spline without changing the polynomial durations
      auto ee_motion_poly_durations = ee_motion_nodes.at(ee)->ConvertPhaseToPolyDurations(phase_durations.at(ee)->GetPhaseDurations());
      auto ee_force_poly_durations = ee_force_nodes.at(ee)->ConvertPhaseToPolyDurations(phase_durations.at(ee)->GetPhaseDurations());

      ee_motion_.push_back(std::make_shared<NodeSpline>(ee_motion_nodes.at(ee).get(), ee_motion_poly_durations));
      ee_force_.push_back (std::make_shared<NodeSpline>(ee_force_nodes.at(ee).get(), ee_force_poly_durations));
    }
  }
}

} /* namespace towr */
