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

#include <towr_ros/towr_ros.h>
#include <towr/initialization/gait_generator.h>


namespace towr {


class TowrApp : public TowrRos {
public:
  void SetSolverParameters(const TowrCommandMsg& msg) override {
    solver_->SetOption("linear_solver", "mumps");
    solver_->SetOption("jacobian_approximation", "exact");
    solver_->SetOption("max_cpu_time", 40.0);
    solver_->SetOption("print_level", 5);
    //  solver_->SetOption("derivative_test", "first-order");
    //  solver_->SetOption("max_iter", 0);

    // modify solver parameters
    if (msg.play_initialization)
      solver_->SetOption("max_iter", 0);
    else
      solver_->SetOption("max_iter", 3000);
  }

  void SetTowrInitialState(const std::vector<Eigen::Vector3d>& nominal_stance_B) override {
    double z_ground = 0.0;
    std::vector<Eigen::Vector3d> initial_ee_pos =  nominal_stance_B;
    std::for_each(initial_ee_pos.begin(), initial_ee_pos.end(),
                  [&](Vector3d& p){ p.z() = z_ground; } // feet at 0 height
    );

    BaseState initial_base;
    initial_base.lin.at(kPos).z() = - nominal_stance_B.front().z() + z_ground;

    towr_.SetInitialState(initial_base, initial_ee_pos);
  }

  Parameters GetTowrParameters(int n_ee, const TowrCommandMsg& msg) const override {
    Parameters params;
    auto gait_gen_ = GaitGenerator::MakeGaitGenerator(n_ee);
    auto id_gait   = static_cast<GaitGenerator::Combos>(msg.gait);
    gait_gen_->SetCombo(id_gait);
    for (int ee=0; ee<n_ee; ++ee) {
      params.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(msg.total_duration, ee));
      params.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
    }

    params.SetSwingConstraint();

    if (msg.optimize_phase_durations)
      params.OptimizePhaseDurations();

    return params;
  }
};

} // namespace towr

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "towr_app");
  towr::TowrApp towr_app;
  ros::spin();

  return 1;
}
