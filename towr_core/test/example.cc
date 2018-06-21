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

#include <cmath>

#include <towr/towr.h>
#include <towr/models/centroidal_model.h>


using namespace towr;


// Generate even ground.
class FlatGround : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override { return 0.0; };
};


// The kinematic limits of a one-legged hopper
class MonopedKinematicModel : public KinematicModel {
public:
  MonopedKinematicModel () : KinematicModel(1)
  {
    nominal_stance_.at(0) = Eigen::Vector3d( 0.0, 0.0, -0.58);
    max_dev_from_nominal_ << 0.25, 0.15, 0.2;
  }
};


// The Centroidal dynamics of a one-legged hopper
class MonopedDynamicModel : public CentroidalModel {
public:
  MonopedDynamicModel()
  : CentroidalModel(20,                              // mass of the robot
                    1.2, 5.5, 6.0, 0.0, -0.2, -0.01, // base inertia
                    1) {}                            // number of endeffectors
};


int main()
{
  // Kinematic limits and dynamic parameters of the hopper
  RobotModel model;
  model.dynamic_model_   = std::make_shared<MonopedDynamicModel>();
  model.kinematic_model_ = std::make_shared<MonopedKinematicModel>();


  // set the initial position of the hopper
  BaseState initial_base;
  initial_base.lin.at(kPos).z() = 0.5;
  Eigen::Vector3d initial_foot_pos_W = Eigen::Vector3d::Zero();


  // define the desired goal state of the hopper
  BaseState goal;
  goal.lin.at(towr::kPos) << 1.0, 0.0, 0.5;


  // Parameters that define the motion. See c'tor for default values or
  // other values that can be modified.
  Parameters params;
  params.t_total_ = 1.6; // [s] time to reach goal state
  // here we define the initial phase durations, that can however be changed
  // by the optimizer. The number of swing and stance phases however is fixed.
  // alternating stance and swing:     ____-----_____-----_____-----_____
  params.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.2});
  params.ee_in_contact_at_start_.push_back(true);


  // Pass this information to the actual solver
  TOWR towr;
  towr.SetInitialState(initial_base, {initial_foot_pos_W});
  towr.SetParameters(goal, params, model, std::make_shared<FlatGround>());

  towr.SolveNLP();
  auto x = towr.GetSolution();


  // Print out the trajecetory at discrete time samples
  using namespace std;
  cout.precision(2);
  cout << fixed;
  cout << "\n====================\nMonoped trajectory:\n====================\n";

  double t = 0.0;
  while (t<=params.t_total_+1e-5) {

    cout << "t=" << t << "\n";
    cout << "Base linear position x,y,z:   \t";
    cout << x.base_linear_->GetPoint(t).p().transpose()     << "\t[m]" << endl;

    cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = x.base_angular_->GetPoint(t).p();
    cout << (rad/M_PI*180).transpose()                      << "\t[deg]" << endl;

    cout << "Foot position x,y,z:          \t";
    cout << x.ee_motion_.at(0)->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "Contact force x,y,z:          \t";
    cout << x.ee_force_.at(0)->GetPoint(t).p().transpose()  << "\t[N]" << endl;

    bool contact = x.phase_durations_.at(0)->IsContactPhase(t);
    std::string foot_in_contact = contact? "yes" : "no";
    cout << "Foot in contact:              \t" + foot_in_contact << endl;

    cout << endl;

    t += 0.2;
  }
}
