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

#include <towr/towr.h>

namespace towr {


TOWR::TOWR (bool print_boilerplate) {
  if (print_boilerplate) {
    using namespace std;
    cout << "\n";
    cout << "************************************************************\n";
    cout << " TOWR - Trajectory Optimizer for Walking Robots (v1.3)\n";
    cout << "                \u00a9 Alexander W. Winkler\n";
    cout << "           https://github.com/ethz-adrl/towr\n";
    cout << "************************************************************";
    cout << "\n\n";
  }
}

void
TOWR::SetInitialState(const BaseState& base, const FeetPos& feet)
{
  factory_.initial_base_ = base;
  factory_.initial_ee_W_ = feet;
}

void
TOWR::SetParameters(const BaseState& final_base,
                   const Parameters& params,
                   const RobotModel& model,
                   HeightMap::Ptr terrain)
{
  factory_.final_base_ = final_base;
  factory_.params_ = params;
  factory_.model_ = model;
  factory_.terrain_ = terrain;
}

void
TOWR::SolveNLP(const ifopt::Solver::Ptr& solver)
{
  nlp_ = BuildNLP();
  solver->Solve(nlp_);
  nlp_.PrintCurrent();
}

SplineHolder
TOWR::GetSolution() const
{
  return factory_.spline_holder_;
}

void
TOWR::SetSolution(int solver_iteration)
{
  nlp_.SetOptVariables(solver_iteration);
}

int
TOWR::GetIterationCount() const
{
  return nlp_.GetIterationCount();
}

ifopt::Problem
TOWR::BuildNLP()
{
  ifopt::Problem nlp;

  for (auto c : factory_.GetVariableSets())
    nlp.AddVariableSet(c);

  for (auto c : factory_.GetConstraints())
    nlp.AddConstraintSet(c);

  for (auto c : factory_.GetCosts())
    nlp.AddCostSet(c);

  return nlp;
}

} /* namespace towr */
