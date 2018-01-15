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

#include <towr/towr.h>

#include <ifopt/solvers/ipopt_adapter.h>
#include <ifopt/solvers/snopt_adapter.h>

#include <towr/nlp_factory.h>


namespace towr {


void
TOWR::SetInitialState(const BaseState& initial_base, const FeetPos& feet)
{
  initial_base_ = initial_base;
  foot_pos_ = feet;
}

void
TOWR::SetParameters(const BaseState& final_base,
                    const OptimizationParameters& params,
                    const RobotModel& model,
                    HeightMap::Ptr terrain)
{
  final_base_ = final_base;
  params_ = params;
  model_ = model;
  terrain_ = terrain;
}

ifopt::Problem
TOWR::BuildNLP (SplineHolder& spline_holder) const
{
  ifopt::Problem nlp;

  NlpFactory factory;
  factory.Init(params_, terrain_, model_, foot_pos_, initial_base_, final_base_);

  for (auto c : factory.GetVariableSets(spline_holder))
    nlp.AddVariableSet(c);

  for (ConstraintName name : params_.GetUsedConstraints())
    for (auto c : factory.GetConstraint(name))
      nlp.AddConstraintSet(c);

  for (const auto& pair : params_.GetCostWeights())
    for (auto c : factory.GetCost(pair.first, pair.second))
      nlp.AddCostSet(c);

  return nlp;
}

void TOWR::SolveNLP(Solver solver)
{
  nlp_ = BuildNLP(spline_holder_);

  switch (solver) {
    case Ipopt: ifopt::IpoptAdapter::Solve(nlp_); break;
    case Snopt: ifopt::SnoptAdapter::Solve(nlp_); break;
    default:  assert(false); // solver not implemented
  }

  nlp_.PrintCurrent();
}

SplineHolder
TOWR::GetSolution() const
{
  return spline_holder_;
}

void
TOWR::SetSolution(int iter)
{
  nlp_.SetOptVariables(iter);
}

int
TOWR::GetIterationCount() const
{
  return nlp_.GetIterationCount();
}


} /* namespace towr */
