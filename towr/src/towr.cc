/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

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
                   double total_time,
                   const RobotModel& model,
                   HeightMap::Ptr terrain)
{
  final_base_ = final_base;
  params_.SetTotalDuration(total_time);
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
