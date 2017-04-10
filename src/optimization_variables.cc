/**
 @file    optimization_variables.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 28, 2017
 @brief   Brief description
 */

#include <xpp/optimization_variables.h>

namespace xpp {
namespace opt {

OptimizationVariables::OptimizationVariables (const std::string& id)
{
  id_ = id;
}

OptimizationVariables::~OptimizationVariables ()
{
}

int
OptimizationVariables::GetOptVarCount () const
{
  return GetVariables().rows();
}

std::string
OptimizationVariables::GetId () const
{
  return id_;
}

VecBound
OptimizationVariables::GetBounds () const
{
  // default value if user hasn't set anything
  if (bounds_.empty())
    SetAllBounds(kNoBound_);

  return bounds_;
}

void
OptimizationVariables::SetAllBounds (const Bound& bound) const
{
  bounds_ = VecBound(GetOptVarCount(), bound);
}

} /* namespace opt */
} /* namespace xpp */

