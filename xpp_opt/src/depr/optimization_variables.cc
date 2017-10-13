/**
 @file    optimization_variables.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 28, 2017
 @brief   Brief description
 */

#include <string>

#include <xpp/opt/depr/optimization_variables.h>

#include <../../include/xpp_opt/nlp_bound.h>

namespace xpp {
namespace opt {

OptimizationVariables::OptimizationVariables (const std::string& id)
{
  name_ = id;
}

OptimizationVariables::~OptimizationVariables ()
{
}

int
OptimizationVariables::GetRows () const
{
  return GetValues().rows();
}

std::string
OptimizationVariables::GetName () const
{
  return name_;
}

VecBound
OptimizationVariables::GetBounds () const
{
  // default if user hasn't set anything
  return VecBound(GetRows(), kNoBound_);
}

//void
//OptimizationVariables::SetAllBounds (const Bound& bound) const
//{
//  bounds_ = VecBound(GetRows(), bound);
//}

} /* namespace opt */
} /* namespace xpp */

