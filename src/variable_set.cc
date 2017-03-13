/**
 @file    variable_set.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Defines the class VariableSet
 */

#include <xpp/variable_set.h>

namespace xpp {
namespace opt {


VariableSet::VariableSet (const VectorXd& values, std::string id, const VecBound& bounds)
{
  x_ = values;
  bounds_  = bounds;
  id_ = id;
}

VariableSet::VariableSet (const VectorXd& values, std::string id, const Bound& bound)
    :VariableSet(values,
                 id,
                 VecBound(values.rows(), bound)) // delegate to top c'tor
{
}

VariableSet::~VariableSet ()
{
}

std::string
VariableSet::GetId () const
{
  return id_;
}

VariableSet::VectorXd
VariableSet::GetVariables () const
{
  return x_;
}

VecBound
VariableSet::GetBounds () const
{
  return bounds_;
}

void
VariableSet::SetVariables (const VectorXd& x)
{
  x_ = x;
}

} // namespace opt
} // namespace xpp
