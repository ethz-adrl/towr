/**
 @file    variable_set.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Defines the class VariableSet
 */

#include <xpp/opt/variable_set.h>

namespace xpp {
namespace opt {

// zmp_ delegate to other constructor
VariableSet::VariableSet (const VectorXd& values, std::string id, const Bound& bound)
{
  x_ = values;
  bounds_.assign(values.rows(), bound);
  id_ = id;
}

VariableSet::VariableSet (const VectorXd& values, std::string id, const VecBound& bounds)
{
  x_ = values;
  bounds_  = bounds;
  id_ = id;
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

VariableSet::VecBound
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
