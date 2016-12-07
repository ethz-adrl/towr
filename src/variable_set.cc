/**
 @file    variable_set.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Defines the class VariableSet
 */

#include <xpp/opt/variable_set.h>

namespace xpp {
namespace opt {

VariableSet::VariableSet (int n_variables, std::string id, const VarBound& bound)
{
  x_ = Eigen::VectorXd::Zero(n_variables);
  bounds_.assign(n_variables, bound);
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


