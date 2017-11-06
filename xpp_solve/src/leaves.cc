/**
 @file    leafs.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 6, 2017
 @brief   Brief description
 */

#include <xpp_solve/leaves.h>

namespace opt {


Constraint::Constraint (const VariablesPtr& variables,
            int row_count,
            const std::string& name) : Component(row_count, name)
{
  variables_ = variables;
}

Constraint::Jacobian
Constraint::GetJacobian () const
{
  Jacobian jacobian(GetRows(), variables_->GetRows());

  int col = 0;
  for (const auto& vars : variables_->GetNonzeroComponents()) {

    int n = vars->GetRows();
    Jacobian jac = Jacobian(GetRows(), n);

    FillJacobianBlock(vars->GetName(), jac);

    // insert the derivative in the correct position in the overall Jacobian
    for (int k=0; k<jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac,k); it; ++it)
        jacobian.coeffRef(it.row(), col+it.col()) = it.value();

    col += n;
  }

  return jacobian;
}


Cost::Cost (const VariablesPtr& variables, const std::string& name)
   :Constraint(variables, 1, name)
{
}


} /* namespace opt */
