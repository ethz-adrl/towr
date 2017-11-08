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

#include <opt_solve/solvers/snopt_adapter.h>

namespace opt {

SnoptAdapter::NLPPtr SnoptAdapter::nlp_;

void
SnoptAdapter::Solve (Problem& ref)
{
  int Cold = 0, Basis = 1, Warm = 2;

  SnoptAdapter snopt(ref);
  snopt.Init();
  SetOptions(snopt);

  // error codes as given in the manual.
  int INFO = snopt.solve(Cold);
  int EXIT = INFO - INFO%10; // change least significant digit to zero

  if (EXIT != 0) {
    std::string msg = "Snopt failed to find a solution. EXIT:" + std::to_string(EXIT) + ", INFO:" + std::to_string(INFO);
    throw std::runtime_error(msg);
  }

  snopt.SetVariables();
}

void SnoptAdapter::SetOptions(SnoptAdapter& solver)
{
  // A complete list of options can be found in the snopt user guide:
  // https://web.stanford.edu/group/SOL/guides/sndoc7.pdf

  // solver.setSpecsFile( "snopt.spc" ); // to set options in config file
  solver.setProbName( "snopt" );

  solver.setIntParameter( "Major Print level", 1 );
  solver.setIntParameter( "Minor Print level", 1 );
  solver.setParameter( "Solution");

  solver.setIntParameter( "Derivative option", 1 ); // 1 = snopt will not calculate missing derivatives
  solver.setIntParameter( "Verify level ", 3 ); // full check on gradients, will throw error

  solver.setIntParameter("Iterations limit", 200000);
  // solver.setIntParameter("Major iterations limit", 2);

  solver.setRealParameter( "Major feasibility tolerance",  1.0e-3); // target nonlinear constraint violation
  solver.setRealParameter( "Minor feasibility tolerance",  1.0e-3); // for satisfying the QP bounds
  solver.setRealParameter( "Major optimality tolerance",   1.0e-2); // target complementarity gap

  // solver.setIntParameter( "Crash option", 3 );
  // solver.setIntParameter( "Hessian updates", 5 );
  // solver.setParameter("Nonderivative linesearch");
}

SnoptAdapter::SnoptAdapter (Problem& ref)
{
  nlp_ = &ref;
}

void
SnoptAdapter::Init ()
{
  int obj_count = nlp_->HasCostTerms()? 1 : 0;
  n     = nlp_->GetNumberOfOptimizationVariables();
  neF   = nlp_->GetNumberOfConstraints() + obj_count;

  x      = new double[n];
  xlow   = new double[n];
  xupp   = new double[n];
  xmul   = new double[n];
  xstate = new    int[n];

  F      = new double[neF];
  Flow   = new double[neF];
  Fupp   = new double[neF];
  Fmul   = new double[neF];
  Fstate = new    int[neF];

  // Set the upper and lower bounds.
  // no bounds on the spline coefficients or footholds
  auto bounds_x = nlp_->GetBoundsOnOptimizationVariables();
  for (uint _n=0; _n<bounds_x.size(); ++_n) {
    xlow[_n] = bounds_x.at(_n).lower_;
    xupp[_n] = bounds_x.at(_n).upper_;
    xstate[_n] = 0;
  }

  // bounds on the cost function if it exists
  int c = 0;
  if (nlp_->HasCostTerms()) {
    Flow[c] = -1e20; Fupp[c] = 1e20; Fmul[c] = 0.0; // no bounds on cost function
    c++;
  }

  // bounds on equality and inequality constraints
  auto bounds_g = nlp_->GetBoundsOnConstraints();
  for (const auto& b : bounds_g) {
    Flow[c] = b.lower_; Fupp[c] = b.upper_; Fmul[c] = 0.0;
    c++;
  }

  // initial values of the optimization
  VectorXd x_all = nlp_->GetVariableValues();
  Eigen::Map<VectorXd>(&x[0], x_all.rows()) = x_all;

  ObjRow  = nlp_->HasCostTerms()? 0 : -1; // the row in user function that corresponds to the objective function
  ObjAdd  = 0.0;                          // the constant to be added to the objective function

  // no linear derivatives/just assume all are nonlinear
  lenA = 0;
  neA = 0;
  iAfun = nullptr;
  jAvar = nullptr;
  A = nullptr;

  // derivatives of nonlinear part
  lenG  = obj_count*n + nlp_->GetJacobianOfConstraints().nonZeros();
  iGfun = new int[lenG];
  jGvar = new int[lenG];

  // the gradient terms of the cost function
  neG=0; // nonzero cells in jacobian of Cost Function AND Constraints

  // the nonzero elements of cost function (assume all)
  if(nlp_->HasCostTerms()) {
    for (int var=0; var<n; ++var) {
      iGfun[neG] = 0;
      jGvar[neG] = var;
      neG++;
    }
  }

  // the nonzero elements of constraints (assume all)
  auto jac = nlp_->GetJacobianOfConstraints();
  for (int k=0; k<jac.outerSize(); ++k) {
    for (Jacobian::InnerIterator it(jac,k); it; ++it) {
      iGfun[neG] = it.row() + obj_count;
      jGvar[neG] = it.col();
      neG++;
    }
  }

  setUserFun(&SnoptAdapter::ObjectiveAndConstraintFct);
}

void
SnoptAdapter::ObjectiveAndConstraintFct (int* Status, int* n, double x[],
                                         int* needF, int* neF, double F[],
                                         int* needG, int* neG, double G[],
                                         char* cu, int* lencu, int iu[],
                                         int* leniu, double ru[], int* lenru)
{
  if ( *needF > 0 ) {
    int i=0;

    // the scalar objective function value
    if (nlp_->HasCostTerms())
      F[i++] = nlp_->EvaluateCostFunction(x);

    // the vector of constraint values
    VectorXd g_eig = nlp_->EvaluateConstraints(x);
    Eigen::Map<VectorXd>(F+i, g_eig.rows()) = g_eig;
  }


  if ( *needG > 0 ) {
    int i=0;

    // the jacobian of the first row (cost function)
    if (nlp_->HasCostTerms()) {
      Eigen::VectorXd grad = nlp_->EvaluateCostFunctionGradient(x);
      i = grad.rows();
      Eigen::Map<VectorXd>(G, i) = grad;
    }

    // the jacobian of all the constraints
    nlp_->EvalNonzerosOfJacobian(x, G+i);
    nlp_->SaveCurrent();
  }
}

void
SnoptAdapter::SetVariables ()
{
  nlp_->SetVariables(x);
}

SnoptAdapter::~SnoptAdapter ()
{
}


} /* namespace opt */
