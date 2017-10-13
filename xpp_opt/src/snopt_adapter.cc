/**
 @file    snopt_adapter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 4, 2016
 @brief   Defines the SnoptAdapter class
 */

#include <xpp/solvers/snopt_adapter.h>

#include <iostream>
#include <string>
#include <vector>
#include <sys/types.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/composite.h>
#include <xpp/nlp_bound.h>


namespace xpp {

SnoptAdapter::NLPPtr SnoptAdapter::nlp_;

using VectorXd = Eigen::VectorXd;

SnoptAdapter::SnoptAdapter (NLP& ref)
{
  nlp_ = &ref;
}

SnoptAdapter::~SnoptAdapter ()
{
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
  VectorXd x_all = nlp_->GetStartingValues();
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

  setUserFun    (&SnoptAdapter::ObjectiveAndConstraintFct);
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
SnoptAdapter::Solve (NLP& ref)
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
    std::cerr << msg;
//    throw std::runtime_error(msg);
  }

   snopt.SetVariables();
}

void SnoptAdapter::SetOptions(SnoptAdapter& solver)
{
  // A complete list of options can be found in the snopt user guide:
  // https://web.stanford.edu/group/SOL/guides/sndoc7.pdf

  solver.setProbName( "snopt" );
//  solver.setSpecsFile  ( "snopt.spc" ); // to set options in config file

  solver.setIntParameter( "Major Print level", 1 );
  solver.setIntParameter( "Minor print level", 1 );
  solver.setParameter( "Solution");

  solver.setIntParameter( "Derivative option", 1 ); // 1 = snopt will not calculate missing derivatives
  solver.setIntParameter( "Verify level ", 3 ); // full check on gradients, will throw error

//  solver.setIntParameter("Major iterations limit", 2);
  solver.setIntParameter("Iterations limit", 200000);

  solver.setRealParameter( "Major feasibility tolerance",  1.0e-3); // target nonlinear constraint violation
  solver.setRealParameter( "Minor feasibility tolerance",  1.0e-3); // for satisfying the QP bounds
  solver.setRealParameter( "Major optimality tolerance",   1.0e-2); // target complementarity gap

  // These options strongly influce SNOPT based on the FAQ.snopt
  // file located in the downloaded source
//  solver.setIntParameter( "Crash option", 3 );
//  solver.setIntParameter( "Hessian updates", 5 );

//  solver.setParameter("Nonderivative linesearch");
}

void
SnoptAdapter::SetVariables ()
{
  nlp_->SetVariables(x);
}

} /* namespace xpp */

