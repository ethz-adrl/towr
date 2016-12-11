/**
 @file    snopt_adapter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 4, 2016
 @brief   Defines the SnoptAdapter class
 */

#include <xpp/opt/snopt_adapter.h>

namespace xpp {
namespace opt {

SnoptAdapter::SelfPtr SnoptAdapter::instance_ = nullptr;

SnoptAdapter::SelfPtr
SnoptAdapter::GetInstance ()
{
  if (!instance_)
    instance_ = new SnoptAdapter();
  return instance_;
}

void
SnoptAdapter::SetNLP (NLPPtr& nlp)
{
  nlp_ = std::move(nlp);
}

void
SnoptAdapter::Init ()
{
  int obj_count = nlp_->HasCostTerms()? 1 : 0;
  n    = nlp_->GetNumberOfOptimizationVariables();
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
  lenG  = obj_count*n + nlp_->GetJacobianOfConstraints()->nonZeros();
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
  for (int k=0; k<jac->outerSize(); ++k) {
    for (NLP::Jacobian::InnerIterator it(*jac,k); it; ++it) {
      iGfun[neG] = it.row() + obj_count;
      jGvar[neG] = it.col();
      neG++;
    }
  }

  setProbName   ( "snopt" );
  setSpecsFile  ( "snopt.spc" );
//  setPrintFile  ( "snopt.out" ); // appends the file

  setUserFun    (&SnoptAdapter::ObjectiveAndConstraintFct);
  setIntParameter( "Derivative option", 1 ); // 1 = snopt will not calculate missing derivatives
//  setIntParameter( "Verify level ", 3 );
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
    if (instance_->nlp_->HasCostTerms())
      F[i++] = instance_->nlp_->EvaluateCostFunction(x);

    // the vector of constraint values
    VectorXd g_eig = instance_->nlp_->EvaluateConstraints(x);
    Eigen::Map<VectorXd>(F+i, g_eig.rows()) = g_eig; // should work as well
  }


  if ( *needG > 0 ) {

    int i=0;
    // the jacobian of the first row (cost function)
    if (instance_->nlp_->HasCostTerms()) {
      Eigen::VectorXd grad = instance_->nlp_->EvaluateCostFunctionGradient(x);
      i = grad.rows();
      Eigen::Map<VectorXd>(G, i) = grad;
    }

    // the jacobian of all the constraints
    instance_->nlp_->EvalNonzerosOfJacobian(x, G+i);
  }
}

SnoptAdapter::SnoptAdapter ()
{
}

SnoptAdapter::~SnoptAdapter ()
{
}

void
SnoptAdapter::SolveSQP (int start_type)
{
  snoptProblemA::solve(start_type);
  nlp_->SetVariables(x);

  // this seems to be necessary, to create a new snopt problem for every run
  delete instance_;
  instance_ = nullptr;
}

} /* namespace opt */
} /* namespace xpp */

