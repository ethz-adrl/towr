/**
 @file    snopt_adapter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 4, 2016
 @brief   Brief description
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
  n    = nlp_->GetNumberOfOptimizationVariables(); // number of optimization variables

  int m = nlp_->GetNumberOfConstraints();
  neF   = nlp_->HasCostTerms()? m+1 : m;

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


  // specify the sparsity pattern of the jacobian
  // specify the row/colum of the nonzero nonlinear gradients that must be estimated
//  int neG = 0;
//  iGfun[neG] = 1;
//  jGvar[neG] = 0;
//  neG++;

  // linear derivatives
//  int     lenA, lenG, neA, neG;
//  int    *iAfun, *jAvar, *iGfun, *jGvar;
  lenA = 0;
  neA = 0;
  iAfun = nullptr;
  jAvar = nullptr;
  A = nullptr;




  // derivatives of nonlinear part
  // maximum number of non-zero nonlinear elements in F
  // conservative guess: take all
  lenG  = neF*n;//nlp_->GetJacobianOfConstraints()->nonZeros();
  iGfun = new int[lenG];
  jGvar = new int[lenG];


  // the gradient terms of the cost function
  neG=0; // nonzero cells in jacobian

  if(nlp_->HasCostTerms()) {
    for (int var=0; var<n; ++var) {
      iGfun[neG] = 0;
      jGvar[neG] = var;
      neG++;
    }
  }


  // the derivative of the constraints
  int row_start = nlp_->HasCostTerms()? 1 : 0;
  auto jac = nlp_->GetJacobianOfConstraints();
  for (int k=0; k<jac->outerSize(); ++k) {
    for (NLP::Jacobian::InnerIterator it(*jac,k); it; ++it) {
      iGfun[neG] = it.row() + row_start;
      jGvar[neG] = it.col();
      neG++;
    }
  }


  setProbName   ( "snopt" );
  setSpecsFile  ( "snopt.spc" );
//  setPrintFile  ( "snopt.out" ); // appends the file

  setProblemSize( n, neF );
  setObjective  ( ObjRow, ObjAdd );
  setX          ( x, xlow, xupp, xmul, xstate );
  setF          ( F, Flow, Fupp, Fmul, Fstate );
  setA          ( lenA, neA, iAfun, jAvar, A);
  setG          ( lenG, neG, iGfun, jGvar );
  setUserFun    (&SnoptAdapter::ObjectiveAndConstraintFct);

  setIntParameter( "Derivative option", 1 ); // 1 = snopt will not calculate missing derivatives
//  setIntParameter( "Verify level ", 3 );
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

void
SnoptAdapter::ObjectiveAndConstraintFct (int* Status, int* n, double x[],
                                         int* needF, int* neF, double F[],
                                         int* needG, int* neG, double G[],
                                         char* cu, int* lencu, int iu[],
                                         int* leniu, double ru[], int* lenru)
{
  if ( *needF > 0 ) {
    int c=0;
    if (instance_->nlp_->HasCostTerms())
      F[c++] = instance_->nlp_->EvaluateCostFunction(x);

    VectorXd g_eig = instance_->nlp_->EvaluateConstraints(x);
    //  Eigen::Map<VectorXd>(g,m) = g_eig;

    // use this way
    //  Eigen::Map<VectorXd>(F+c, g_eig.rows()) = g_eig; // should work as well
    for (int i=0; i<g_eig.rows(); ++i) {
      F[c++] = g_eig[i];
    }
  }


  if ( *needG > 0 ) {

    int c = 0;
    if (instance_->nlp_->HasCostTerms()) {
      Eigen::VectorXd grad = instance_->nlp_->EvaluateCostFunctionGradient(x);
      for (int i=0; i<grad.rows(); ++i) {
        G[c++] = grad[i];
      }
    }

    instance_->nlp_->EvalNonzerosOfJacobian(x, G+c);

//    double dg[*neG];
//    instance_->nlp_->EvalNonzerosOfJacobian(x, dg);
//
//    for (int j=0; j<*neG; ++j) {
//      G[c++] = dg[j];
//    }

  }

// example
//  F[0] =  x[1];
//  F[1] =  x[0]*x[0] + 4*x[1]*x[1];
//  F[2] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
}

SnoptAdapter::SnoptAdapter ()
{
}

SnoptAdapter::~SnoptAdapter ()
{
  // TODO Auto-generated destructor stub
}

} /* namespace zmp */
} /* namespace xpp */

