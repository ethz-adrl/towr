/**
 @file    snopt_adapter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 4, 2016
 @brief   Brief description
 */

#include <xpp/zmp/snopt_adapter.h>

namespace xpp {
namespace zmp {

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
  n   = nlp_->GetNumberOfOptimizationVariables(); // number of optimization variables
  neF = nlp_->GetNumberOfConstraints()+1;         // +1 because of cost function

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
  // no bounds on the spline coefficients of footholds
  auto bounds_x = nlp_->GetBoundsOnOptimizationVariables();
  for (uint c=0; c<bounds_x.size(); ++c) {
    xlow[c] = bounds_x.at(c).lower_;
    xupp[c] = bounds_x.at(c).upper_;
    xstate[c] = 0;
  }

  // specific bounds depending on equality and inequality constraints
  Flow[0] = -1e20; Fupp[0] = 1e20; Fmul[0] = 0.0;
  auto bounds_g = nlp_->GetBoundsOnConstraints();
  for (uint c=0; c<bounds_g.size(); ++c) {
    Flow[c+1] = bounds_g.at(c).lower_;
    Fupp[c+1] = bounds_g.at(c).upper_;
    Fmul[c+1] = 0.0;
  }

  // initial values of the optimization
  VectorXd x_all = nlp_->GetStartingValues();
  Eigen::Map<VectorXd>(&x[0], x_all.rows()) = x_all;


  ObjRow  = 0;   // the row in user function that corresponds to the objective function
  ObjAdd  = 0.0; // the constant to be added to the objective function

  setProbName   ( "SnoptNLP" );
  setPrintFile  ( "SnoptNLP.out" );

  setProblemSize( n, neF );
  setObjective  ( ObjRow, ObjAdd );
  setX          ( x, xlow, xupp, xmul, xstate );
  setF          ( F, Flow, Fupp, Fmul, Fstate );
  setUserFun    (&SnoptAdapter::ObjectiveAndConstraintFct);

  setIntParameter( "Derivative option", 0 ); // let snopt estimate derivatives
  setIntParameter( "Verify level ", 3 );
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
  F[0] = instance_->nlp_->EvaluateCostFunction(x);

  VectorXd g_eig = instance_->nlp_->EvaluateConstraints(x);

//  Eigen::Map<VectorXd>(F+1, g_eig.rows()) = g_eig; // should work as well
  for (int i=0; i<g_eig.rows(); ++i) {
    F[i+1] = g_eig[i];
  }

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

