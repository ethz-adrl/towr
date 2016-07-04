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
    instance_ = new SnoptAdapter;
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
  n = 2;   // number of optimization variables
  neF = 3; // number of constraints + 1

  x      = new double[n];
  xlow   = new double[n];
  xupp   = new double[n];
  xmul   = new double[n];
  xstate = new    int[n];

  F      = new double[neF];
  Flow   = new double[neF];
  Fupp   = new double[neF];
  Fmul   = new double[neF];
  Fstate = new int[neF];


  // Set the upper and lower bounds.
  xlow[0]   =  0.0;  xlow[1]   = -1e20;
  xupp[0]   = 1e20;  xupp[1]   =  1e20;
  xstate[0] =    0;  xstate[1] =  0;

  Flow[0] = -1e20; Flow[1] = -1e20; Flow[2] = -1e20;
  Fupp[0] =  1e20; Fupp[1] =   4.0; Fupp[2] =  5.0;
  Fmul[0] =   0;   Fmul[0] =   0;   Fmul[0] =    0;

  // initial values of the optimization
  x[0]    = 1.0;
  x[1]    = 1.0;

  ObjRow  = 0;   // the row in user function that corresponds to the objective function
  ObjAdd  = 0.0; // the constant to be added to the objective function


  setProbName("Toy0");
  setIntParameter( "Derivative option", 0 );
  setIntParameter( "Verify level ", 3 );
  setUserFun(&SnoptAdapter::ObjectiveAndConstraintFct);
}

void
SnoptAdapter::ObjectiveAndConstraintFct (int* Status, int* n, double x[],
                                         int* needF, int* neF, double F[],
                                         int* needG, int* neG, double G[],
                                         char* cu, int* lencu, int iu[],
                                         int* leniu, double ru[], int* lenru)
{
  // want to call member variable "nlp" in here...

  instance_->Init();

//  F[0] = instance_->nlp_->EvaluateCostFunction(x);

  F[0] =  x[1];
  F[1] =  x[0]*x[0] + 4*x[1]*x[1];
  F[2] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
}

Eigen::VectorXd
SnoptAdapter::GetVariables () const
{
  return Eigen::Map<const VectorXd>(x,n);
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
