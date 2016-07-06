#include <gtest/gtest.h>

#include <stdio.h>
#include <string.h>
#include <iostream>

extern "C" void toyusrf_ ( int    *Status, int *n,    double x[],
                int    *needF,  int *neF,  double F[],
                int    *needG,  int *neG,  double G[],
                char   *cu,     int *lencu,
                int    iu[],    int *leniu,
                double ru[],    int *lenru );

// sntoya.cpp
#include "snoptProblem.hpp"
using namespace std;

void toyusrf_(int    *Status, int *n,    double x[],
	      int    *needF,  int *neF,  double F[],
	      int    *needG,  int *neG,  double G[],
	      char      *cu,  int *lencu,
	      int    iu[],    int *leniu,
	      double ru[],    int *lenru )
{
  //==================================================================
  // Computes the nonlinear objective and constraint terms for the toy
  // problem featured in the SnoptA users guide.
  // neF = 3, n = 2.
  //
  //   Minimize     x1
  //
  //   subject to   x0^2       + 4*x1^2  <= 4,
  //               (x0 - 2)^2  +   x1^2  <= 5,
  //                x0 >= 0.
  //
  //==================================================================
  // Remember: Cannot include constant terms in here if the constraints
  // are linear in the variables. Snopt will just ignore them. The
  // constants MUST be put into the bounds if the constraint is linear.
  F[0] =  x[1];
  F[1] =  x[0]*x[0] + 4*x[1]*x[1];
  F[2] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
}

TEST(SnoptTest, sntoyA)
{

  snoptProblemA ToyProb;

  // Allocate and initialize;
  int n     =  2;
  int neF   =  3;

  double *x      = new double[n];
  double *xlow   = new double[n];
  double *xupp   = new double[n];
  double *xmul   = new double[n];
  int    *xstate = new    int[n];

  double *F      = new double[neF];
  double *Flow   = new double[neF];
  double *Fupp   = new double[neF];
  double *Fmul   = new double[neF];
  int    *Fstate = new int[neF];

  int    ObjRow  = 0;
  double ObjAdd  = 0;

  int Cold = 0, Basis = 1, Warm = 2;


  // Set the upper and lower bounds.
  xlow[0]   =  0.0;  xlow[1]   = -1e20;
  xupp[0]   = 1e20;  xupp[1]   =  1e20;
  xstate[0] =    0;  xstate[1] =  0;

  Flow[0] = -1e20; Flow[1] = -1e20; Flow[2] = -1e20;
  Fupp[0] =  1e20; Fupp[1] =   4.0; Fupp[2] =  5.0;
  Fmul[0] =   0;   Fmul[1] =   0;   Fmul[2] =    0;

  // initial values of the optimization
  x[0]    = 1.0;
  x[1]    = 1.0;


  // Load the data for ToyProb ...
  ToyProb.setProbName   ("Toy0");
  ToyProb.setPrintFile  ( "Toy0.out" );

  ToyProb.setProblemSize( n, neF );
  ToyProb.setObjective  ( ObjRow, ObjAdd );
  ToyProb.setX          ( x, xlow, xupp, xmul, xstate );
  ToyProb.setF          ( F, Flow, Fupp, Fmul, Fstate );

  ToyProb.setUserFun    ( toyusrf_ );


  // snopta will compute the Jacobian by finite-differences.
  // The user has the option of calling  snJac  to define the
  // coordinate arrays (iAfun,jAvar,A) and (iGfun, jGvar).
  ToyProb.setIntParameter( "Derivative option", 0 );
  ToyProb.setIntParameter( "Verify level ", 3 );

  // Solve the problem.
  // snJac is called implicitly in this case to compute the Jacobian.
  ToyProb.solve( Cold );

  for (int i = 0; i < n; i++ ){
    cout << "x = " << x[i] << " xstate = " << xstate[i] << endl;
  }
  for (int i = 0; i < neF; i++ ){
    cout << "F = " << F[i] << " Fstate = " << Fstate[i] << endl;
  }

  EXPECT_NEAR(0,x[0], 1e-5);
  EXPECT_NEAR(-1,x[1],1e-5);

  delete []x;      delete []xlow;   delete []xupp;
  delete []xmul;   delete []xstate;

  delete []F;      delete []Flow;   delete []Fupp;
  delete []Fmul;   delete []Fstate;
}
