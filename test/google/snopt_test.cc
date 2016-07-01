#include <gtest/gtest.h>

#include <stdio.h>
#include <string.h>
#include <iostream>

extern "C" {
void toyusrf_ ( int    *Status, int *n,    double x[],
                int    *needF,  int *neF,  double F[],
                int    *needG,  int *neG,  double G[],
                char   *cu,     int *lencu,
                int    iu[],    int *leniu,
                double ru[],    int *lenru );

void toyusrfg_( int    *Status, int *n,    double x[],
                int    *needF,  int *neF,  double F[],
                int    *needG,  int *neG,  double G[],
                char   *cu,     int *lencu,
                int    iu[],    int *leniu,
                double ru[],    int *lenru);
}

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
  //   Minimize     x(2)
  //
  //   subject to   x(1)**2      + 4 x(2)**2  <= 4,
  //               (x(1) - 2)**2 +   x(2)**2  <= 5,
  //                x(1) >= 0.
  //
  //==================================================================
  F[0] =  x[1];
  F[1] =  x[0]*x[0] + 4*x[1]*x[1];
  F[2] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
}

void toyusrfg_( int    *Status, int *n,    double x[],
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
  //   Minimize     x(2)
  //
  //   subject to   x(1)**2      + 4 x(2)**2  <= 4,
  //               (x(1) - 2)**2 +   x(2)**2  <= 5,
  //                x(1) >= 0.
  //
  // The triples (g(k),iGfun(k),jGvar(k)), k = 1:neG, define
  // the sparsity pattern and values of the nonlinear elements
  // of the Jacobian.
  //==================================================================

  if ( *needF > 0 ) {
    F[0] =  x[1]; //  Objective row
    F[1] =  x[0]*x[0] + 4*x[1]*x[1];
    F[2] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
  }


  if ( *needG > 0 ) {
    // iGfun[0] = 1
    // jGvar[0] = 0
    G[0] = 2*x[0];

    // iGfun[1] = 1
    // jGvar[1] = 1
    G[1] = 8*x[1];

    // iGfun[2] = 2
    // jGvar[2] = 0
    G[2] = 2*(x[0] - 2);

    // iGfun[3] = 2
    // jGvar[3] = 1
    G[3] = 2*x[1];
  }
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
  Fmul[0] =   0;   Fmul[0] =   0;   Fmul[0] =    0;
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


  printf("\nSolving toy1 problem using derivatives...\n");

  // Reset the variables and solve ...

  int lenA   = 6;
  int *iAfun = new int[lenA];
  int *jAvar = new int[lenA];
  double *A  = new double[lenA];

  int lenG   = 6;
  int *iGfun = new int[lenG];
  int *jGvar = new int[lenG];

  int neA, neG; // neA and neG must be defined when providing dervatives

  xstate[0] =   0;  xstate[1] = 0;
  Fmul[0]   =   0;  Fmul[0]   = 0; Fmul[0] =    0;
  x[0]      = 1.0;
  x[1]      = 1.0;


  // Provide the elements of the Jacobian explicitly.
  iGfun[0] = 1;
  jGvar[0] = 0;

  iGfun[1] = 1;
  jGvar[1] = 1;

  iGfun[2] = 2;
  jGvar[2] = 0;

  iGfun[3] = 2;
  jGvar[3] = 1;
  neG      = 4;

  iAfun[0] = 0;
  jAvar[0] = 1;
  A[0]     = 1.0;
  neA      = 1;

  ToyProb.setProbName    ( "Toy1" );         // Give the problem a new name for Snopt.

  // neA and neG must be set here
  ToyProb.setA           ( lenA, neA, iAfun, jAvar, A );
  ToyProb.setG           ( lenG, neG, iGfun, jGvar );

  ToyProb.setUserFun     ( toyusrfg_ );      // Sets the usrfun that supplies G and F.

  ToyProb.setPrintFile   ( "Toy1.out" );
  ToyProb.setSpecsFile   ( "sntoya.spc" );
  ToyProb.setIntParameter( "Derivative option", 1 );
  ToyProb.setIntParameter( "Major Iteration limit", 250 );
  ToyProb.setIntParameter( "Verify level ", 3 );
  ToyProb.solve          ( Cold );

  for (int i = 0; i < n; i++ ){
    cout << "x = " << x[i] << " xstate = " << xstate[i] << endl;
  }
  for (int i = 0; i < neF; i++ ){
    cout << "F = " << F[i] << " Fstate = " << Fstate[i] << endl;
  }

  ToyProb.solve          ( Warm );

  for (int i = 0; i < n; i++ ){
    cout << "x = " << x[i] << " xstate = " << xstate[i] << endl;
  }
  for (int i = 0; i < neF; i++ ){
    cout << "F = " << F[i] << " Fstate = " << Fstate[i] << endl;
  }


  delete []iAfun;  delete []jAvar;  delete []A;
  delete []iGfun;  delete []jGvar;

  delete []x;      delete []xlow;   delete []xupp;
  delete []xmul;   delete []xstate;

  delete []F;      delete []Flow;   delete []Fupp;
  delete []Fmul;   delete []Fstate;

}
