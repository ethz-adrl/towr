/**
 @file    snopt_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 4, 2016
 @brief   Some unit test for testing the Snopt installation.

 These files test the basic snopt installation on your system and document
 its basic usage. For more information, please checkout the user guide:
 http://web.stanford.edu/group/SOL/guides/sndoc7.pdf

 A few insights:
 SnoptProblemA ignores constants in purely linear constraints. To use those,
 the constant term must be put in the bound values.
 */


#include <gtest/gtest.h>

#include <stdio.h>
#include <string.h>
#include <iostream>

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
  xlow[0] =   0.0; xupp[0] = 1e20; xstate[0] =  0;
  xlow[1] = -1e20; xupp[1] = 1e20; xstate[1] =  0;

  Flow[0] = -1e20; Fupp[0] =  1e20; Fmul[0] =   0;
  Flow[1] = -1e20; Fupp[1] =   4.0; Fmul[1] =   0;
  Flow[2] = -1e20; Fupp[2] =   5.0; Fmul[2] =   0;

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


using namespace std;

void toyobjB ( int *mode,  int *nnObj, double x[],
         double *fObj,  double gObj[], int *nState,
         char    *cu, int *lencu,
         int    iu[], int *leniu,
         double ru[], int *lenru )
{
  //==================================================================
  // Computes the nonlinear objective and constraint terms for the toy
  // problem featured in the SnoptA users guide.
  // m = 3, n = 2.
  //
  //   Minimize     x(2)
  //
  //   subject to   x(1)**2      + 4 x(2)**2  <= 4,
  //               (x(1) - 2)**2 +   x(2)**2  <= 5,
  //                x(1) >= 0.
  //
  //==================================================================

  if ( *mode == 0 || *mode == 2 ) {
    *fObj   =  0;
  }

  if ( *mode == 1 || *mode == 2 ) {
    // gObj not set; no nonlinear variables in objective
  }

}

void toyconB ( int *mode,  int *nnCon, int *nnJac, int *negCon,
         double x[], double fCon[], double gCon[], int *nState,
         char    *cu, int *lencu,
         int    iu[], int *leniu,
         double ru[], int *lenru )
{
  //==================================================================
  // Computes the nonlinear objective and constraint terms for the toy
  // problem featured in the SnoptA users guide.
  // m = 3, n = 2.
  //
  //   Minimize     x(2)
  //
  //   subject to   x(1)**2      + 4 x(2)**2  <= 4,
  //               (x(1) - 2)**2 +   x(2)**2  <= 5,
  //                x(1) >= 0.
  //
  //==================================================================

  if ( *mode == 0 || *mode == 2 ) {
    fCon[0] =  x[0]*x[0] + 4*x[1]*x[1];
    fCon[1] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
  }

  if ( *mode == 1 || *mode == 2 ) {
    gCon[0] = 2*x[0];
    gCon[1] = 2*(x[0] - 2);
    gCon[2] = 8*x[1];
    gCon[3] = 2*x[1];
  }

}

TEST(SnoptTest, sntoyB)
{
  snoptProblemB ToyProb("ToyB");

  int n     =  2;
  int m     =  3;
  int ne    =  5;
  int nnCon =  2;
  int nnObj =  0;
  int nnJac =  2;

  int    *indJ = new int[ne];
  int    *locJ = new int[n+1];
  double *valJ = new double[ne];

  double *x  = new double[n+m];
  double *bl = new double[n+m];
  double *bu = new double[n+m];
  double *pi = new double[m];
  double *rc = new double[n+m];
  int    *hs = new    int[n+m];

  int    iObj    = 2;
  double ObjAdd  = 0;

  int Cold = 0, Basis = 1, Warm = 2;


  // Set the upper and lower bounds.
  bl[0] =     0;  bu[0] = 1e20;
  bl[1] = -1e20;  bu[1] = 1e20;
  bl[2] = -1e20;  bu[2] =    4;
  bl[3] = -1e20;  bu[3] =    5;
  bl[4] = -1e20;  bu[4] = 1e20;

  // Initialize states, x and multipliers
  for ( int i = 0; i < n+m; i++ ) {
    hs[i] = 0;
     x[i] = 0;
    rc[i] = 0;
  }

  for ( int i = 0; i < m; i++ ) {
    pi[i] = 0;
  }

  x[0]    = 1.0;
  x[1]    = 1.0;

  // Set up the Jacobian matrix
  // Column 1
  locJ[0] = 0;

  indJ[0] = 0;
  valJ[0] = 0;

  indJ[1] = 1;
  valJ[1] = 0;

  // Column 2
  locJ[1] = 2;

  indJ[2] = 0;
  valJ[2] = 0;

  indJ[3] = 1;
  valJ[3] = 0;

  indJ[4] = 2;
  valJ[4] = 1;

  locJ[2] = 5;

  ToyProb.setProblemSize ( m, n, nnCon, nnJac, nnObj );
  ToyProb.setObjective   ( iObj, ObjAdd );
  ToyProb.setJ           ( ne, valJ, indJ, locJ );
  ToyProb.setX           ( bl, bu, x, pi, rc, hs );

  ToyProb.setFunobj      ( toyobjB );
  ToyProb.setFuncon      ( toyconB );

  ToyProb.setIntParameter( "Verify level", 3 );
  ToyProb.setIntParameter( "Derivative option", 3 );

  ToyProb.solve          ( Cold );

  for (int i = 0; i < n; i++ ){
    cout << "x = " << x[i] << endl;
  }

  EXPECT_NEAR(0,x[0], 1e-5);
  EXPECT_NEAR(-1,x[1],1e-5);

  delete []indJ;  delete []locJ; delete []valJ;

  delete []x;     delete []bl;   delete []bu;
  delete []pi;    delete []rc;   delete []hs;
}

