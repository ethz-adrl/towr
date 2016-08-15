// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: MyNLP.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef __MYNLP_HPP__
#define __MYNLP_HPP__

#include "IpTNLP.hpp"
#include <cassert>

using namespace Ipopt;

/** C++ Example NLP for interfacing a problem with IPOPT.
 *  MyNLP implements a C++ example showing how to interface with IPOPT
 *  through the TNLP interface. This example is designed to go along with
 *  the tutorial document (see Examples/CppTutorial/).
 *  This class implements the following NLP.
 *
 * min_x f(x) = -(x2-2)^2
 *  s.t.
 *       0 = -(x1^2 + x2 - 1)
 *       -1 <= x1 <= 1
 *
 */
class MyNLP : public TNLP
{
public:
  /** default constructor */
  MyNLP();

  /** default destructor */
  virtual ~MyNLP();

  /**@name Overloaded from TNLP */
  //@{
  /** Method to return some info about the nlp */
  virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style);

  /** Method to return the bounds for my problem */
  virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                               Index m, Number* g_l, Number* g_u);

  /** Method to return the starting point for the algorithm */
  virtual bool get_starting_point(Index n, bool init_x, Number* x,
                                  bool init_z, Number* z_L, Number* z_U,
                                  Index m, bool init_lambda,
                                  Number* lambda);

  /** Method to return the objective value */
  virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value);

  /** Method to return the gradient of the objective */
  virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f);

  /** Method to return the constraint residuals */
  virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g);

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
  virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          Number* values);

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
  virtual bool eval_h(Index n, const Number* x, bool new_x,
                      Number obj_factor, Index m, const Number* lambda,
                      bool new_lambda, Index nele_hess, Index* iRow,
                      Index* jCol, Number* values);

  //@}

  /** @name Solution Methods */
  //@{
  /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
  virtual void finalize_solution(SolverReturn status,
                                 Index n, const Number* x, const Number* z_L, const Number* z_U,
                                 Index m, const Number* g, const Number* lambda,
                                 Number obj_value,
				 const IpoptData* ip_data,
				 IpoptCalculatedQuantities* ip_cq);
  //@}

private:
  /**@name Methods to block default compiler methods.
   * The compiler automatically generates the following three methods.
   *  Since the default compiler implementation is generally not what
   *  you want (for all but the most simple classes), we usually 
   *  put the declarations of these methods in the private section
   *  and never implement them. This prevents the compiler from
   *  implementing an incorrect "default" behavior without us
   *  knowing. (See Scott Meyers book, "Effective C++")
   *  
   */
  //@{
  //  MyNLP();
  MyNLP(const MyNLP&);
  MyNLP& operator=(const MyNLP&);
  //@}
};


// this was originally in the cpp file
MyNLP::MyNLP()
{}

MyNLP::~MyNLP()
{}

bool MyNLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                         Index& nnz_h_lag, IndexStyleEnum& index_style)
{
  // The problem described in MyNLP.hpp has 2 variables, x1, & x2,
  n = 2;

  // one equality constraint,
  m = 1;

  // 2 nonzeros in the jacobian (one for x1, and one for x2),
  nnz_jac_g = 2;

  // and 2 nonzeros in the hessian of the lagrangian
  // (one in the hessian of the objective for x2,
  //  and one in the hessian of the constraints for x1)
  nnz_h_lag = 2;

  // We use the standard fortran index style for row/col entries
  index_style = FORTRAN_STYLE;

  return true;
}

bool MyNLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
                            Index m, Number* g_l, Number* g_u)
{
  // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
  // If desired, we could assert to make sure they are what we think they are.
  assert(n == 2);
  assert(m == 1);

  // x1 has a lower bound of -1 and an upper bound of 1
  x_l[0] = -1.0;
  x_u[0] = 1.0;

  // x2 has no upper or lower bound, so we set them to
  // a large negative and a large positive number.
  // The value that is interpretted as -/+infinity can be
  // set in the options, but it defaults to -/+1e19
  x_l[1] = -1.0e19;
  x_u[1] = +1.0e19;

  // we have one equality constraint, so we set the bounds on this constraint
  // to be equal (and zero).
  g_l[0] = g_u[0] = 0.0;

  return true;
}

bool MyNLP::get_starting_point(Index n, bool init_x, Number* x,
                               bool init_z, Number* z_L, Number* z_U,
                               Index m, bool init_lambda,
                               Number* lambda)
{
  // Here, we assume we only have starting values for x, if you code
  // your own NLP, you can provide starting values for the others if
  // you wish.
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);

  // we initialize x in bounds, in the upper right quadrant
  x[0] = 0.5;
  x[1] = 1.5;

  return true;
}

bool MyNLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
  // return the value of the objective function
  Number x2 = x[1];
  obj_value = -(x2 - 2.0) * (x2 - 2.0);
  return true;
}

bool MyNLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  // return the gradient of the objective function grad_{x} f(x)

  // grad_{x1} f(x): x1 is not in the objective
  grad_f[0] = 0.0;

  // grad_{x2} f(x):
  Number x2 = x[1];
  grad_f[1] = -2.0*(x2 - 2.0);

  return true;
}

bool MyNLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
  // return the value of the constraints: g(x)
  Number x1 = x[0];
  Number x2 = x[1];

  g[0] = -(x1*x1 + x2 - 1.0);

  return true;
}

bool MyNLP::eval_jac_g(Index n, const Number* x, bool new_x,
                       Index m, Index nele_jac, Index* iRow, Index *jCol,
                       Number* values)
{
  if (values == NULL) {
    // return the structure of the jacobian of the constraints - i.e. specify positions of non-zero elements.

    // element at 1,1: grad_{x1} g_{1}(x)
    iRow[0] = 1;
    jCol[0] = 1;

    // element at 1,2: grad_{x2} g_{1}(x)
    iRow[1] = 1;
    jCol[1] = 2;
  }
  else {
    // return the values of the jacobian of the constraints
    Number x1 = x[0];

    // element at 1,1: grad_{x1} g_{1}(x)
    values[0] = -2.0 * x1;

    // element at 1,2: grad_{x1} g_{1}(x)
    values[1] = -1.0;
  }

  return true;
}

bool MyNLP::eval_h(Index n, const Number* x, bool new_x,
                   Number obj_factor, Index m, const Number* lambda,
                   bool new_lambda, Index nele_hess, Index* iRow,
                   Index* jCol, Number* values)
{
  if (values == NULL) {
    // return the structure. This is a symmetric matrix, fill the lower left
    // triangle only.

    // element at 1,1: grad^2_{x1,x1} L(x,lambda)
    iRow[0] = 1;
    jCol[0] = 1;

    // element at 2,2: grad^2_{x2,x2} L(x,lambda)
    iRow[1] = 2;
    jCol[1] = 2;

    // Note: off-diagonal elements are zero for this problem
  }
  else {
    // return the values

    // element at 1,1: grad^2_{x1,x1} L(x,lambda)
    values[0] = -2.0 * lambda[0];

    // element at 2,2: grad^2_{x2,x2} L(x,lambda)
    values[1] = -2.0 * obj_factor;

    // Note: off-diagonal elements are zero for this problem
  }

  return true;
}

void MyNLP::finalize_solution(SolverReturn status,
                              Index n, const Number* x, const Number* z_L, const Number* z_U,
                              Index m, const Number* g, const Number* lambda,
                              Number obj_value,
            const IpoptData* ip_data,
            IpoptCalculatedQuantities* ip_cq)
{
  // here is where we would store the solution to variables, or write to a file, etc
  // so we could use the solution. Since the solution is displayed to the console,
  // we currently do nothing here.
}



#endif
