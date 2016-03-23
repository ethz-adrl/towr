// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: MyNLPHyQ.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef __MYNLP_HPP__
#define __MYNLP_HPP__

#include <IpTNLP.hpp>

#include <xpp/hyq/supp_triangle_container.h>
#include <xpp/zmp/continuous_spline_container.h>

namespace Ipopt {


class NlpIpoptZmp : public Ipopt::TNLP
{

public:
  typedef xpp::zmp::ContinuousSplineContainer Splines;
  typedef xpp::utils::MatVec MatVec;

public:
  /** default constructor */
	NlpIpoptZmp();

  /** default destructor */
  virtual ~NlpIpoptZmp();

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
//  virtual bool eval_h(Index n, const Number* x, bool new_x,
//                      Number obj_factor, Index m, const Number* lambda,
//                      bool new_lambda, Index nele_hess, Index* iRow,
//                      Index* jCol, Number* values);

  //@}


  virtual bool intermediate_callback(AlgorithmMode mode,
                                     Index iter, Number obj_value,
                                     Number inf_pr, Number inf_du,
                                     Number mu, Number d_norm,
                                     Number regularization_size,
                                     Number alpha_du, Number alpha_pr,
                                     Index ls_trials,
                                     const IpoptData* ip_data,
                                     IpoptCalculatedQuantities* ip_cq);


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




  void SetupNlp(const xpp::hyq::SuppTriangleContainer& supp_triangle_container,
                const xpp::zmp::ContinuousSplineContainer& zmp_spline_container,
                const MatVec& qp_cost_function,
                const MatVec& qp_equality_constraints,
                const Eigen::VectorXd& initial_coefficients = Eigen::Vector2d::Zero()
  );

  Eigen::VectorXd x_final_spline_coeff_;
  Eigen::VectorXd x_final_footholds_;

private:
  xpp::hyq::SuppTriangleContainer supp_triangle_container_;
  xpp::zmp::ContinuousSplineContainer zmp_spline_container_;

  MatVec cf_;
  MatVec eq_;
  MatVec ineq_;

  int n_spline_coeff_;
  int n_eq_constr_;
  int n_ineq_constr_;
  int n_steps_;

  MatVec x_zmp_;
  MatVec y_zmp_;

  Eigen::VectorXd initial_coefficients_;
  std::vector<xpp::hyq::Foothold> initial_footholds_;

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
  //  MyNLPHyQ();
  NlpIpoptZmp(const NlpIpoptZmp&);
  NlpIpoptZmp& operator=(const NlpIpoptZmp&);
  //@}

};

} // namespace Ipopt

#endif
