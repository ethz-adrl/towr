// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: MyNLPHyQ.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef __MYNLP_HPP__
#define __MYNLP_HPP__

#include "IpTNLP.hpp"

namespace Ipopt {


class NlpIpoptZmp : public Ipopt::TNLP
{

public:
//	typedef Eigen::Matrix< double , 2*(iit::HyQ::jointsCount + 6), 1 > ODEState;
//	typedef Eigen::Matrix< double ,   iit::HyQ::jointsCount , 1 > ODEInput;
//	typedef std::vector<ODEInput> ODEInputVec;

//	typedef boost::numeric::odeint::euler
//	  <
//		  ODEState,
//	    double,
//			ODEState,
//	    double,
//	    boost::numeric::odeint::vector_space_algebra
//	  > EulerStepper;
//
//
//	typedef boost::numeric::odeint::runge_kutta4
//	  <
//		  ODEState,
//		  double,
//			ODEState,
//			double,
//		  boost::numeric::odeint::vector_space_algebra
//	  > RungeKuttaStepper;

	static constexpr int kInputNodesCount = 10;
	static constexpr double kTmaxStart = 0.4; //s
	static constexpr double kTIntegrationStep = 0.01; //s
	static const int kNumEE = 4;


	int count_;
	int count_prev_;

public:
  /** default constructor */
	NlpIpoptZmp(bool init_from_file);

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
//  virtual bool eval_h1(Index n, const Number* x, bool new_x,
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
  //  MyNLPHyQ();
  NlpIpoptZmp(const NlpIpoptZmp&);
  NlpIpoptZmp& operator=(const NlpIpoptZmp&);
  //@}


//  // some stuff used for integration
//  iit::HyQ::dyn::InertiaProperties ip_;
//  iit::HyQ::MotionTransforms mt_;
//  iit::HyQ::dyn::InverseDynamics id_;
//
//  std::array<ODEInput, kInputNodesCount> ode_inputs_;
//
////  EulerStepper stepper_;
//
//  ODEState y_start_;
//  ODEState y_final_des_;
//  ODEState y_inter_des_;
//
//  bool init_from_file_;
//
//
//
//
//  void integrateODE(const Number* x, double T, ODEState& y_final);
//
//
////  ff::HyQKinematics hyq_kinematics_;
////  shot::HyQSystemDynamics hyq_system_dyn_;



};

} // namespace Ipopt

#endif
