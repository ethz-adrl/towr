/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/**
 @file    ipopt_adapter.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 10, 2016
 @brief   Defines the Adapter class to interact with the Ipopt Library.
 */

#ifndef OPT_SOLVE_INCLUDE_OPT_IPOPT_ADAPTER_H_
#define OPT_SOLVE_INCLUDE_OPT_IPOPT_ADAPTER_H_

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>

#include <opt_solve/nlp.h>

namespace opt {

/** @brief Converts the NLP defined in the XPP to the IPOPT interface.
  *
  * https://projects.coin-or.org/Ipopt
  *
  * This implements the Adapter pattern. This class should not add any functionality,
  * but merely delegate it to the Adaptee (the nlp class).
  */
class IpoptAdapter : public Ipopt::TNLP {
public:
  using Index    = Ipopt::Index;
  using Number   = Ipopt::Number;
  using VectorXd = NLP::VectorXd;
  using Jacobian = NLP::Jacobian;

	/** @brief Builds an Ipopt NLP and solves it.
	  * @param[in/out] nlp the nonlinear program to be modified.
	  */
	static void Solve(NLP& nlp);


private:
	IpoptAdapter(NLP& nlp);
  virtual ~IpoptAdapter();

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



  virtual bool intermediate_callback(Ipopt::AlgorithmMode mode,
                                     Index iter, Number obj_value,
                                     Number inf_pr, Number inf_du,
                                     Number mu, Number d_norm,
                                     Number regularization_size,
                                     Number alpha_du, Number alpha_pr,
                                     Index ls_trials,
                                     const Ipopt::IpoptData* ip_data,
                                     Ipopt::IpoptCalculatedQuantities* ip_cq);

  /** This method is called when the algorithm is complete so the TNLP can
    * store/write the solution */
  virtual void finalize_solution(Ipopt::SolverReturn status,
                                 Index n, const Number* x, const Number* z_L, const Number* z_U,
                                 Index m, const Number* g, const Number* lambda,
                                 Number obj_value,
                                 const Ipopt::IpoptData* ip_data,
                                 Ipopt::IpoptCalculatedQuantities* ip_cq);

private:
  NLP* nlp_;
  static void SetOptions(Ipopt::SmartPtr<Ipopt::IpoptApplication> app);
};

} // namespace opt

#endif /* OPT_SOLVE_INCLUDE_OPT_IPOPT_ADAPTER_H_ */
