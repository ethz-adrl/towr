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

#ifndef OPT_SOLVE_INCLUDE_OPT_IPOPT_ADAPTER_H_
#define OPT_SOLVE_INCLUDE_OPT_IPOPT_ADAPTER_H_

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>

#include <opt_solve/problem.h>

namespace opt {

/**
 * @brief Converts an optimization problem into an IPOPT interface.
 *
 * @ingroup solvers
 *
 * Given an optimization Problem with variables, costs and constraints, this
 * class wraps it and makes it conform with the interface defined by IPOPT
 * https://projects.coin-or.org/Ipopt
 *
 * This implements the Adapter pattern. This class should not add any
 * functionality, but merely delegate it to the Adaptee (the Problem object).
 */
class IpoptAdapter : public Ipopt::TNLP {
public:
  using Index    = Ipopt::Index;
  using VectorXd = Problem::VectorXd;
  using Jacobian = Problem::Jacobian;

	/** @brief  Creates an IpoptAdapter and solves the NLP.
	  * @param [in/out]  nlp  The specific problem.
	  *
	  * This function creates the actual solver, sets the solver specific
	  * options (see SetOptions()) and passes the IpoptAdapter problem to it
	  * to be modified.
	  */
	static void Solve(Problem& nlp);

private:
  /**
   * @brief  Defines settings for the Ipopt solver @a app.
   * @param app  The actual Ipopt solver.
   *
   * These settings include which QP solver to use, if gradients should
   * be approximated or the provided analytical ones used, when the iterations
   * should be terminated,...
   *
   * A complete list of options can be found at:
   * https://www.coin-or.org/Ipopt/documentation/node40.html
   */
  static void SetOptions(Ipopt::SmartPtr<Ipopt::IpoptApplication> app);

  Problem* nlp_; ///< The solver independent problem definition

private:
  /**
   * @brief  Creates an IpoptAdapter wrapping the @a nlp.
   * @param  nlp  The specific nonlinear programming problem.
   *
   * This constructor holds and modifies the passed nlp.
   */
	IpoptAdapter(Problem& nlp);
  virtual ~IpoptAdapter();

  /** Method to return some info about the nlp */
  virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style);

  /** Method to return the bounds for my problem */
  virtual bool get_bounds_info(Index n, double* x_l, double* x_u,
                               Index m, double* g_l, double* g_u);

  /** Method to return the starting point for the algorithm */
  virtual bool get_starting_point(Index n, bool init_x, double* x,
                                  bool init_z, double* z_L, double* z_U,
                                  Index m, bool init_lambda,
                                  double* lambda);

  /** Method to return the objective value */
  virtual bool eval_f(Index n, const double* x, bool new_x, double& obj_value);

  /** Method to return the gradient of the objective */
  virtual bool eval_grad_f(Index n, const double* x, bool new_x, double* grad_f);

  /** Method to return the constraint residuals */
  virtual bool eval_g(Index n, const double* x, bool new_x, Index m, double* g);

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
  virtual bool eval_jac_g(Index n, const double* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          double* values);

  /** This is called after every iteration and is used to save intermediate
    *  solutions in the nlp */
  virtual bool intermediate_callback(Ipopt::AlgorithmMode mode,
                                     Index iter, double obj_value,
                                     double inf_pr, double inf_du,
                                     double mu, double d_norm,
                                     double regularization_size,
                                     double alpha_du, double alpha_pr,
                                     Index ls_trials,
                                     const Ipopt::IpoptData* ip_data,
                                     Ipopt::IpoptCalculatedQuantities* ip_cq);

  /** This method is called when the algorithm is complete so the TNLP can
    * store/write the solution */
  virtual void finalize_solution(Ipopt::SolverReturn status,
                                 Index n, const double* x, const double* z_L, const double* z_U,
                                 Index m, const double* g, const double* lambda,
                                 double obj_value,
                                 const Ipopt::IpoptData* ip_data,
                                 Ipopt::IpoptCalculatedQuantities* ip_cq);
};

} // namespace opt

#endif /* OPT_SOLVE_INCLUDE_OPT_IPOPT_ADAPTER_H_ */
