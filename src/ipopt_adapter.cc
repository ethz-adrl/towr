/**
 @file    ipopt_adapter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 10, 2016
 @brief   Defines the IPOPT adapter
 */

#include <xpp/ipopt_adapter.h>

#include <cassert>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/types.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace xpp {
namespace opt {

using VectorXd = Eigen::VectorXd;

IpoptAdapter::IpoptAdapter(NLP& nlp)
{
  nlp_ = &nlp;
}

IpoptAdapter::~IpoptAdapter ()
{
}

bool IpoptAdapter::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                         Index& nnz_h_lag, IndexStyleEnum& index_style)
{
  n = nlp_->GetNumberOfOptimizationVariables();
  m = nlp_->GetNumberOfConstraints();
  nnz_jac_g = nlp_->GetJacobianOfConstraints().nonZeros();

  std::cout << "nnz_jac_g: " << nnz_jac_g << std::endl;

  // nonzeros in the Hessian of the Lagrangian
  // (one in the Hessian of the objective for x2,
  //  and one in the Hessian of the constraints for x1)
  nnz_h_lag = n*n;

  // start index at 0 for row/col entries
  index_style = C_STYLE;

  return true;
}

bool IpoptAdapter::get_bounds_info(Index n, Number* x_lower, Number* x_upper,
                            Index m, Number* g_l, Number* g_u)
{
  auto bounds_x = nlp_->GetBoundsOnOptimizationVariables();
  for (uint c=0; c<bounds_x.size(); ++c) {
    x_lower[c] = bounds_x.at(c).lower_;
    x_upper[c] = bounds_x.at(c).upper_;
  }

  // specific bounds depending on equality and inequality constraints
  auto bounds_g = nlp_->GetBoundsOnConstraints();
  for (uint c=0; c<bounds_g.size(); ++c) {
    g_l[c] = bounds_g.at(c).lower_;
    g_u[c] = bounds_g.at(c).upper_;
  }

  return true;
}

bool IpoptAdapter::get_starting_point(Index n, bool init_x, Number* x,
                               bool init_z, Number* z_L, Number* z_U,
                               Index m, bool init_lambda,
                               Number* lambda)
{
	// Here, we assume we only have starting values for x
	assert(init_x == true);
	assert(init_z == false);
	assert(init_lambda == false);

  VectorXd x_all = nlp_->GetStartingValues();
  Eigen::Map<VectorXd>(&x[0], x_all.rows()) = x_all;

  return true;
}

bool IpoptAdapter::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
  obj_value = nlp_->EvaluateCostFunction(x);
  return true;
}

bool IpoptAdapter::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  Eigen::VectorXd grad = nlp_->EvaluateCostFunctionGradient(x);
  Eigen::Map<Eigen::MatrixXd>(grad_f,n,1) = grad;
	return true;
}

bool IpoptAdapter::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
  VectorXd g_eig = nlp_->EvaluateConstraints(x);
  Eigen::Map<VectorXd>(g,m) = g_eig;
  return true;
}

bool IpoptAdapter::eval_jac_g(Index n, const Number* x, bool new_x,
                       Index m, Index nele_jac, Index* iRow, Index *jCol,
                       Number* values)
{
	// defines the positions of the nonzero elements of the jacobian
  if (values == NULL) {

    auto jac = nlp_->GetJacobianOfConstraints();
    int nele=0; // nonzero cells in jacobian
    for (int k=0; k<jac.outerSize(); ++k) {
      for (Jacobian::InnerIterator it(jac,k); it; ++it) {
        iRow[nele] = it.row();
        jCol[nele] = it.col();
        nele++;
      }
    }

    assert(nele == nele_jac); // initial sparsity structure is never allowed to change
  }
  else {
    // only gets used if "jacobian_approximation finite-difference-values" is not set
    nlp_->EvalNonzerosOfJacobian(x, values);
  }

  return true;
}

bool IpoptAdapter::intermediate_callback(Ipopt::AlgorithmMode mode,
                                   Index iter, Number obj_value,
                                   Number inf_pr, Number inf_du,
                                   Number mu, Number d_norm,
                                   Number regularization_size,
                                   Number alpha_du, Number alpha_pr,
                                   Index ls_trials,
                                   const Ipopt::IpoptData* ip_data,
                                   Ipopt::IpoptCalculatedQuantities* ip_cq)
{
  nlp_->SaveCurrent();
	return true;
}

void IpoptAdapter::finalize_solution(Ipopt::SolverReturn status,
                              Index n, const Number* x, const Number* z_L, const Number* z_U,
                              Index m, const Number* g, const Number* lambda,
                              Number obj_value,
			                        const Ipopt::IpoptData* ip_data,
			                        Ipopt::IpoptCalculatedQuantities* ip_cq)
{

  nlp_->SetVariables(x);
  nlp_->SaveCurrent();
}

void
IpoptAdapter::Solve (NLP& nlp)
{
  using namespace Ipopt;
  using IpoptPtr            = SmartPtr<TNLP>;
  using IpoptApplicationPtr = SmartPtr<IpoptApplication>;

  // initialize the ipopt solver
  IpoptApplicationPtr ipopt_app_ = new IpoptApplication();
  ipopt_app_->RethrowNonIpoptException(true); // this allows to see the error message of exceptions thrown inside ipopt
  SetOptions(ipopt_app_);

  ApplicationReturnStatus status_ = ipopt_app_->Initialize();
  if (status_ != Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }

  // Convert the NLP problem to Ipopt
  IpoptPtr nlp_ptr = new IpoptAdapter(nlp);
  status_ = ipopt_app_->OptimizeTNLP(nlp_ptr);

  if (status_ != Solve_Succeeded) {
    std::string msg = "Ipopt failed to find a solution. ReturnCode: " + std::to_string(status_);
    std::cerr << msg;
  }
}

void
IpoptAdapter::SetOptions (Ipopt::SmartPtr<Ipopt::IpoptApplication> ipopt_app_)
{
  // A complete list of options can be found here
  // https://www.coin-or.org/Ipopt/documentation/node40.html

  // Set some options
  ipopt_app_->Options()->SetStringValue("linear_solver", "ma57"); // 27, 57, 77, 86, 97
//  ipopt_app_->Options()->SetStringValue("jacobian_approximation", "finite-difference-values");
  ipopt_app_->Options()->SetStringValue("hessian_approximation", "limited-memory");
  ipopt_app_->Options()->SetNumericValue("derivative_test_tol", 1e-3);
//  ipopt_app_->Options()->SetStringValue("derivative_test", "first-order");
//  ipopt_app_->Options()->SetStringValue("derivative_test", "second-order");

  ipopt_app_->Options()->SetNumericValue("tol", 0.001);
//  ipopt_app_->Options()->SetNumericValue("constr_viol_tol", 1e-3);
//  ipopt_app_->Options()->SetNumericValue("dual_inf_tol", 1e10);
//  ipopt_app_->Options()->SetNumericValue("compl_inf_tol", 1e10);

  ipopt_app_->Options()->SetNumericValue("max_cpu_time", 10.0);
//  ipopt_app_->Options()->SetIntegerValue("max_iter", 0);
//  ipopt_app_->Options()->SetNumericValue("bound_relax_factor", 0.01);
//  ipopt_app_->Options()->SetNumericValue("bound_frac", 0.5);

  ipopt_app_->Options()->SetIntegerValue("print_level", 5);
  ipopt_app_->Options()->SetStringValue("print_user_options", "yes");
  ipopt_app_->Options()->SetStringValue("print_timing_statistics", "no");
//  ipopt_app_->Options()->SetStringValue("output_file", "ipopt.out");

//  ipopt_app_->Options()->SetNumericValue("obj_scaling_factor", 10);
//  ipopt_app_->Options()->SetStringValue("nlp_scaling_method", "none"); // "equilibration-based", "gradient-based"
}

} // namespace opt
} // namespace xpp
