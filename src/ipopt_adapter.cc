/**
 @file    nlp_ipopt_zmp.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 10, 2016
 @brief   Defines the actual IPOPT solver
 */

#include <xpp/zmp/ipopt_adapter.h>

// only to get the optimization variables in the intermediate callback
//#include "IpIpoptCalculatedQuantities.hpp"
//#include "IpIpoptData.hpp"
//#include "IpTNLPAdapter.hpp"
//#include "IpOrigIpoptNLP.hpp"

namespace Ipopt {


IpoptAdapter::IpoptAdapter(OptimizationVariables& opt_variables,
                         CostContainer& cost_container,
                         ConstraintContainer& constraint_container,
                         IVisualizer& visualizer)
    :opt_variables_(opt_variables),
     cost_container_(cost_container),
     constraint_container_(constraint_container),
     // These epsilons play a big role in convergence
     cf_num_diff_functor_(1*std::numeric_limits<double>::epsilon()),
     visualizer_(visualizer)
{
  cf_num_diff_functor_.AddCosts(opt_variables_, cost_container_);
}


bool IpoptAdapter::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                         Index& nnz_h_lag, IndexStyleEnum& index_style)
{
  // How many variables to optimize over
  n = opt_variables_.GetOptimizationVariableCount();
  std::cout << "optimizing n= " << n << " variables\n";

  m = constraint_container_.GetBounds().size();
  std::cout << "with m= " << m << "constraints\n";


  // nonzeros in the jacobian of the constraint g(x)
  nnz_jac_g = m * n; // fixme all constraints depend on all inputs

  // nonzeros in the hessian of the lagrangian
  // (one in the hessian of the objective for x2,
  //  and one in the hessian of the constraints for x1)
  nnz_h_lag = n*n;

  // start index at 0 for row/col entries
  index_style = C_STYLE;

  return true;
}

bool IpoptAdapter::get_bounds_info(Index n, Number* x_lower, Number* x_upper,
                            Index m, Number* g_l, Number* g_u)
{

  // no bounds on the spline coefficients of footholds
  for (int i=0; i<n; ++i) {
    x_lower[i] = -1.0e19;
    x_upper[i] = +1.0e19;
  }

  // specific bounds depending on equality and inequality constraints
  std::vector<xpp::zmp::AConstraint::Bound> bounds = constraint_container_.GetBounds();
  for (uint c=0; c<bounds.size(); ++c) {
    g_l[c] = bounds.at(c).lower_;
    g_u[c] = bounds.at(c).upper_;
  }

  return true;
}

bool IpoptAdapter::get_starting_point(Index n, bool init_x, Number* x,
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

  int c = 0;

  VectorXd x_spline_coeff_init = opt_variables_.GetSplineCoefficients();
	Eigen::Map<VectorXd>(&x[c], x_spline_coeff_init.rows()) = x_spline_coeff_init;
	c += x_spline_coeff_init.rows();

	VectorXd x_footholds_init = opt_variables_.GetFootholdsEig();
	Eigen::Map<VectorXd>(&x[c], x_footholds_init.rows()) = x_footholds_init;
	c += x_footholds_init.rows();

	assert(c == n);
  return true;
}


bool IpoptAdapter::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
  opt_variables_.SetVariables(x);
  obj_value = cost_container_.EvaluateTotalCost();
  return true;
}


bool IpoptAdapter::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  Eigen::MatrixXd jac(1,n);
  VectorXd x_eig = Eigen::Map<const VectorXd>(x,n);
  cf_num_diff_functor_.df(x_eig, jac);
  Eigen::Map<Eigen::MatrixXd>(grad_f,1,n) = jac;
	return true;
}


bool IpoptAdapter::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
//  VectorXd g_eig = constraints_.EvalContraints(nlp_structure_.ConvertToEigen(x));
  opt_variables_.SetVariables(x);
  VectorXd g_eig = constraint_container_.EvaluateConstraints();
  Eigen::Map<VectorXd>(g,m) = g_eig;
  return true;
}


bool IpoptAdapter::eval_jac_g(Index n, const Number* x, bool new_x,
                       Index m, Index nele_jac, Index* iRow, Index *jCol,
                       Number* values)
{
  assert(nele_jac == n*m); // number of elements in jacobian

  // FIXME exploit sparsity structure more.
	// say at which positions the nonzero elements of the jacobian are
  if (values == NULL) {
    // return the structure of the jacobian of the constraints - i.e. specify positions of non-zero elements.
  	int c_nonzero = 0;
    for (int row=0; row<m; ++row) {
      for (int col=0; col<n; ++col) {
  			iRow[c_nonzero] = row;
  			jCol[c_nonzero] = col;
  			c_nonzero++;
  		}
  	}
  }
  else {
//    // only gets used if "jacobian_approximation finite-difference-values"
//    // is not set
//    Eigen::MatrixXd jac(m,n);
//    num_diff_constraints_.df(nlp_structure_.ConvertToEigen(x),jac);
//    Eigen::Map<Eigen::MatrixXd>(values,jac.rows(),jac.cols()) = jac;
  }

  return true;
}



bool IpoptAdapter::intermediate_callback(AlgorithmMode mode,
                                   Index iter, Number obj_value,
                                   Number inf_pr, Number inf_du,
                                   Number mu, Number d_norm,
                                   Number regularization_size,
                                   Number alpha_du, Number alpha_pr,
                                   Index ls_trials,
                                   const IpoptData* ip_data,
                                   IpoptCalculatedQuantities* ip_cq)
{
//   std::cout << "Press Enter to continue...";
//   std::cin.get(); // use to pause after every iteration

//  // Get the current value of the optimization variable:
//  Ipopt::TNLPAdapter* tnlp_adapter = NULL;
//  if( ip_cq != NULL )
//  {
//    Ipopt::OrigIpoptNLP* orignlp;
//    orignlp = dynamic_cast<OrigIpoptNLP*>(GetRawPtr(ip_cq->GetIpoptNLP()));
//    if( orignlp != NULL ) {
//      tnlp_adapter = dynamic_cast<TNLPAdapter*>(GetRawPtr(orignlp->nlp()));
//      double* x = new double[opt_variables_.GetOptimizationVariableCount()];
//      tnlp_adapter->ResortX(*ip_data->curr()->x(), x);
//
//      opt_variables_.SetVariables(x);

  visualizer_.PublishMsg();
	return true;
}



void IpoptAdapter::finalize_solution(SolverReturn status,
                              Index n, const Number* x, const Number* z_L, const Number* z_U,
                              Index m, const Number* g, const Number* lambda,
                              Number obj_value,
			                        const IpoptData* ip_data,
			                        IpoptCalculatedQuantities* ip_cq)
{

  opt_variables_.SetVariables(x);
//  opt_variables_.spline_coeff_ = nlp_structure_.ExtractSplineCoefficients(x);
//  opt_variables_.footholds_ = nlp_structure_.ExtractFootholds(x);

  //  // write data to xml file
  //	Eigen::MatrixXd opt_u(1,n);
  //	for (int i=0; i<n; ++i)
  //		opt_u(i) = u[i];
  //
  //	Eigen::MatrixXd opt_x(integrator_.ode_states_.size(), y_start_.rows());
  //	for (int i=0; i<integrator_.ode_states_.size(); ++i)
  //		opt_x.row(i) = integrator_.ode_states_.at(i);
  //
  //  std::ofstream os("optimized_torques.xml");
  //  cereal::XMLOutputArchive xmlarchive(os);
  //  xmlarchive(cereal::make_nvp("U", opt_u), cereal::make_nvp("X", opt_x));
}



} // namespace Ipopt


