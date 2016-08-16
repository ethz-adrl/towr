/**
 @file    nlp_ipopt_zmp.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 10, 2016
 @brief   Defines the IPOPT adapter
 */

#include <xpp/zmp/ipopt_adapter.h>

// only to get the optimization variables in the intermediate callback
//#include "IpIpoptCalculatedQuantities.hpp"
//#include "IpIpoptData.hpp"
//#include "IpTNLPAdapter.hpp"
//#include "IpOrigIpoptNLP.hpp"

namespace xpp {
namespace zmp {


IpoptAdapter::IpoptAdapter(NLP& nlp,
                           IVisualizer& visualizer)
    :nlp_(nlp),
     visualizer_(visualizer)
{
}


bool IpoptAdapter::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                         Index& nnz_h_lag, IndexStyleEnum& index_style)
{
  // How many variables to optimize over
  n = nlp_.GetNumberOfOptimizationVariables();
  std::cout << "optimizing n= " << n << " variables\n";

  m = nlp_.GetNumberOfConstraints();
  std::cout << "with m= " << m << "constraints\n";


  // nonzeros in the jacobian of the constraint g(x)
  double* x_start = new double[n];
  for (int i=0; i<n; ++i) {
    x_start[i] = 1.0;
  }
  NLP::Jacobian jac = nlp_.EvalJacobianOfConstraints(x_start);
  nnz_jac_g = jac.nonZeros();
//  nnz_jac_g = m * n; // fixme all constraints depend on all inputs

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
  auto bounds_x = nlp_.GetBoundsOnOptimizationVariables();
  for (uint c=0; c<bounds_x.size(); ++c) {
    x_lower[c] = bounds_x.at(c).lower_;
    x_upper[c] = bounds_x.at(c).upper_;
  }

  // specific bounds depending on equality and inequality constraints
  auto bounds_g = nlp_.GetBoundsOnConstraints();
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
	// Here, we assume we only have starting values for x, if you code
	// your own NLP, you can provide starting values for the others if
	// you wish.
	assert(init_x == true);
	assert(init_z == false);
	assert(init_lambda == false);

  VectorXd x_all = nlp_.GetStartingValues();
  Eigen::Map<VectorXd>(&x[0], x_all.rows()) = x_all;

  return true;
}


bool IpoptAdapter::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
  obj_value = nlp_.EvaluateCostFunction(x);
  return true;
}


bool IpoptAdapter::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  Eigen::VectorXd grad = nlp_.EvaluateCostFunctionGradient(x);
  Eigen::Map<Eigen::MatrixXd>(grad_f,n,1) = grad;
	return true;
}


bool IpoptAdapter::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
  VectorXd g_eig = nlp_.EvaluateConstraints(x);
  Eigen::Map<VectorXd>(g,m) = g_eig;
  return true;
}


bool IpoptAdapter::eval_jac_g(Index n, const Number* x, bool new_x,
                       Index m, Index nele_jac, Index* iRow, Index *jCol,
                       Number* values)
{
//  assert(nele_jac == n*m); // number of elements in jacobian


  // FIXME exploit sparsity structure more.
	// say at which positions the nonzero elements of the jacobian are
  if (values == NULL) {


//    // return the structure of the jacobian of the constraints - i.e. specify positions of non-zero elements.
//  	int c_nonzero = 0;
//    for (int row=0; row<m; ++row) {
//      for (int col=0; col<n; ++col) {
//  			iRow[c_nonzero] = row;
//  			jCol[c_nonzero] = col;
//  			c_nonzero++;
//  		}
//  	}


    // fixme, this doesn't work, because sparsity structure estimated by
    // zero entries, but this could be just for initial values of decision variables
    // -> define all the elements as sparse matrix
    // nonzeros in the jacobian of the constraint g(x)
    double* x_start = new double[n];
    for (int i=0; i<n; ++i) {
      x_start[i] = 1.0;
    }

    NLP::Jacobian jac = nlp_.EvalJacobianOfConstraints(x_start);
    std::cout << "first jac: " << jac << std::endl;
    int nele=0; // nonzero cells in jacobian
    for (int k=0; k<jac.outerSize(); ++k) {
      for (NLP::Jacobian::InnerIterator it(jac,k); it; ++it) {
        iRow[nele] = it.row();
        jCol[nele] = it.col();
        nele++;
      }
    }





  }
  else {
//    // only gets used if "jacobian_approximation finite-difference-values" is not set
//    Eigen::MatrixXd jac(m,n);
//    num_diff_constraints_.df(nlp_structure_.ConvertToEigen(x),jac);
//    Eigen::Map<Eigen::MatrixXd>(values,jac.rows(),jac.cols()) = jac;


    int nele=0;
    NLP::Jacobian jac = nlp_.EvalJacobianOfConstraints(x);
    for (int k=0; k<jac.outerSize(); ++k)
      for (NLP::Jacobian::InnerIterator it(jac,k); it; ++it)
        values[nele++] = it.value();

    std::cout << "jac.size(): " << jac.rows() << " x " << jac.cols() << std::endl;
    std::cout << "jac: " << jac << std::endl;

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

  visualizer_.Visualize();
	return true;
}



void IpoptAdapter::finalize_solution(Ipopt::SolverReturn status,
                              Index n, const Number* x, const Number* z_L, const Number* z_U,
                              Index m, const Number* g, const Number* lambda,
                              Number obj_value,
			                        const Ipopt::IpoptData* ip_data,
			                        Ipopt::IpoptCalculatedQuantities* ip_cq)
{


  nlp_.SetVariables(x);
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



} // namespace zmp
} // namespace xpp


