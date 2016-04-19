// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: MyNLPHyQ.cpp 2005 2011-06-06 12:55:16Z stefan $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#include <xpp/zmp/nlp_ipopt_zmp.h>

#include <ros/ros.h>

namespace Ipopt {

#define prt(x) std::cout << #x << " = " << std::endl << x << std::endl << std::endl;

NlpIpoptZmp::NlpIpoptZmp(const CostFunction& cost_function,
                         const Constraints& constraints,
                         const NlpStructure& nlp_structure,
                         const VectorXd& initial_spline_coefficients)
    :cost_function_(cost_function),
     num_diff_cost_function_(cost_function),
     constraints_(constraints),
     num_diff_constraints_(constraints),
     nlp_structure_(nlp_structure),
     zmp_publisher_(constraints.GetSplineContainer())
{
  initial_spline_coeff_ = initial_spline_coefficients;
}


bool NlpIpoptZmp::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                         Index& nnz_h_lag, IndexStyleEnum& index_style)
{
  // How many variables to optimize over
  n = nlp_structure_.GetOptimizationVariableCount(); // x,y-coordinate of footholds
  std::cout << "optimizing n= " << n << " variables\n";

  m = constraints_.GetBounds().size();
  std::cout << "with m= " << m << "constraints\n";


  // nonzeros in the jacobian of the constraint g(x)
  nnz_jac_g = m * n; // all constraints depend on all inputs

  // nonzeros in the hessian of the lagrangian
  // (one in the hessian of the objective for x2,
  //  and one in the hessian of the constraints for x1)
  nnz_h_lag = n*n;

  // start index at 0 for row/col entries
  index_style = C_STYLE;

  return true;
}

bool NlpIpoptZmp::get_bounds_info(Index n, Number* x_lower, Number* x_upper,
                            Index m, Number* g_l, Number* g_u)
{

  // no bounds on the spline coefficients of footholds
  for (int i=0; i<n; ++i) {
    x_lower[i] = -1.0e19;
    x_upper[i] = +1.0e19;
  }

  // specific bounds depending on equality and inequality constraints
  std::vector<Constraints::Bound> bounds = constraints_.GetBounds();
  for (uint c=0; c<bounds.size(); ++c) {
    g_l[c] = bounds.at(c).lower_;
    g_u[c] = bounds.at(c).upper_;
  }

  return true;
}

bool NlpIpoptZmp::get_starting_point(Index n, bool init_x, Number* x,
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


	// set initial value to zero if wrong size input
  if (initial_spline_coeff_.rows() != nlp_structure_.n_spline_coeff_ ) {
    initial_spline_coeff_.resize(nlp_structure_.n_spline_coeff_,1);
    initial_spline_coeff_.setZero();
  }


  int c = 0;
	for (int i=0; i<initial_spline_coeff_.rows(); ++i) {
	  x[c++] = 0.0; //initial_spline_coeff_[i]; // splines of the form x = t^5+t^4+t^3+t^2+t
	}


	// initialize footstep locations
	for (int i=0; i<nlp_structure_.n_steps_; ++i) {
	  xpp::hyq::LegID leg = constraints_.GetPlannedFoothold(i).leg;

	  // initialize with start stance
	  x[c++] = constraints_.GetStartStance(leg).p.x();
	  x[c++] = constraints_.GetStartStance(leg).p.y();

	  // // intialize with planned footholds
    // x[c++] = constraints_.planned_footholds_.at(i).p.y();
    // x[c++] = constraints_.planned_footholds_.at(i).p.y();
	}

  return true;
}


bool NlpIpoptZmp::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
  nlp_structure_.UpdateOptimizationVariables(x);
  obj_value = cost_function_.EvalCostFunction(nlp_structure_.opt_all_);
  return true;
}


bool NlpIpoptZmp::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  nlp_structure_.UpdateOptimizationVariables(x);

  Eigen::MatrixXd jac(1,n);
  num_diff_cost_function_.df(nlp_structure_.opt_all_, jac);

  Eigen::Map<Eigen::MatrixXd>(grad_f,1,n) = jac;
	return true;
}


bool NlpIpoptZmp::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
  nlp_structure_.UpdateOptimizationVariables(x);
  //FIXME pass nlp structure directly
  VectorXd g_eig = constraints_.EvalContraints(nlp_structure_.opt_all_);
  Eigen::Map<VectorXd>(g,m) = g_eig;
  return true;
}


bool NlpIpoptZmp::eval_jac_g(Index n, const Number* x, bool new_x,
                       Index m, Index nele_jac, Index* iRow, Index *jCol,
                       Number* values)
{

  // FIXME exploit sparsity structure more.
	// say at which positions the nonzero elements of the jacobian are
  if (values == NULL) {
    // return the structure of the jacobian of the constraints - i.e. specify positions of non-zero elements.
  	int c_nonzero = 0;
  	// colum major so eigen matrices can be mapped directly as default
    for (int col=0; col<n; ++col) {
  	  for (int row=0; row<m; ++row) {
  			iRow[c_nonzero] = row;
  			jCol[c_nonzero] = col;
  			c_nonzero++;
  		}
  	}
  }
  else {
    nlp_structure_.UpdateOptimizationVariables(x);
    Eigen::MatrixXd jac(m,n);
    num_diff_constraints_.df(nlp_structure_.opt_all_,jac);
    Eigen::Map<Eigen::MatrixXd>(values,jac.rows(),jac.cols()) = jac;
  }

  return true;
}



bool NlpIpoptZmp::intermediate_callback(AlgorithmMode mode,
                                   Index iter, Number obj_value,
                                   Number inf_pr, Number inf_du,
                                   Number mu, Number d_norm,
                                   Number regularization_size,
                                   Number alpha_du, Number alpha_pr,
                                   Index ls_trials,
                                   const IpoptData* ip_data,
                                   IpoptCalculatedQuantities* ip_cq)
{
//   std::cin.get();

  ZmpPublisher::VecFoothold footholds = constraints_.GetPlannedFootholds();
  for (uint i=0; i<footholds.size(); ++i) {
    footholds.at(i).p << nlp_structure_.opt_footholds_.at(i).x(),
                         nlp_structure_.opt_footholds_.at(i).y(),
                         0.0;
  }

  zmp_publisher_.zmp_msg_.markers.clear();
  zmp_publisher_.AddRvizMessage(nlp_structure_.opt_coeff_, footholds, constraints_.gap_center_x_,
                                constraints_.gap_width_x_, "nlp", 1.0);
  zmp_publisher_.publish();

	return true;
}



void NlpIpoptZmp::finalize_solution(SolverReturn status,
                              Index n, const Number* x, const Number* z_L, const Number* z_U,
                              Index m, const Number* g, const Number* lambda,
                              Number obj_value,
			                        const IpoptData* ip_data,
			                        IpoptCalculatedQuantities* ip_cq)
{
  nlp_structure_.UpdateOptimizationVariables(x);

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


