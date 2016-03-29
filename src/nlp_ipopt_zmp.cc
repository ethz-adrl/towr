// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: MyNLPHyQ.cpp 2005 2011-06-06 12:55:16Z stefan $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#include <xpp/zmp/nlp_ipopt_zmp.h>

namespace Ipopt {

#define prt(x) std::cout << #x << " = " << std::endl << x << std::endl << std::endl;

NlpIpoptZmp::NlpIpoptZmp(const MatVec& cf_quadratic,
                         const xpp::zmp::Constraints& constraints)
    :cost_function_quadratic_(cf_quadratic),
     constraints_(constraints)
{}

NlpIpoptZmp::~NlpIpoptZmp()
{}

void NlpIpoptZmp::SetupNlp(
    const xpp::hyq::SuppTriangleContainer& supp_triangle_container,
    const xpp::zmp::ContinuousSplineContainer& zmp_spline_container,
    const MatVec& qp_cost_function,
    const MatVec& qp_equality_constraints,
    const Eigen::VectorXd& initial_coefficients)
{
  supp_triangle_container_ = supp_triangle_container;
  zmp_spline_container_ = zmp_spline_container;

  n_spline_coeff_ = zmp_spline_container.GetTotalFreeCoeff();
  n_steps_ = supp_triangle_container.footholds_.size(); // use intial footholds for this
  footholds_.resize(n_steps_);

  // cost function and equality constraints
  eq_   =  qp_equality_constraints;
  n_eq_constr_    = qp_equality_constraints.v.rows();     // spline junctions

  // inequality constraints
  double walking_height = 0.58;
  x_zmp_ = zmp_spline_container.ExpressZmpThroughCoefficients(walking_height, xpp::utils::X);
  y_zmp_ = zmp_spline_container.ExpressZmpThroughCoefficients(walking_height, xpp::utils::Y);
  ineq_= supp_triangle_container.AddLineConstraints(x_zmp_, y_zmp_, zmp_spline_container);
  n_ineq_constr_  = ineq_.v.rows();  // support polygons

  initial_coefficients_ = initial_coefficients;
  initial_footholds_ = supp_triangle_container.footholds_;



  // TODO pass spline container and supp_tr in constructor, and fill rest
  constraints_.zmp_spline_container_ = zmp_spline_container;
  constraints_.initial_footholds_ = supp_triangle_container.footholds_;
  constraints_.supp_triangle_container_ = supp_triangle_container;
  constraints_.x_zmp_ = zmp_spline_container.ExpressZmpThroughCoefficients(walking_height, xpp::utils::X);
  constraints_.y_zmp_ = zmp_spline_container.ExpressZmpThroughCoefficients(walking_height, xpp::utils::Y);
}


bool NlpIpoptZmp::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                         Index& nnz_h_lag, IndexStyleEnum& index_style)
{
  // How many variables to optimize over
  n = n_spline_coeff_ + 2*n_steps_; // x,y-coordinate of footholds
  std::cout << "optimizing n= " << n << " variables ";


  // constraints
  std::cout << "with " << n_eq_constr_ << " equality and "
                       << n_ineq_constr_ << " inequality constraints\n";

  m = n_eq_constr_ + n_ineq_constr_ + 2*n_steps_; // fix y-coordinate + distance of each step

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

  // bounds on equality contraint always be equal (and zero).
  int c=0;
  for (int i=0; i<n_eq_constr_; ++i)
  {
		g_l[c] = g_u[c] = 0.0;
		c++;
  }

  // inequality on inside of support polygon
  for (int i=0; i<n_ineq_constr_; ++i)
  {
    g_l[c] = 0.0;
    g_u[c] = +1.0e19;
    c++;
  }

  // fixing the y position of the footholds to the intial values
  for (int i=0; i<2*n_steps_; ++i)
  {
    g_l[c] = g_u[c] = 0.0;
    c++;
  }


//  // restricting the length of each step
//  for (int i=0; i<n_steps_; ++i)
//  {
//    g_l[c] = -1.0e19;
//    g_u[c] = 0.0;
//    c++;
//  }

  assert(c == m);

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
  if (initial_coefficients_.rows() != n_spline_coeff_ ) {
    initial_coefficients_.resize(n_spline_coeff_,1);
    initial_coefficients_.setZero();
  }


  int c = 0;
	for (int i=0; i<initial_coefficients_.rows(); ++i) {
	  x[c++] = initial_coefficients_[i]; // splines of the form x = t^5+t^4+t^3+t^2+t
	}



	// initialize with steps from footstep planner
	for (int i=0; i<n_steps_; ++i) {

	  xpp::hyq::Foothold f = initial_footholds_.at(i);

	  x[c++] = f.p.x();
	  x[c++] = f.p.y();
	}

  return true;
}

bool NlpIpoptZmp::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
//  std::cout << "in eval_f";

//  min 0.5 * x G x + g0 x
//  s.t.
//  CE^T x + ce0 = 0
//  CI^T x + ci0 >= 0
//
//  The matrix and vectors dimensions are as follows:
//  G: n * n
//  g0: n
//
//  CE: n * p
//  ce0: p
//
//  CI: n * m
//  ci0: m
//
//  x: n

  UpdateOptimizationVariables(x);

  // make an eigen vector out of the optimization variables
//  Eigen::Map<const Eigen::VectorXd> x_vec(x,n_spline_coeff_); // ATTENTION: still not sure if this is correct
//  Eigen::VectorXd x_vec(n);
//  for (int i=0; i<x_vec.rows(); ++i) {
//    x_vec[i] = x[i];
//  }

  obj_value = cost_function_quadratic_.EvalObjective(x_coeff_);

//  obj_value = 0.0;
//  obj_value = x_vec.transpose() * cf_.M * x_vec;
//  obj_value += cf_.v.transpose() * x_vec;

  return true;
}


bool NlpIpoptZmp::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  UpdateOptimizationVariables(x);

  for (int i=0; i<n; ++i) {
    grad_f[i] = 0.0;
  }

//  Eigen::Map<const Eigen::VectorXd> x_vec(x,n_spline_coeff_);

  Eigen::VectorXd grad_f_vec = cost_function_quadratic_.EvalGradientOfObjective(x_coeff_);

  Eigen::Map<Eigen::VectorXd>(grad_f,n_spline_coeff_) = grad_f_vec;

	return true;
}


bool NlpIpoptZmp::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
  //  min 0.5 * x G x + g0 x
  //  s.t.
  //  CE^T x + ce0 = 0
  //  CI^T x + ci0 >= 0
  //
  //  The matrix and vectors dimensions are as follows:
  //  G: n * n
  //  g0: n
  //
  //  CE: n * p
  //  ce0: p
  //
  //  CI: n * m 
  //  ci0: m
  //
  //  x:

//  Eigen::VectorXd x_vec(n);
//  for (int r=0; r<x_vec.rows(); ++r) {
//    x_vec[r] = x[r];
//  }
  using namespace xpp::utils::coords_wrapper;
  UpdateOptimizationVariables(x);

//  Eigen::Map<const Eigen::VectorXd> x_vec(x,n_spline_coeff_);


  // equality constraints
  Eigen::VectorXd g_vec_eq = eq_.M*x_coeff_ + eq_.v;


  // inequality constraints
  //here i am adapting the constraints depending on the footholds
//  for (int i=0; i<n_steps_; ++i) {
//    supp_triangle_container_.footholds_.at(i).p.segment<2>(xpp::utils::X) = footholds_.at(i);
//  }
//  ineq_ = supp_triangle_container_.AddLineConstraints(x_zmp_, y_zmp_, zmp_spline_container_);
//  Eigen::VectorXd g_vec_in = ineq_.M*x_coeff_ + ineq_.v;

  Eigen::VectorXd g_vec_in = constraints_.EvalContraints(footholds_, x_coeff_);


//  // constraints on the footsteps
//  Eigen::VectorXd g_vec_footsteps(2*n_steps_);
//  g_vec_footsteps.setZero();
//  // fix footholds in x and y direction
//  int c=0;
//  for (uint i=0; i<n_steps_; ++i) {
//
//    xpp::hyq::Foothold f = initial_footholds_.at(i);
//
//    int idx = 2*i;
//
//    g_vec_footsteps(c++) = footholds_.at(i).x() - f.p.x();
//    g_vec_footsteps(c++) = footholds_.at(i).y() - f.p.y();
//  }


  // restrict distance to previous foothold small
  // initialize with steps from footstep planner
//  for (int i=0; i<supp_triangle_container_.footholds_.size(); ++i) {
//
//    int idx = n_spline_coeff_+2*i;
//    Eigen::Vector2d f;
//    f << x[idx+0], x[idx+1];
//
//    Eigen::Vector2d f_prev = start_stance_[supp_triangle_container_.footholds_.at(i).leg].p.segment<2>(0);
//    if (i>=4) {
//      int idx = n_spline_coeff_+2*(i-4);
//      f_prev << x[idx+0], x[idx+1];
//    }
//
//    double dx = f.x()-f_prev.x();
//    double dy = f.y()-f_prev.y();
//
//    g_vec_footsteps(c++) = hypot(dx,dy) - 0.3;
//  }






  // combine all the g vectors
  Eigen::VectorXd g_vec(m);
  g_vec << g_vec_eq, g_vec_in;

  // fill these values into g
  Eigen::Map<Eigen::VectorXd>(g,m) = g_vec; // don't know which to use

  return true;
}


bool NlpIpoptZmp::eval_jac_g(Index n, const Number* x, bool new_x,
                       Index m, Index nele_jac, Index* iRow, Index *jCol,
                       Number* values)
{

	// say at which positions the nonzero elements of the jacobian are
  if (values == NULL) {
    // return the structure of the jacobian of the constraints - i.e. specify positions of non-zero elements.
  	int c_nonzero = 0;
  	for (int row=0; row<m; ++row) {
  		for (int col=0; col<n; ++col)
  		{
  			iRow[c_nonzero] = row;
  			jCol[c_nonzero] = col;
  			c_nonzero++;
  		}
  	}
  }
  else {
  // approximated by ipopt through finite differences
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
	// print something useful here
	return true;
}



void NlpIpoptZmp::finalize_solution(SolverReturn status,
                              Index n, const Number* x, const Number* z_L, const Number* z_U,
                              Index m, const Number* g, const Number* lambda,
                              Number obj_value,
			                        const IpoptData* ip_data,
			                        IpoptCalculatedQuantities* ip_cq)
{
  UpdateOptimizationVariables(x);

//  // make an eigen vector out of the optimization variables
//  x_final_spline_coeff_.resize(n_spline_coeff_);
//  for (int r=0; r<x_final_spline_coeff_.rows(); ++r) {
//    x_final_spline_coeff_[r] = x[r];
//  }
//
//  // make an eigen vector out of the optimization variables
//  x_final_footholds_.resize(2*n_steps_);
//  for (int r=0; r<x_final_footholds_.rows(); ++r) {
//    x_final_footholds_[r] = x[n_spline_coeff_+r];
//  }



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

void
NlpIpoptZmp::UpdateOptimizationVariables(const Number* x)
{
  x_coeff_ = Eigen::Map<const Eigen::VectorXd>(x,n_spline_coeff_);
  for (int i=0; i<footholds_.size(); ++i) {
    Eigen::Vector2d& f = footholds_.at(i);
    f.x() = x[n_spline_coeff_+2*i+xpp::utils::X];
    f.y() = x[n_spline_coeff_+2*i+xpp::utils::Y];
  }
//  x_footholds_    = Eigen::Map<const Eigen::VectorXd>(x+n_spline_coeff_,2*n_steps_);
}

} // namespace Ipopt


