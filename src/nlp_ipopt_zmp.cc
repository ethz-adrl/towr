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
#define prt(x)


/* Constructor. */
NlpIpoptZmp::NlpIpoptZmp()
{
}

NlpIpoptZmp::~NlpIpoptZmp()
{}


bool NlpIpoptZmp::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                         Index& nnz_h_lag, IndexStyleEnum& index_style)
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
  //  x: n

  // How many variables to optimize over

  n = cf_.M.rows();
  std::cout << "optimizing n= " << n << " variables ";


  // constraints
  int n_eq = eq_.v.rows();
  int n_ineq = ineq_.v.rows();
  std::cout << "with " << n_eq << " equality and " << n_ineq << " inequality constraints\n";

  m = n_eq + n_ineq;

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
  // no bounds on the spline coefficients
  for (int i=0; i<n; ++i) {
    x_lower[i] = -1.0e19;
    x_upper[i] = +1.0e19;
  }

  // bounds on equality contraint always be equal (and zero).
  int n_eq = eq_.v.rows();
  int n_ineq = ineq_.v.rows();

  std::cout << "n_eq: " << n_eq;
  std::cout << "n_ineq: " << n_ineq;



  for (int i=0; i<n_eq; ++i)
  {
//    // allow tiny deviation from equality constraint to avoid
//    // "TOO_FEW_DOF" error message from ipopt
//		g_l[i] =  -0.001;
//		g_u[i] =  +0.001;

		g_l[i] = g_u[i] = 0.0; // Throws: Too few DoF errors

  }

  // inequality on inside of support polygon
  // allow also numbers slightly smaller than zero for if points start at the border
  for (int i=n_eq; i<m; ++i)
  {
    g_l[i] = 0.0;//0.0;
    g_u[i] = +1.0e19;
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


	for (int i=0; i<n; ++i) {
	  x[i] = initial_values_[i]; // splines of the form x = t^5+t^4+t^3+t^2+t
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

  // make an eigen vector out of the optimization variables
  Eigen::Map<const Eigen::VectorXd> x_vec(x,n); // FIXME, this doesn't work, ask Michael
//  Eigen::VectorXd x_vec(n);
//  for (int i=0; i<x_vec.rows(); ++i) {
//    x_vec[i] = x[i];
//  }


//  Eigen::MatrixXd M = zmp_optimizer_.cf_.M;
  obj_value = 0.0;
  obj_value = x_vec.transpose() * cf_.M * x_vec;
  obj_value += cf_.v.transpose() * x_vec;

  return true;
}


bool NlpIpoptZmp::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  // if nothing set, then this is automatically done by ipopt through finite
	// differences

  Eigen::Map<const Eigen::VectorXd> x_vec(x,n); // FIXME, this doesn't work, ask Michael
//  Eigen::VectorXd x_vec(n);
//  for (int i=0; i<x_vec.rows(); ++i) {
//    x_vec[i] = x[i];
//  }

  Eigen::VectorXd grad_f_vec = cf_.M*x_vec;


  Eigen::Map<Eigen::VectorXd>(grad_f,n) = grad_f_vec; // don't know which to use
//  for (int i=0; i<n; ++i) {
//    grad_f[i] = grad_f_vec[i];
//  }



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
  Eigen::Map<const Eigen::VectorXd> x_vec(x,n);


  // equality constraints
//  Eigen::MatrixXd A_eq = eq_.M.transpose();
//  Eigen::VectorXd b_eq = eq_.v;
  Eigen::VectorXd g_vec_eq = eq_.M.transpose()*x_vec + eq_.v;


  // inequality constraints
//  Eigen::MatrixXd A_in = ineq_.M.transpose();
//  Eigen::VectorXd b_in = ineq_.v;
  Eigen::VectorXd g_vec_in = ineq_.M.transpose()*x_vec + ineq_.v;


  // combine the two g vectors
  Eigen::VectorXd g_vec(g_vec_eq.rows()+g_vec_in.rows());
  g_vec << g_vec_eq, g_vec_in;


  // fill these values into g
  Eigen::Map<Eigen::VectorXd>(g,m) = g_vec; // don't know which to use
//  g = g_vec.data();
//  for (int r=0; r<m; ++r) {
//    g[r] = g_vec[r];
//  }






//	double T = kTmaxStart; // x[n-1];
//	ODEState y_final;
//
//	integrateODE(u, T, y_final);
//	prt(y_final);
//
//
//  for (int i=0; i<m; ++i)
//  {
//    g[i] = 0; // initialize all constraints as fullfilled just in case
//  }
//
//
//	int mm = 0;
//	for (int i=0; i<y_final_des_.rows(); ++i)
//	{
//
////	  if (i == iit::rbd::AX || i == iit::rbd::AY || i == iit::rbd::AZ)
////	    g[mm++] = y_final[i] - y_final_des_[i]; // ang + lin posusleep(microseconds);
//
////	  if (i == iit::rbd::LZ)
////	    g[mm++] = y_final[i] - y_final_des_[i]; // ang + lin posusleep(microseconds);
//
//
//
//	  if (0  <= i && i <  6) {
//	    // final state
////	    if (i==rbd::AX || i==rbd::AY || i==rbd::AZ )
//	      g[mm++] = y_final[i] - y_final_des_[i]; // pos (ang + lin)
////	    if (i==rbd::LZ)
////	      g[mm++] = y_final[i] - y_final_des_[i]; // pos (ang + lin)
//	    // intermediate state
////	    g[mm++] = integrator_.ode_states_.at(kTmaxStart/2.0 / kTIntegrationStep)[i] - y_inter_des_[i];
//	  }
//
//	  if (6  <= i && i < 12)  {
//	    g[mm++] = y_final[i] - y_final_des_[i]; // velocity (ang + lin)
//	  }
//
//	  // TODO LF_LEG final joint position
////	  if (12 <= i && i < 15 ) g[mm++] = y_final[i] - y_final_des_[i];
//
//	  //		if (12 <= i && i < 24)  g[mm++] = y_final[i] - y_final_des_[i]; // joint ang + lin pos
//
//	  // final joint velocities
//	  if (24 <= i && i < 36)  g[mm++] = y_final[i] - y_final_des_[i];   // joint ang + lin vel
//	}
//
//
//	// add constraint that section times add up to final time
//	double t_section_sum;
//  for (int i=0; i < kInputNodesCount; ++i)
//    t_section_sum += u[kInputNodesCount*iit::HyQ::jointsCount + i];
//	g[mm++] = kTmaxStart - t_section_sum;
//
//	if (mm > m) throw 20; // increase number of constraints

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



//bool NlpIpoptZmp::intermediate_callback(AlgorithmMode mode,
//                                   Index iter, Number obj_value,
//                                   Number inf_pr, Number inf_du,
//                                   Number mu, Number d_norm,
//                                   Number regularization_size,
//                                   Number alpha_du, Number alpha_pr,
//                                   Index ls_trials,
//                                   const IpoptData* ip_data,
//                                   IpoptCalculatedQuantities* ip_cq)
//{
//	// print something useful here
////  std::cout << "count = " << count_ << "\t delta count = " << count_ - count_prev_ << std::endl;
////  count_prev_ = count_;
//	return true;
//}



void NlpIpoptZmp::finalize_solution(SolverReturn status,
                              Index n, const Number* x, const Number* z_L, const Number* z_U,
                              Index m, const Number* g, const Number* lambda,
                              Number obj_value,
			                        const IpoptData* ip_data,
			                        IpoptCalculatedQuantities* ip_cq)
{
//  // here is where we would store the solution to variables, or write to a file, etc
//  // so we could use the solution. Since the solution is displayed to the console,
//  // we currently do nothing here.


  // make an eigen vector out of the optimization variables
  x_final_.resize(n);
  for (int r=0; r<x_final_.rows(); ++r) {
    x_final_[r] = x[r];
  }



//
//	double torque_sum = 0;
//	std::cout << "final torques:\n" << std::setprecision(3) << std::fixed;
//	for (int i=0; i<n; ++i)
//	{
//		std::cout << u[i] << ", ";
//		if (i%iit::HyQ::jointsCount == 11) std::cout << std::endl;
//		torque_sum += std::pow(u[i],2);
//	}
//
//	std::cout << "\ntorques sum: " << torque_sum;
//
//
//	std::cout << "\nfinal state body:\n";
//	for (int i=0; i<6+6; ++i)
//	{
//		std::cout << integrator_.ode_states_.back()[i] << "  ";
//	}
//
//	std::cout << "\nfinal position joints:\n";
//	for (int i=12; i<12+iit::HyQ::jointsCount; ++i)
//	{
//		std::cout << integrator_.ode_states_.back()[i] << " ";
//	}
//
//	std::cout << "\nfinal velocity velocities:\n";
//	for (int i=12+iit::HyQ::jointsCount; i<12+2*iit::HyQ::jointsCount; ++i)
//	{
//		std::cout << integrator_.ode_states_.back()[i] << "  ";
//	}
//
//
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


