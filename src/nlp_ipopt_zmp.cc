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
NlpIpoptZmp::NlpIpoptZmp(bool init_from_file)
{

//  init_from_file_ = init_from_file;
//
//
//
//  // ATTENTION: if changing the body position, the joint would theoretically
//  // also need to be adapted using inverse kinematics
//  // throws of my contact force calculation for sure
//	// always start at initial zero pose of arm
//	y_start_.setZero();
//	y_start_ <<
//		0,  0,  0,					 // base ori
//		0.0,  0.0,  0.552,       // base pos
//		0,0,0,0,0,0,     // base vel
//
//	  -0.2,  0.723,  -1.458, // LF pos
//	  -0.2,  0.723,  -1.458, // RF
//	  -0.2, -0.723,   1.458, // LH
//	  -0.2, -0.723,   1.458, // RH
//	  0,0,0,0,0,0,0,0,0,0,0,0; // joint vel
//
//
////	// Configuration for robot being shifted to back right
////  y_start_ <<
////    0,  0,  0,           // base ori
////    -0.2,  -0.15,   0.552,       // base pos
////    0,0,0,0,0,0,     // base vel
////
////    -0.433,  0.182,  -1.057, // LF pos
////    0.058,  0.283,  -1.340,  // RF
////    -0.433,  -0.854,  1.016, // LH
////    0.058,  -1.029,  1.304,  // RH
////    0,0,0,0,0,0,0,0,0,0,0,0; // joint vel
//
//
//  int int_count = kTmaxStart/kTIntegrationStep;
//  for (int i=0; i<int_count; ++i) {
//    integrator_.ode_states_.push_back(y_start_);
//  }
//
//
//  y_inter_des_.setZero();
//  y_inter_des_ <<
//      0,  0.0,   0,       // base ori
//      -0.1,  -0.1,   0.552,   // base pos
//      0,0,0,0,0,0, // base vel
//
//      -0.2,  0.723,  -1.458, // LF pos
//      -0.2,  0.723,  -1.458, // RF
//      -0.2, -0.723,   1.458, // LH
//      -0.2, -0.723,   1.458, // RH
//      0,0,0,0,0,0,0,0,0,0,0,0; // joint vel
//
//
//	y_final_des_.setZero();
//	y_final_des_ <<
//			0,  -0.0,   0,       // base ori
//			0.0,  0.0,  0.552,       // base pos
//			0,0,0,0,0,0, // base vel
//
//			-0.2,  0.723,  -1.458, // LF pos
//		  -0.2,  0.723,  -1.458, // RF
//		  -0.2, -0.723,   1.458, // LH
//		  -0.2, -0.723,   1.458, // RH
//		  0,0,0,0,0,0,0,0,0,0,0,0; // joint vel
//
//
//
//
//  count_ = 0;
//  count_prev_ = 0;
}

NlpIpoptZmp::~NlpIpoptZmp()
{}


bool NlpIpoptZmp::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                         Index& nnz_h_lag, IndexStyleEnum& index_style)
{
//  // How many variables to optimize over
//	// +1 is for making total time of trajectory adaptive
//  n = kInputNodesCount*iit::HyQ::jointsCount + kInputNodesCount; // + 1 if time is to be optimized as well
//
//  // equality constraints
//  // only penalizing base state (pos/vel) = 12
//  m = 36;  // final body state constraint
//  m += 1;  // node spacing adds up to one
//
//  // nonzeros in the jacobian of the constraint g(x) (one for x1, and one for x2),
//  nnz_jac_g = m * n; // all constraints depend on all inputs
//
//  // and 2 nonzeros in the hessian of the lagrangian
//  // (one in the hessian of the objective for x2,
//  //  and one in the hessian of the constraints for x1)
//  nnz_h_lag = n*n;
//
//  // We use the standard fortran index style for row/col entries
//  index_style = C_STYLE;
//
//  return true;
}

bool NlpIpoptZmp::get_bounds_info(Index n, Number* u_lower, Number* u_upper,
                            Index m, Number* g_l, Number* g_u)
{
//  // bounds on the inputs
//  for (int node=0; node<kInputNodesCount; ++node) {
//    for (int u=0; u<iit::HyQ::jointsCount; ++u) {
//      int i = node*iit::HyQ::jointsCount + u;
//      u_lower[i] = -200;
//      u_upper[i] = +200;
//
//      // no torque allowed on front legs
////      if (node == 2 || node == 3) {
////        if (0 <= u && u <3) {
////          // sometimes, non strict bounds allowed solver to find solution
////          u_lower[i] = -1e-1;
////          u_upper[i] =  -u_lower[i];
////        }
////      }
//    }
//  }
//
//  // bounds on node spacing
//  for (int i=kInputNodesCount*iit::HyQ::jointsCount; i < n; ++i)
//  {
//    u_lower[i] = 0.0;
//    u_upper[i] = kTmaxStart;
//  }
//
//
//
//  // bounds on equality contraint always be equal (and zero).
//  for (int i=0; i<m; ++i)
//  {
//		g_l[i] =  0.0;
//		g_u[i] =  0.0;
//  }
//
//  return true;
}

bool NlpIpoptZmp::get_starting_point(Index n, bool init_u, Number* u,
                               bool init_z, Number* z_L, Number* z_U,
                               Index m, bool init_lambda,
                               Number* lambda)
{
//	// Here, we assume we only have starting values for x, if you code
//	// your own NLP, you can provide starting values for the others if
//	// you wish.
//	assert(init_u == true);
//	assert(init_z == false);
//	assert(init_lambda == false);
//
//
//
//  // if initialize from archive from cereal
//	if (init_from_file_) {
//
//	  std::ifstream is("optimized_torques.xml");
//	  cereal::XMLInputArchive xmlarchive(is);
//	  Eigen::MatrixXd opt_u;
//	  xmlarchive(cereal::make_nvp("U", opt_u));
//	  for (int i=0; i<opt_u.cols(); ++i)
//	    u[i] = opt_u(i);
//
//	  return true;
//	}
//
//
//
//	// start with all torques for all joints for all timesteps to zero
//	// initialize with hovering torquesq_des
//	iit::HyQ::JointState q,qd,qdd_des, tau;
//
//	q = y_start_.segment(2*6,iit::HyQ::jointsCount);
//	qd = y_start_.segment(2*6 + iit::HyQ::jointsCount,iit::HyQ::jointsCount);
//	qdd_des.setZero();
//	decltype(id_)::Velocity trunk_v = y_start_.segment(6,6);
//
//
//	decltype(id_)::Acceleration g_W, g_B;
//	g_W.setZero(); g_B.setZero();
//	g_W[iit::rbd::LZ] = -iit::rbd::g;
//	g_B = g_W; // assuming robot starts in horizontal angle
//
//
//	// assume equally distributed constant contact forces on all joints
//  // count endeffectors in contact
//	iit::rbd::ForceVector des_force_body_W, des_force_body_B;
//	des_force_body_W.setZero(); des_force_body_B.setZero();
//	des_force_body_W(iit::rbd::LZ) = - ip_.getTotalMass() * g_W(iit::rbd::LZ); // N
//	des_force_body_B = des_force_body_W; // Since assuming robot is initially horizontal
//
//
//	std::cout << ip_.getCOM_trunk() << std::endl;
//
//
//	std::array<bool, kNumEE> is_ee_in_contact_start = {true, true, true, true};
//
//  Eigen::MatrixXd base_M_ee = integrator_.hyq_system_dyn_.hyq_kinematics_.getMapFromEndeffectorForcesToBaseWrench(q,is_ee_in_contact_start);
//
//  // only use ee z coordinates
//  int z=0;
//  Eigen::MatrixXd base_M_ee_z(base_M_ee.rows(), base_M_ee.cols()/rbd::k3D);
//  for (int i=0; i<base_M_ee.cols(); ++i) {
//    if (i==2 || i==5 || i==8 || i==11) // extract z parts
//      base_M_ee_z.col(z++) = base_M_ee.col(i);
//  }
//
//
//  // find optimal ground reaction forces that create the desired base wrench
////  math::MatrixAnalyser<6, 3*4> matrix_analyser(M_ee_to_base);
////  Eigen::Matrix<double, 4*3, 1> grf_eigen_B = matrix_analyser.SolveLinearSystemOfEquations(des_force_body_B);
//  Eigen::MatrixXd base_M_ee_inv;
//  kindr::linear_algebra::pseudoInverse(base_M_ee_z, base_M_ee_inv, 1e-6);
//  Eigen::MatrixXd grf_eigen_B = base_M_ee_inv * des_force_body_B;
//
//
//  // pack into vector
//  std::array<Eigen::Vector3d, 4> grf_B;
//	int z_force = 0;
//	for (int ee=0; ee<kNumEE; ++ee) {
//	  if (is_ee_in_contact_start[ee]) {
//
//      // extract z force to full vector
//      Eigen::Vector3d f; f.setZero();
//      f(iit::rbd::Z) = grf_eigen_B(z_force++);
//
//      grf_B.at(ee) = f;
////	    grf_B.push_back(grf_eigen_B.segment<3>(3*ee));
//	    std::cout << "grf_B[" << ee << "]" << grf_B.at(ee).transpose() << std::endl;
//	  }
//	}
//
//	std::array<Eigen::Vector3d, kNumEE > pos_ee_b = integrator_.hyq_system_dyn_.hyq_kinematics_.getEEPosInBase(q);
//	decltype(id_)::ExtForces ext_forces = integrator_.hyq_system_dyn_.hyq_dynamics_.transformContactToEELinkForces(q,pos_ee_b,grf_B);
//
//	// solve the inverse dynamics problem to find initial torques
//	decltype(id_)::Acceleration trunk_a;
//	id_.id(tau, trunk_a, g_W, trunk_v, q, qd, qdd_des, ext_forces);
//	// double check if these external forces keep robot standing with zero acceleration
//  // std::cout << "trunk_a:" << trunk_a;
//
//	// torques to be optimized
//	for (int i=0; i< kInputNodesCount; ++i) {
//		for (int j=0; j<iit::HyQ::jointsCount; ++j) {
//			u[i*iit::HyQ::jointsCount + j] = tau[j];
//		}
//	}
//	// spacing of nodes to be optimized
//	for (int i=kInputNodesCount*iit::HyQ::jointsCount; i < n; ++i)
//	{
//	  double dt_equal = kTmaxStart/static_cast<double>(kInputNodesCount);
//	  u[i] = dt_equal;
//	}

	return true;
}

bool NlpIpoptZmp::eval_f(Index n, const Number* u, bool new_u, Number& obj_value)
{

//	double T = kTmaxStart; // x[n-1];
//	double obj_inter = 0.0;
//	double obj_final_pos = 0.0;
//	double obj_final_vel = 0.0;
//	double obj_input = 0.0;
//
//
//	ODEState y_final;
//	integrateODE(u, T, y_final);
//
////	// calculate costs based on this integration:
////	// Intermediate costs:
//	static const int joint_vel_start = 2*6 + iit::HyQ::jointsCount;
//	for (ODEState x : integrator_.ode_states_)
//	{
//		for (int i=joint_vel_start; i<joint_vel_start+iit::HyQ::jointsCount; ++i) {
//			obj_inter +=  (x(i)*x(i));              //vel
//		}
//	}
//
////  // penalize deviation from desired state
////  obj_final_pos += std::pow(y_final_des_[iit::rbd::AX] - y_final[iit::rbd::AX], 2);
////  obj_final_pos += std::pow(y_final_des_[iit::rbd::AY] - y_final[iit::rbd::AY], 2);
////  obj_final_pos += std::pow(y_final_des_[iit::rbd::AZ] - y_final[iit::rbd::AZ], 2);
////  obj_final_pos += std::pow(y_final_des_[iit::rbd::LX] - y_final[iit::rbd::LX], 2);
////  obj_final_pos += std::pow(y_final_des_[iit::rbd::LY] - y_final[iit::rbd::LY], 2);
////  obj_final_pos += std::pow(y_final_des_[iit::rbd::LZ] - y_final[iit::rbd::LZ], 2);
////  prt(obj_final_pos);
//
////  // penalize velocities different from zero
////  obj_final_vel += std::pow(y_final[6 + iit::HyQ::SAA], 2);
////  obj_final_vel += std::pow(y_final[6 + iit::HyQ::SFE], 2);
////  obj_final_vel += std::pow(y_final[6 + iit::HyQ::HR] , 2);
////  obj_final_vel += std::pow(y_final[6 + iit::HyQ::EFE], 2);
////  obj_final_vel += std::pow(y_final[6 + iit::HyQ::WR] , 2);
////  obj_final_vel += std::pow(y_final[6 + iit::HyQ::WFE], 2);
////  prt(obj_final_vel);
//
////  //  penalize control, e.g. regularization term
////  for (int i=0; i<n; ++i) {
////  	obj_input += std::pow(u[i], 2);
////  }
////  prt(obj_input);
//
//  obj_value = 0.0;
//  obj_value += obj_inter;
//  obj_value += obj_final_pos;
//  obj_value += obj_final_vel;
//  obj_value += obj_input;

  return true;
}


bool NlpIpoptZmp::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  // if nothing set, then this is automatically done by ipopt through finite
	// differences
	return true;
}


bool NlpIpoptZmp::eval_g(Index n, const Number* u, bool new_x, Index m, Number* g)
{
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
//
//	// say at which positions the nonzero elements of the jacobian are
//  if (values == NULL) {
//    // return the structure of the jacobian of the constraints - i.e. specify positions of non-zero elements.
//
//  	int c_nonzero = 0;
//  	for (int row=0; row<m; ++row) {
//  		for (int col=0; col<n; ++col)
//  		{
//  			iRow[c_nonzero] = row;
//  			jCol[c_nonzero] = col;
//  			c_nonzero++;
//  		}
//  	}
//  }
//  else {
//  // approximated by ipopt through finite differences
//  }
//
//  return true;
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
//  std::cout << "count = " << count_ << "\t delta count = " << count_ - count_prev_ << std::endl;
  count_prev_ = count_;
	return true;
}


//void MyNLPHyQ::integrateODE(const Number* u, double T, ODEState& y_final)
//{
//  count_++;
//	// assign the optimization variables to correct physical inputs (time + joints)
//	// used later by system dynamics derivative function
//	std::vector<ODEInput> control;
//	for (int i=0; i<kInputNodesCount; ++i) {
//		ODEInput input;
//		for (int j=0; j<iit::HyQ::jointsCount; ++j) {
//			input(j) = u[i*iit::HyQ::jointsCount + j];
//		}
//		control.push_back(input);
//	}
//
//
//	// lamda function that changes member function to odeint compliant function
//	// "captures/pull" "this" into local scope to allow to use in function
//	// also captures input to local scope to add to integrate
//	// u is only captured to get time between nodes
//	auto f = [this, T, control, u]( const ODEState& y, ODEState& dydt, double t)
//	{
//    // use current u for integration
//
//	  bool use_constant_inputs = true;
//
//	  int num_different_input_segments;
//	  if (use_constant_inputs)
//	    num_different_input_segments = kInputNodesCount;
//	  else
//	    num_different_input_segments = kInputNodesCount-1; // u1 <---> u2 <---> u3
//
//	  // TODO remember which way of interpolating the inputs is being used
//		static const double u_interval = T / num_different_input_segments;
//		int node = std::floor(t/u_interval);
//
//
//		// find input that is used at current time
////		int node;
////		for (int section=0; section<kInputNodesCount; ++section) {
////
////		  // sum up time of previous sections
////		  double t_end_current_section = 0;
////		  for (int i=0; i<=section; ++i) t_end_current_section+=u[kInputNodesCount*iit::HyQ::jointsCount+i];
////
////		  if (t < t_end_current_section ) {
////		    node = section;
////		    break;
////		  }
////		}
//
//
////		// linear function approximator for control
//		// somehow this made my optimization one sided
////		ODEInput prev = control[node];
////		ODEInput next = control[node+1];
////		double t_prev = node * u_interval;
////		double delta_t = t - t_prev;
////		ODEInput input = prev + delta_t * (next-prev)/u_interval;
//
//		// piecewise constant approximation of input
////     ODEInput input = control[0];
//     ODEInput	input = control[node];
//
////		std::array<bool,4> ee_in_contact = is_ee_in_contact_;
////		if (t > kTmaxStart/2.0) {
////		  ee_in_contact = {true, true, true, true};
////		}
//
//
////     std::array<bool,kNumEE> is_ee_in_contact = {false, false, false, false};
//     integrator_.hyq_system_dyn_.calcDerivative(y, input, integrator_.is_ee_in_contact_, kTIntegrationStep, dydt);
//	};
//
//	auto saveStates = [this]( const ODEState &y , const double t)
//	{
////	  std::cout << std::setprecision(2) << std::fixed;
////	  std::cout << "t="<< t << ":\n";
////	  std::cout << "base pos:\t" << y.segment(0,6).transpose() << "\n";
////	  std::cout << "base vel:\t" << y.segment(6,6).transpose() << "\n";
////	  std::cout << "joint pos:\t" << y.segment(2*6,12).transpose() << "\n";
////	  std::cout << "joint vel:\t" << y.segment(2*6+12,12).transpose() << "\n\n";
//		integrator_.ode_states_.push_back(y);
//	};
//
//
//  // sine the member variable is write protected
//	ODEState y_start = y_start_;
//
////	integrator_.ode_states_.clear();
////	boost::numeric::odeint::integrate_const(
////			stepper_,
////			f,
////			y_start,
////			0.0,
////			T,
////			kTIntegrationStep,
////			saveStates);
//
//	integrator_.IntegrateAndImpulse(y_start, f, T, kTIntegrationStep);
//
//  // joint velocities can be copied directly
//	y_final = y_start;
//
//  //Restrict position angles between between -pi and pi
//	// body orientation
//	for (int i=0; i<3; ++i)
//	{
//		y_final(i) = kindr::common::wrapPosNegPI(y_start(i));
//	}
//	// joint angles
//	for (int i=2*6; i<2*6 + iit::HyQ::jointsCount; ++i)
//	{
//		y_final(i) = kindr::common::wrapPosNegPI(y_start(i));
//	}
//
//}
//
//
void NlpIpoptZmp::finalize_solution(SolverReturn status,
                              Index n, const Number* u, const Number* z_L, const Number* z_U,
                              Index m, const Number* g, const Number* lambda,
                              Number obj_value,
			                        const IpoptData* ip_data,
			                        IpoptCalculatedQuantities* ip_cq)
{
//  // here is where we would store the solution to variables, or write to a file, etc
//  // so we could use the solution. Since the solution is displayed to the console,
//  // we currently do nothing here.
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


