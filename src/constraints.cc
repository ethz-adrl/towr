/*
 * constraints.cc
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/constraints.h>

namespace xpp {
namespace zmp {

Constraints::Constraints ()
{
  // TODO Auto-generated constructor stub

}

Constraints::~Constraints ()
{
  // TODO Auto-generated destructor stub
}


std::vector<Constraints::Bound>
Constraints::GetBounds() const
{
  const Bound ineq_bound(0.0, +1.0e19);
  const Bound eq_bound(0.0, 0.0);
  std::vector<Constraints::Bound> bounds;

  for (int i=0; i<n_equality_constraints_; ++i)
    bounds.push_back(eq_bound);

  for (int i=0; i<n_inequality_constraints_; ++i)
    bounds.push_back(ineq_bound);

  return bounds;
}


Eigen::VectorXd
Constraints::EvalContraints(const Eigen::VectorXd& x) const
{
//  // equality constraints
//  Eigen::VectorXd g_vec_eq = eq_.M*x_coeff_ + eq_.v;
//
//
//  // inequality constraints
//  std::vector<xpp::hyq::Foothold> steps;
//  for (int i=0; i<n_steps_; ++i) {
//    steps.push_back(xpp::hyq::Foothold(x_footholds_[2*i+X],
//                                       x_footholds_[2*i+Y],
//                                       0.0,
//                                       initial_footholds_.at(i).leg));
//  }
//
//
//  //here i am adapting the constraints depending on the footholds
//  supp_triangle_container_.footholds_ = steps;
//  ineq_ = supp_triangle_container_.AddLineConstraints(x_zmp_, y_zmp_, zmp_spline_container_);
//  Eigen::VectorXd g_vec_in = ineq_.M*x_coeff_ + ineq_.v;
//
//
//
//
//
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
//    g_vec_footsteps(c++) = x_footholds_[idx+X] - f.p.x();
//    g_vec_footsteps(c++) = x_footholds_[idx+Y] - f.p.y();
//  }


  return x;
}





} /* namespace zmp */
} /* namespace xpp */
