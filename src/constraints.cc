/*
 * constraints.cc
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/constraints.h>

namespace xpp {
namespace zmp {

Constraints::Constraints (const xpp::hyq::SupportPolygonContainer& supp_triangle_container,
                           const xpp::zmp::ContinuousSplineContainer& zmp_spline_container,
                           const MatVec& qp_equality_constraints)
{
  supp_triangle_container_ = supp_triangle_container;
  zmp_spline_container_    = zmp_spline_container;

  spline_junction_constraints_ = qp_equality_constraints;

  // inequality constraints
  double walking_height = 0.58;
  x_zmp_ = zmp_spline_container.ExpressZmpThroughCoefficients(walking_height, xpp::utils::X);
  y_zmp_ = zmp_spline_container.ExpressZmpThroughCoefficients(walking_height, xpp::utils::Y);

  initial_footholds_ = supp_triangle_container.footholds_;
}


Eigen::VectorXd
Constraints::EvalContraints(const Footholds& footholds, const Eigen::VectorXd& x_coeff)
{
  std::vector<Eigen::VectorXd> g_vec;

  g_vec.push_back(EvalSplineJunctionConstraints(x_coeff, bounds_));
  g_vec.push_back(EvalSuppPolygonConstraints(footholds, x_coeff, bounds_));
//  g_vec.push_back(EvalFootholdConstraints(footholds, bounds_));
  g_vec.push_back(EvalStepLengthConstraints(footholds, bounds_));


  // create correct size constraint vector the first time this function is called
  if (first_constraint_eval_) {
    int n_constraints = 0;
    for (Eigen::VectorXd& g : g_vec) {
      n_constraints += g.rows();
    }
    g_.resize(n_constraints);
  }


  //  combine all the g vectors
  //  g_ << g_vec[0], g_vec[1], g_vec[2];
  int c = 0;
  for (Eigen::VectorXd& g : g_vec) {
    g_.middleRows(c, g.rows()) = g;
    c += g.rows();
  }


  first_constraint_eval_ = false;
  return g_;
}


Eigen::VectorXd
Constraints::EvalSuppPolygonConstraints(const Footholds& footholds, const Eigen::VectorXd& x_coeff,
                                        std::vector<Constraints::Bound>& bounds)
{
  for (int i=0; i<footholds.size(); ++i) {
    supp_triangle_container_.footholds_.at(i).p.x() = footholds.at(i).x();
    supp_triangle_container_.footholds_.at(i).p.y() = footholds.at(i).y();
  }


  MatVec ineq = supp_triangle_container_.AddLineConstraints(x_zmp_, y_zmp_, zmp_spline_container_);

  // add bounds
  if (first_constraint_eval_) {
    Bound ineq_bound(0.0, +1.0e19);
    for (int c=0; c<ineq.v.rows(); ++c) {
      bounds.push_back(ineq_bound);
    }
  }

  return ineq.M*x_coeff + ineq.v;
}


Eigen::VectorXd
Constraints::EvalFootholdConstraints(const Footholds& footholds,
                                     std::vector<Constraints::Bound>& bounds) const
{
  // constraints on the footsteps
  Eigen::VectorXd g(2*footholds.size());
  g.setZero();
  int c=0;
  for (uint i=0; i<footholds.size(); ++i) {
    xpp::hyq::Foothold f = initial_footholds_.at(i);

    // fix footholds in x and y direction
    g(c++) = footholds.at(i).x() - f.p.x();
    g(c++) = footholds.at(i).y() - f.p.y();
  }


  if (first_constraint_eval_) {
    Bound eq_bound(0.0, 0.0);
    for (int c=0; c<g.rows(); ++c) {
      bounds.push_back(eq_bound);
    }
  }

  return g;
}


Eigen::VectorXd
Constraints::EvalStepLengthConstraints(const Footholds& footholds,
                                     std::vector<Constraints::Bound>& bounds) const
{
  Eigen::VectorXd g(2*footholds.size());

  int c=0;
  for (uint i=0; i<footholds.size(); ++i)
  {
    xpp::hyq::LegID leg = initial_footholds_.at(i).leg; // leg sequence stays the same as initial
    Eigen::Vector2d f_prev = supp_triangle_container_.start_stance_[leg].p.segment<2>(0);
    if (i>=4) {
      f_prev = footholds.at(i-4); // FIXME: only works for repeating same step sequence
    }

    double dx = footholds.at(i).x() - f_prev.x();
    double dy = footholds.at(i).y() - f_prev.y();

//    g(i) = hypot(dx,dy);
    // this seems to converge better than the combination of both
    g(c++) = dx*dx;
    g(c++) = dy*dy;
  }


  // add bounds that step length should never exceed max step length
  double max_step_length = 0.25;
  if (first_constraint_eval_) {
    Bound ineq_bound(0.0, std::pow(max_step_length,2));
    for (int c=0; c<g.rows(); ++c) {
      bounds.push_back(ineq_bound);
    }
  }

  return g;
}


Eigen::VectorXd
Constraints::EvalSplineJunctionConstraints(const Eigen::VectorXd& x_coeff,
                                           std::vector<Constraints::Bound>& bounds) const
{
  Eigen::VectorXd g = spline_junction_constraints_.M*x_coeff + spline_junction_constraints_.v;

  if (first_constraint_eval_) {
    Bound eq_bound(0.0, 0.0);
    for (int c=0; c<g.rows(); ++c) {
      bounds.push_back(eq_bound);
    }
  }

  return g;
}


} /* namespace zmp */
} /* namespace xpp */
