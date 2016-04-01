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
    :planned_footholds_(supp_triangle_container.footholds_)
{
  supp_polygon_container_ = supp_triangle_container;
  zmp_spline_container_    = zmp_spline_container;

  spline_junction_constraints_ = qp_equality_constraints;

  // inequality constraints
  double walking_height = 0.58;
  x_zmp_ = zmp_spline_container.ExpressZmpThroughCoefficients(walking_height, xpp::utils::X);
  y_zmp_ = zmp_spline_container.ExpressZmpThroughCoefficients(walking_height, xpp::utils::Y);
}


Eigen::VectorXd
Constraints::EvalContraints(const Footholds& footholds, const Eigen::VectorXd& x_coeff)
{
  std::vector<Eigen::VectorXd> g_vec;

  g_vec.push_back(EvalSplineJunctionConstraints(x_coeff));
  g_vec.push_back(EvalSuppPolygonConstraints(footholds, x_coeff));
  g_vec.push_back(EvalFootholdConstraints(footholds));
//  g_vec.push_back(EvalStepLengthConstraints(footholds, bounds_));


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
    g_.middleRows(c, g.rows()) = g; //g.normalized()
    c += g.rows();
  }


  first_constraint_eval_ = false;
  return g_;
}


Eigen::VectorXd
Constraints::EvalSuppPolygonConstraints(const Footholds& footholds, const Eigen::VectorXd& x_coeff)
{
  for (uint i=0; i<footholds.size(); ++i) {
    supp_polygon_container_.footholds_.at(i).p.x() = footholds.at(i).x();
    supp_polygon_container_.footholds_.at(i).p.y() = footholds.at(i).y();
  }


  MatVec ineq = supp_polygon_container_.AddLineConstraints(x_zmp_, y_zmp_, zmp_spline_container_);


  AddBounds(ineq.v.rows(), 0.0, +1.0e19);

  return ineq.M*x_coeff + ineq.v;
}


Eigen::VectorXd
Constraints::EvalFootholdConstraints(const Footholds& footholds)
{
  // constraints on the footsteps
  Eigen::VectorXd g(2*footholds.size());
  g.setZero();
  int c=0;
  for (uint i=0; i<footholds.size(); ++i) {
    xpp::hyq::Foothold f = planned_footholds_.at(i);

    // fix footholds in x and y direction
    g(c++) = footholds.at(i).x() - f.p.x();
    g(c++) = footholds.at(i).y() - f.p.y();
  }


  AddBounds(g.rows(), 0.0, 0.0);

  return g;
}


Eigen::VectorXd
Constraints::EvalStepLengthConstraints(const Footholds& footholds)
{
  Eigen::VectorXd g(footholds.size());

  int c=0;
  for (uint i=0; i<footholds.size(); ++i)
  {
    xpp::hyq::LegID leg = planned_footholds_.at(i).leg; // leg sequence stays the same as initial
    Eigen::Vector2d f_prev = supp_polygon_container_.start_stance_[leg].p.segment<2>(0);
    if (i>=4) {
      f_prev = footholds.at(i-4); // FIXME: only works for repeating same step sequence
    }

    double dx = footholds.at(i).x() - f_prev.x();
    double dy = footholds.at(i).y() - f_prev.y();

    g(c++) = hypot(dx,dy);
    // this seems to converge better than the combination of both
//    g(c++) = dx*dx;
//    g(c++) = dy*dy;
  }


  // add bounds that step length should never exceed max step length
  double max_step_length = 0.3;
  AddBounds(g.rows(), 0.0, max_step_length);

  return g;
}


Eigen::VectorXd
Constraints::EvalWorkspaceConstraints(const Eigen::VectorXd& x_coeff,
                                         const Footholds& footholds)
{
  Eigen::VectorXd g(10);

//   get position of cog through xpline coefficients
  zmp_spline_container_.AddOptimizedCoefficients(x_coeff);
  xpp::utils::Point2d cog_xy;
  double t_global = 0.2; //s
  zmp_spline_container_.GetCOGxy(t_global, cog_xy);


  // calculate distance cog to foothold for each leg
  return g;
}


Eigen::VectorXd
Constraints::EvalSplineJunctionConstraints(const Eigen::VectorXd& x_coeff)
{
  Eigen::VectorXd g = spline_junction_constraints_.M*x_coeff + spline_junction_constraints_.v;


  AddBounds(g.rows(), 0.0, 0.0);
  return g;
}


void Constraints::AddBounds(int m_constraints, double lower, double upper)
{
  if (first_constraint_eval_) {
    Bound eq_bound(lower, upper);
    for (int c=0; c<m_constraints; ++c) {
      bounds_.push_back(eq_bound);
    }
  }
}


} /* namespace zmp */
} /* namespace xpp */
