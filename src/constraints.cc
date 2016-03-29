/*
 * constraints.cc
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/constraints.h>

namespace xpp {
namespace zmp {

Constraints::Constraints (const xpp::hyq::SuppTriangleContainer& supp_triangle_container,
                           const xpp::zmp::ContinuousSplineContainer& zmp_spline_container)
{
  supp_triangle_container_ = supp_triangle_container;
  zmp_spline_container_    = zmp_spline_container;

  // inequality constraints
  double walking_height = 0.58;
  x_zmp_ = zmp_spline_container.ExpressZmpThroughCoefficients(walking_height, xpp::utils::X);
  y_zmp_ = zmp_spline_container.ExpressZmpThroughCoefficients(walking_height, xpp::utils::Y);

  initial_footholds_ = supp_triangle_container.footholds_;
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
Constraints::EvalContraints(const Footholds& footholds, const Eigen::VectorXd& x_coeff)
{
  Eigen::VectorXd g_spline_junctions = EvalSplineJunctionConstraints(x_coeff);
  Eigen::VectorXd g_supp = EvalSuppPolygonConstraints(footholds, x_coeff);
  Eigen::VectorXd g_footholds = EvalFootholdConstraints(footholds);

  // combine all the g vectors
  // TODO: DRY, figure out how to remove these two rows
  Eigen::VectorXd g(g_supp.rows() + g_footholds.rows() + g_spline_junctions.rows());
  g << g_spline_junctions, g_supp, g_footholds;

  return g;
}


Eigen::VectorXd
Constraints::EvalSuppPolygonConstraints(const Footholds& footholds, const Eigen::VectorXd& x_coeff)
{;
  for (int i=0; i<footholds.size(); ++i) {
    supp_triangle_container_.footholds_.at(i).p.x() = footholds.at(i).x();
    supp_triangle_container_.footholds_.at(i).p.y() = footholds.at(i).y();
  }

  MatVec ineq = supp_triangle_container_.AddLineConstraints(x_zmp_, y_zmp_, zmp_spline_container_);
  return ineq.M*x_coeff + ineq.v;
}


Eigen::VectorXd
Constraints::EvalFootholdConstraints(const Footholds& footholds) const
{
  // constraints on the footsteps
  Eigen::VectorXd g_vec_footsteps(2*footholds.size());
  g_vec_footsteps.setZero();
  // fix footholds in x and y direction
  int c=0;
  for (uint i=0; i<footholds.size(); ++i) {

    xpp::hyq::Foothold f = initial_footholds_.at(i);

    int idx = 2*i;

    g_vec_footsteps(c++) = footholds.at(i).x() - f.p.x();
    g_vec_footsteps(c++) = footholds.at(i).y() - f.p.y();
  }


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

  return g_vec_footsteps;
}


Eigen::VectorXd
Constraints::EvalSplineJunctionConstraints(const Eigen::VectorXd& x_coeff) const
{
  return spline_function_constraints_.M*x_coeff + spline_function_constraints_.v;
}


} /* namespace zmp */
} /* namespace xpp */
