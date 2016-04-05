/*
 * constraints.cc
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/constraints.h>

namespace xpp {
namespace zmp {

Constraints::Constraints (const xpp::hyq::SupportPolygonContainer& supp_poly_container,
                           const xpp::zmp::ContinuousSplineContainer& zmp_spline_container,
                           const MatVec& qp_equality_constraints)
    :planned_footholds_(supp_poly_container.GetFootholds()),
     zmp_constraint_(zmp_spline_container)
{
  zmp_spline_container_    = zmp_spline_container;
  supp_polygon_container_  = supp_poly_container;

  spline_junction_constraints_ = qp_equality_constraints;

  // inequality constraints
  double walking_height = 0.58;
  x_zmp_ = zmp_spline_container.ExpressZmpThroughCoefficients(walking_height, xpp::utils::X);
  y_zmp_ = zmp_spline_container.ExpressZmpThroughCoefficients(walking_height, xpp::utils::Y);

  first_constraint_eval_ = true;
}


Eigen::VectorXd
Constraints::EvalContraints(const Eigen::VectorXd& x_coeff, const StdVecEigen2d& footholds)
{
  std::vector<Eigen::VectorXd> g_std;

  // update the member variables
  zmp_spline_container_.AddOptimizedCoefficients(x_coeff);
  for (uint i=0; i<footholds.size(); ++i)
    supp_polygon_container_.SetFootholdsXY(i,footholds.at(i).x(), footholds.at(i).y());

  // generate constraint violation values
  g_std.push_back(FixFootholdPosition(footholds));
  g_std.push_back(KeepZmpInSuppPolygon(x_coeff));
//  g_std.push_back(RestrictFootholdToCogPos(x_coeff));
  g_std.push_back(SmoothAccJerkAtSplineJunctions(x_coeff)); // FIXME extract intial and final constraints
//  g_std.push_back(RestrictMaxStepLength(footholds));


  CombineToEigenVector(g_std, g_);


  first_constraint_eval_ = false;
  return g_;
}


Eigen::VectorXd
Constraints::KeepZmpInSuppPolygon(const Eigen::VectorXd& x_coeff)
{
  MatVec ineq = zmp_constraint_.AddLineConstraints(x_zmp_, y_zmp_, supp_polygon_container_);

  AddBounds(ineq.v.rows(), 0.0, +1.0e19);

  return ineq.M*x_coeff + ineq.v;
}


Eigen::VectorXd
Constraints::FixFootholdPosition(const StdVecEigen2d& footholds)
{
  // constraints on the footsteps
  std::vector<int> fixed_dim = {xpp::utils::X, xpp::utils::Y};
  Eigen::VectorXd g(fixed_dim.size()*footholds.size());
  int c=0;

  for (uint i=0; i<footholds.size(); ++i) {
    xpp::hyq::Foothold f = planned_footholds_.at(i);

    // fix footholds in x and y direction
    for (int dim : fixed_dim)
      g(c++) = footholds.at(i)(dim) - f.p(dim);
  }


  AddBounds(g.rows(), 0.0, 0.0);

  return g;
}


Eigen::VectorXd
Constraints::RestrictMaxStepLength(const StdVecEigen2d& footholds)
{
  Eigen::VectorXd g(footholds.size());
  int c=0;

  for (uint i=0; i<footholds.size(); ++i)
  {
    xpp::hyq::LegID leg = planned_footholds_.at(i).leg; // leg sequence stays the same as initial
    Eigen::Vector2d f_prev = supp_polygon_container_.GetStartStance()[leg].p.segment<2>(0);
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
Constraints::RestrictFootholdToCogPos(const Eigen::VectorXd& x_coeff)
{
  double dt = 0.2; //zmp_spline_container_.dt_;
  double T  = zmp_spline_container_.GetTotalTime();
  int N     = std::ceil(T/dt);

  int n_constraints = 3*N*2;
  Eigen::VectorXd g(n_constraints); // 3 legs in contact at every discrete time, 2 b/c x-y
  int c=0;

  double t=0.0;
  do {
    // know legs in contact at each step
    int step = zmp_spline_container_.GetStep(t);
    VecFoothold stance_legs = supp_polygon_container_.GetStanceLegs(step);

    xpp::utils::Point2d cog_xy;
    zmp_spline_container_.GetCOGxy(t, cog_xy);

    // calculate distance to base for every stance leg
    // restrict to quadrants
    for (const hyq::Foothold& f : stance_legs) {
      double dx = f.p.x() - cog_xy.p.x();
      double dy = f.p.y() - cog_xy.p.y();

//      g(c++) = hypot(dx,dy);
      // restrict to quadrandts
      g(c++) = dx;
      g(c++) = dy;
//      g(c++) = std::fabs(dx);
//      g(c++) = std::fabs(dy);

      if (first_constraint_eval_) {
        double max_range = 0.4;
        Bound bound_pos(0.0, max_range);
        Bound bound_neg(-max_range, 0.0);
        switch (f.leg) {
          case hyq::LF:
            bounds_.push_back(bound_pos); // x
            bounds_.push_back(bound_pos); // y
            break;
          case hyq::RF:
            bounds_.push_back(bound_pos); // x
            bounds_.push_back(bound_neg); // y
            break;
          case hyq::LH:
            bounds_.push_back(bound_neg); // x
            bounds_.push_back(bound_pos); // y
            break;
          case hyq::RH:
            bounds_.push_back(bound_neg); // x
            bounds_.push_back(bound_neg); // y
            break;
          default:
            break;
        }
      }

    }
    t += dt;

  } while(t < T);


  assert(n_constraints == c);
  // add bounds that foot position should never be to far away from body
//  double max_range = 0.4;
//  AddBounds(g.rows(), 0.0, max_range);

  return g;
}


Eigen::VectorXd
Constraints::SmoothAccJerkAtSplineJunctions(const Eigen::VectorXd& x_coeff)
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


void
Constraints::CombineToEigenVector(const std::vector<Eigen::VectorXd>& g_std, Eigen::VectorXd& g_eig) const
{
  // create correct size constraint vector the first time this function is called
  if (first_constraint_eval_) {
    int n_constraints = 0;
    for (const Eigen::VectorXd& g : g_std) {
      n_constraints += g.rows();
    }
    g_eig.resize(n_constraints);
  }

  //  combine all the g vectors
  //  g_ << g_vec[0], g_vec[1], g_vec[2];
  int c = 0;
  for (const Eigen::VectorXd& g : g_std) {
    g_eig.middleRows(c, g.rows()) = g; //g.normalized()
    c += g.rows();
  }
}


} /* namespace zmp */
} /* namespace xpp */
