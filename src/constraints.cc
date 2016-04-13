/*
 * constraints.cc
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/constraints.h>
#include <xpp/zmp/spline_constraints.h>

#define prt(x) std::cout << #x << " = " << x << std::endl;
//#define prt(x)

namespace xpp {
namespace zmp {

Constraints::Constraints (const xpp::hyq::SupportPolygonContainer& supp_poly_container,
                          const xpp::zmp::ContinuousSplineContainer& zmp_spline_container,
                          double walking_height)
    :ProblemSpecification(supp_poly_container, zmp_spline_container),
     zmp_constraint_(zmp_spline_container, walking_height)
{
  State final_state; // zero vel,acc,jerk
  final_state.p = supp_poly_container.GetCenterOfFinalStance();
  SplineConstraints spline_constraint(zmp_spline_container);
  spline_junction_constraints_    = spline_constraint.CreateJunctionConstraints();
  spline_initial_acc_constraints_ = spline_constraint.CreateInitialAccConstraints();
  spline_final_constraints_       = spline_constraint.CreateFinalConstraints(final_state);

  first_constraint_eval_ = true;
  bounds_.clear();
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
  // ATTENTION: order seems to play a role
  g_std.push_back(FinalState(x_coeff));
//  g_std.push_back(InitialAcceleration(x_coeff));
  g_std.push_back(SmoothAccJerkAtSplineJunctions(x_coeff));
  g_std.push_back(KeepZmpInSuppPolygon(x_coeff));
//  g_std.push_back(FixFootholdPosition(footholds));
  g_std.push_back(RestrictFootholdToCogPos(x_coeff));
//  g_std.push_back(AddObstacle());




  CombineToEigenVector(g_std, constraints_);

  assert(constraints_.rows() == bounds_.size());
  first_constraint_eval_ = false;
  return constraints_;
}


Eigen::VectorXd
Constraints::KeepZmpInSuppPolygon(const Eigen::VectorXd& x_coeff)
{
  MatVec ineq = zmp_constraint_.CreateLineConstraints(supp_polygon_container_);

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
Constraints::AddObstacle()
{
  std::vector<double> g_vec;

//  xpp::utils::Ellipse ellipse(gap_depth, gap_width, x_center, y_center);

  for (const hyq::Foothold& f : supp_polygon_container_.GetFootholds())
  {
//    g_vec.push_back(ellipse.DistanceToEdge(f.p.x(), f.p.y()));
    g_vec.push_back(std::pow(f.p.x()-gap_center_x_,2));

    if (first_constraint_eval_) {
      bounds_.push_back(Bound(std::pow(gap_width_x_/2.0,2), 1.0e19)); // be outside of this ellipse
    }
  }
  return Eigen::Map<const Eigen::VectorXd>(&g_vec.front(),g_vec.size());
}



Eigen::VectorXd
Constraints::RestrictFootholdToCogPos(const Eigen::VectorXd& x_coeff)
{
  double x_nominal_b = 0.3; // 0.4
  double y_nominal_b = 0.3; // 0.4
  double x_radius = 0.15;
  double y_radius = 0.10;

  Bound bound_pos_x( x_nominal_b-x_radius,  x_nominal_b+x_radius);
  Bound bound_neg_x(-x_nominal_b-x_radius, -x_nominal_b+x_radius);

  Bound bound_pos_y( y_nominal_b-y_radius,  y_nominal_b+y_radius);
  Bound bound_neg_y(-y_nominal_b-y_radius, -y_nominal_b+y_radius);

//  double min_range = 0.1;
  double dt = 0.3; //zmp_spline_container_.dt_;
  double T  = zmp_spline_container_.GetTotalTime();
  int N     = std::ceil(T/dt);
  int approx_n_constraints = 4*N*2; // 3 or 4 legs in contact at every discrete time, 2 b/c x-y
  std::vector<double> g_vec;
  g_vec.reserve(approx_n_constraints);

//  Eigen::VectorXd g(approx_n_constraints);
//  int c=0;

  double t=0.0;
  do {
    // know legs in contact at each step
    VecFoothold stance_legs;
    int step = zmp_spline_container_.GetStep(t);
    stance_legs = supp_polygon_container_.GetStanceDuring(step);



    xpp::utils::Point2d cog_xy;
    zmp_spline_container_.GetCOGxy(t, cog_xy);

    // calculate distance to base for every stance leg
    // restrict to quadrants
    for (const hyq::Foothold& f : stance_legs) {
      double dx = f.p.x() - cog_xy.p.x();
      double dy = f.p.y() - cog_xy.p.y();

//      g_vec.push_back(hypot(dx,dy));
      // restrict to quadrandts
      g_vec.push_back(dx);
      g_vec.push_back(dy);
//      g(c++) = std::fabs(dx);
//      g(c++) = std::fabs(dy);

      if (first_constraint_eval_) {
        // kinematic constraints (slightly ugly)
        switch (f.leg) {
          case hyq::LF:
            bounds_.push_back(bound_pos_x); // x
            bounds_.push_back(bound_pos_y); // y
            break;
          case hyq::RF:
            bounds_.push_back(bound_pos_x); // x
            bounds_.push_back(bound_neg_y); // y
            break;
          case hyq::LH:
            bounds_.push_back(bound_neg_x); // x
            bounds_.push_back(bound_pos_y); // y
            break;
          case hyq::RH:
            bounds_.push_back(bound_neg_x); // x
            bounds_.push_back(bound_neg_y); // y
            break;
          default:
            throw std::runtime_error("RestrictFootholdToCogPos(): leg does not exist");
            break;
        }
      }

    }
    t += dt;

  } while(t < T);



//  g.resize(c);
//  assert(n_constraints == c); // FIXME, put this back in
  // add bounds that foot position should never be to far away from body
//  double max_range = 0.4;
//  AddBounds(g.rows(), 0.0, max_range);

//  Eigen::Map<const Eigen::VectorXd>(&g_vec.front(),g_vec.size());


  return Eigen::Map<const Eigen::VectorXd>(&g_vec.front(),g_vec.size());
}


Eigen::VectorXd
Constraints::SmoothAccJerkAtSplineJunctions(const Eigen::VectorXd& x_coeff)
{
  Eigen::VectorXd g = spline_junction_constraints_.M*x_coeff + spline_junction_constraints_.v;
  AddBounds(g.rows(), 0.0, 0.0);
  return g;
}


Eigen::VectorXd
Constraints::InitialAcceleration(const Eigen::VectorXd& x_coeff)
{
  Eigen::VectorXd g = spline_initial_acc_constraints_.M*x_coeff + spline_initial_acc_constraints_.v;
  AddBounds(g.rows(), 0.0, 0.0);
  return g;
}


Eigen::VectorXd
Constraints::FinalState(const Eigen::VectorXd& x_coeff)
{
  Eigen::VectorXd g = spline_final_constraints_.M*x_coeff + spline_final_constraints_.v;
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
