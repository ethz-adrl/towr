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

}


std::vector<Constraints::Bound>
Constraints::GetBounds()
{
  std::vector<Constraints::Bound> bounds;

  // non initialized values just to compute the bounds
  Eigen::VectorXd x_coeff(zmp_spline_container_.GetTotalFreeCoeff());
  StdVecEigen2d x_footholds(supp_polygon_container_.GetNumberOfSteps());

  std::vector<Constraint> g_std = GetConstraintsOnly(x_coeff, x_footholds);

  for (const Constraint& g : g_std) {
    AddBounds(g.values_.rows(), g.type_, bounds);
  }

  constraints_.resize(bounds.size());
  return bounds;
}



void
Constraints::AddBounds(int m_constraints, ConstraintType type,
                       std::vector<Constraints::Bound>& bounds) const
{
  static const std::map<ConstraintType, Bound> bound_types {
    {EQUALITY, Bound(0.0, 0.0)},
    {INEQUALITY, Bound(0.0, 1.0e19)},
    {COGTOFOOTHOLD, Bound(-0.15, 0.15)}
  };

  for (int c=0; c<m_constraints; ++c) {
    bounds.push_back(bound_types.at(type));
  }
}


std::vector<Constraints::Constraint>
Constraints::GetConstraintsOnly(const VectorXd& x_coeff,
                                const StdVecEigen2d& footholds) const
{
  std::vector<Constraint> g_std;

  // generate constraint violation values
  // ATTENTION: order seems to play a role
  g_std.push_back(FinalState(x_coeff));
//  g_std.push_back(InitialAcceleration(x_coeff));
  g_std.push_back(SmoothAccJerkAtSplineJunctions(x_coeff));
  g_std.push_back(KeepZmpInSuppPolygon(x_coeff));
//  g_std.push_back(FixFootholdPosition(footholds));
  g_std.push_back(RestrictFootholdToCogPos(x_coeff));
//  g_std.push_back(AddObstacle());


  return g_std;
}



Constraints::VectorXd
Constraints::EvalContraints(const VectorXd& x_coeff, const StdVecEigen2d& footholds)
{

  // update the member variables
  zmp_spline_container_.AddOptimizedCoefficients(x_coeff);
  for (uint i=0; i<footholds.size(); ++i)
    supp_polygon_container_.SetFootholdsXY(i,footholds.at(i).x(), footholds.at(i).y());


  std::vector<Constraint> g_std = GetConstraintsOnly(x_coeff, footholds);


  CombineToEigenVector(g_std, constraints_);

  return constraints_;
}


Constraints::Constraint
Constraints::KeepZmpInSuppPolygon(const VectorXd& x_coeff) const
{

  MatVec ineq = zmp_constraint_.CreateLineConstraints(supp_polygon_container_);

  Constraint constraints;
  constraints.values_ = ineq.M*x_coeff + ineq.v;
  constraints.type_ = INEQUALITY;

  return constraints;
}


Constraints::Constraint
Constraints::FixFootholdPosition(const StdVecEigen2d& footholds) const
{
  // constraints on the footsteps
  std::vector<int> fixed_dim = {xpp::utils::X, xpp::utils::Y};
  VectorXd g(fixed_dim.size()*footholds.size());
  int c=0;

  for (uint i=0; i<footholds.size(); ++i) {
    xpp::hyq::Foothold f = planned_footholds_.at(i);

    // fix footholds in x and y direction
    for (int dim : fixed_dim)
      g(c++) = footholds.at(i)(dim) - f.p(dim);
  }


  Constraint constraints;
  constraints.values_ = g;
  constraints.type_ = EQUALITY;

  return constraints;
}


Constraints::Constraint
Constraints::AddObstacle() const
{
  std::vector<double> g_vec;

//  xpp::utils::Ellipse ellipse(gap_depth, gap_width, x_center, y_center);

  for (const hyq::Foothold& f : supp_polygon_container_.GetFootholds())
  {
//    g_vec.push_back(ellipse.DistanceToEdge(f.p.x(), f.p.y()));
    double foot_to_center = f.p.x()-gap_center_x_;
    double minimum_to_center = gap_width_x_/2.0;
    double cost = std::pow(foot_to_center,2) - std::pow(minimum_to_center,2);
    g_vec.push_back(cost);
  }

  Constraint constraints;
  constraints.values_ = Eigen::Map<const VectorXd>(&g_vec.front(),g_vec.size());
  constraints.type_ = INEQUALITY;
  return constraints;
}



Constraints::Constraint
Constraints::RestrictFootholdToCogPos(const VectorXd& x_coeff) const
{
  double x_nominal_b = 0.3; // 0.4
  double y_nominal_b = 0.3; // 0.4
  double x_radius = 0.15;
  double y_radius = 0.10;

  Eigen::Vector2d r_BLF; r_BLF << x_nominal_b, y_nominal_b;
  Eigen::Vector2d r_BRF; r_BRF << x_nominal_b, -y_nominal_b;
  Eigen::Vector2d r_BLH; r_BLH << -x_nominal_b, y_nominal_b;
  Eigen::Vector2d r_BRH; r_BRH << -x_nominal_b, -y_nominal_b;

//
//
//  Bound bound_pos_x( x_nominal_b-x_radius,  x_nominal_b+x_radius);
//  Bound bound_neg_x(-x_nominal_b-x_radius, -x_nominal_b+x_radius);
//
//  Bound bound_pos_y( y_nominal_b-y_radius,  y_nominal_b+y_radius);
//  Bound bound_neg_y(-y_nominal_b-y_radius, -y_nominal_b+y_radius);

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

      // vector base to foot
      Eigen::Vector2d r_BF = f.p.segment<2>(0) - cog_xy.p;

      //      double dx = f.p.x() - cog_xy.p.x();
      //      double dy = f.p.y() - cog_xy.p.y();


      Eigen::Vector2d r_FC;
      // kinematic constraints (slightly ugly)
      switch (f.leg) {
        case hyq::LF:
          r_FC = -r_BF + r_BLF;

          break;
        case hyq::RF:
          r_FC = -r_BF + r_BRF;

          break;
        case hyq::LH:
          r_FC = -r_BF + r_BLH;

          break;
        case hyq::RH:
          r_FC = -r_BF + r_BRH;

          break;
        default:
          throw std::runtime_error("RestrictFootholdToCogPos(): leg does not exist");
          break;
      }


      g_vec.push_back(r_FC.x());
      g_vec.push_back(r_FC.y());
    }
    t += dt;

  } while(t < T);

  Constraint c;
  c.values_ = Eigen::Map<const VectorXd>(&g_vec.front(),g_vec.size());
  c.type_ = COGTOFOOTHOLD;
  return c;
}


Constraints::Constraint
Constraints::SmoothAccJerkAtSplineJunctions(const VectorXd& x_coeff) const
{
  Constraint c;
  c.values_ = spline_junction_constraints_.M*x_coeff + spline_junction_constraints_.v;
  c.type_ = EQUALITY;
  return c;
}


Constraints::Constraint
Constraints::InitialAcceleration(const VectorXd& x_coeff) const
{
  Constraint c;
  c.values_ = spline_initial_acc_constraints_.M*x_coeff + spline_initial_acc_constraints_.v;
  c.type_ = EQUALITY;
  return c;
}


Constraints::Constraint
Constraints::FinalState(const VectorXd& x_coeff) const
{
  Constraint c;
  c.values_ = spline_final_constraints_.M*x_coeff + spline_final_constraints_.v;
  c.type_ = EQUALITY;
  return c;
}


void
Constraints::CombineToEigenVector(const std::vector<Constraint>& g_std, VectorXd& g_eig) const
{
  //  combine all the g vectors
  //  g_ << g_vec[0], g_vec[1], g_vec[2];
  int c = 0;
  for (const Constraint& g : g_std) {
    g_eig.middleRows(c, g.values_.rows()) = g.values_; //g.normalized()
    c += g.values_.rows();
  }
}


} /* namespace zmp */
} /* namespace xpp */
