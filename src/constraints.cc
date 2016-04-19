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


Constraints::VectorXd
Constraints::EvalContraints(const VectorXd& x_coeff, const StdVecEigen2d& footholds)
{
  UpdateCurrentState(x_coeff, footholds);
  std::vector<Constraint> g_std = GetConstraintsOnly(x_coeff, footholds);
  CombineToEigenVector(g_std, constraints_);

  return constraints_;
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
  g_std.push_back(KeepZmpInSuppPolygon(x_coeff, supp_polygon_container_));
  g_std.push_back(FixFootholdPosition(footholds));
//  g_std.push_back(RestrictFootholdToCogPos());
//  g_std.push_back(AddObstacle(footholds));


  return g_std;
}


Constraints::Constraint
Constraints::KeepZmpInSuppPolygon(const VectorXd& x_coeff,
                                  const SupportPolygonContainer& support_polygon_container) const
{
  MatVec ineq = zmp_constraint_.CreateLineConstraints(support_polygon_container);

  Constraint constraints;
  constraints.values_ = ineq.M*x_coeff + ineq.v;
  constraints.type_ = INEQUALITY;

  return constraints;
}


Constraints::Constraint
Constraints::FixFootholdPosition(const StdVecEigen2d& footholds) const
{
  Constraint constraints;
  constraints.values_ = DistanceFootFromPlanned(footholds);
  constraints.type_ = EQUALITY;

  return constraints;
}


Constraints::Constraint
Constraints::AddObstacle(const StdVecEigen2d& footholds) const
{
  Constraint constraints;
  constraints.values_ = DistanceSquareFootToGapboarder(footholds, gap_center_x_, gap_width_x_);
  constraints.type_ = INEQUALITY;
  return constraints;
}


Constraints::Constraint
Constraints::RestrictFootholdToCogPos() const
{
  Constraint c;
  c.values_ = DistanceFootToNominalStance();
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
