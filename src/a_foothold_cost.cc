/**
 @file    a_foothold_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 8, 2016
 @brief   Defines an abstract FootholdCost class and one concrete derivation.
 */

#include <xpp/opt/a_foothold_cost.h>
#include <xpp/opt/optimization_variables.h>

namespace xpp {
namespace opt {

AFootholdCost::AFootholdCost ()
{
  // TODO Auto-generated constructor stub
}

AFootholdCost::~AFootholdCost ()
{
  // TODO Auto-generated destructor stub
}

void
AFootholdCost::Init (const SupportPolygonContainer& supp_polygon_container)
{
  supp_polygon_container_ = supp_polygon_container;
}

void
AFootholdCost::UpdateVariables (const OptimizationVariables* opt_var)
{
  Eigen::VectorXd footholds = opt_var->GetVariables(VariableNames::kFootholds);
  supp_polygon_container_.SetFootholdsXY(utils::ConvertEigToStd(footholds));
}

FootholdGoalCost::FootholdGoalCost (const Vector2d& goal_xy,
                                    const SupportPolygonContainer& supp_poly)
{
  Init(supp_poly);
  goal_xy_ = goal_xy;
}

FootholdGoalCost::~FootholdGoalCost ()
{
}

FootholdGoalCost::VectorXd
FootholdGoalCost::EvaluateConstraint () const
{
  VectorXd g;
  Vector2d center_final_stance_W = supp_polygon_container_.GetCenterOfFinalStance();
  Vector2d distance_to_center = goal_xy_ - center_final_stance_W;
  g = distance_to_center.transpose() * distance_to_center;
  return g; // so far 1-dimensional, so scalar
}

FootholdGoalCost::Jacobian
FootholdGoalCost::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac;
  using namespace xpp::utils;

  if (var_set == VariableNames::kFootholds) {

    int n_foothold_opt_vars = supp_polygon_container_.GetTotalFreeCoeff();
    int n_constraints = GetNumberOfConstraints();
    jac = Jacobian(n_constraints, n_foothold_opt_vars);

    Vector2d center_final_stance_W = supp_polygon_container_.GetCenterOfFinalStance();
    Vector2d distance_to_center = goal_xy_ - center_final_stance_W;

    std::cout << "distance center to goal: " << distance_to_center.transpose() << std::endl;

    auto final_stance = supp_polygon_container_.GetFinalFootholds();
    for (auto f : final_stance) {
      if (!f.FixedByStart()) {
        for (auto dim : {X,Y}) {
          int idx = SupportPolygonContainer::Index(f.id,dim);
          jac.insert(0,idx) =  2*distance_to_center[dim]*(-1.0/final_stance.size());
        }
      }
    }
  }
  return jac;
}

FootholdGoalCost::VecBound
FootholdGoalCost::GetBounds () const
{
  int n_constraints = EvaluateConstraint().rows();
  VecBound bounds(n_constraints);
  for (auto& bound : bounds)
    bound = kEqualityBound_;

  return bounds;
}

} /* namespace zmp */
} /* namespace xpp */

