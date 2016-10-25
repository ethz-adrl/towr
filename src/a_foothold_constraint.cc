/**
 @file    a_foothold_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 8, 2016
 @brief   Defines an abstract FootholdCost class and one concrete derivation.
 */

#include <xpp/opt/a_foothold_constraint.h>
#include <xpp/opt/optimization_variables.h>
#include <xpp/opt/a_robot_interface.h>

namespace xpp {
namespace opt {

using namespace xpp::utils; // X, Y

AFootholdConstraint::AFootholdConstraint ()
{
  // TODO Auto-generated constructor stub
}

AFootholdConstraint::~AFootholdConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
AFootholdConstraint::Init (const SupportPolygonContainer& supp_polygon_container)
{
  supp_polygon_container_ = supp_polygon_container;
}

void
AFootholdConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  Eigen::VectorXd footholds = opt_var->GetVariables(VariableNames::kFootholds);
  supp_polygon_container_.SetFootholdsXY(utils::ConvertEigToStd(footholds));
}

FootholdFinalStanceConstraint::FootholdFinalStanceConstraint (
                                    const Vector2d& goal_xy,
                                    const SupportPolygonContainer& supp_poly,
                                    RobotPtrU robot)
{
  Init(supp_poly);
  goal_xy_ = goal_xy;
  robot_ = std::move(robot);
}

FootholdFinalStanceConstraint::~FootholdFinalStanceConstraint ()
{
}

FootholdFinalStanceConstraint::VectorXd
FootholdFinalStanceConstraint::EvaluateConstraint () const
{
  auto final_stance = supp_polygon_container_.GetFinalFreeFootholds();

  VectorXd g(final_stance.size()*kDim2d);

  int c=0;
  for (auto f : final_stance) {
    Vector2d foot_to_nominal_W = GetFootToNominalInWorld(f);
    for (auto dim : {X,Y})
      g(c++) = foot_to_nominal_W(dim);
  }

  return g;
}

FootholdFinalStanceConstraint::Vector2d
FootholdFinalStanceConstraint::GetFootToNominalInWorld ( const hyq::Foothold& foot_W) const
{
  Vector2d goal_to_nom_B = robot_->GetNominalStanceInBase(foot_W.leg);
  Eigen::Matrix2d W_R_B = Eigen::Matrix2d::Identity(); // attention: assumes no rotation world to base
  Vector2d goal_to_nom_W = W_R_B * goal_to_nom_B;

  Vector2d foot_to_goal_W    = goal_xy_ - foot_W.p.topRows(utils::kDim2d);
  Vector2d foot_to_nominal_W = foot_to_goal_W + goal_to_nom_W;

  return foot_to_nominal_W;
}

FootholdFinalStanceConstraint::Jacobian
FootholdFinalStanceConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac;

  if (var_set == VariableNames::kFootholds) {

    int n_foothold_opt_vars = supp_polygon_container_.GetTotalFreeCoeff();
    int n_constraints = GetNumberOfConstraints();
    jac = Jacobian(n_constraints, n_foothold_opt_vars);

    auto final_stance = supp_polygon_container_.GetFinalFreeFootholds();
    int c=0;
    for (auto f : final_stance) {
      Vector2d foot_to_nominal_W = GetFootToNominalInWorld(f);
      for (auto dim : {X,Y}) {
        int idx = SupportPolygonContainer::Index(f.id,dim);
        jac.insert(c++,idx) =  -1;
      }
    }
  }
  return jac;
}

FootholdFinalStanceConstraint::VecBound
FootholdFinalStanceConstraint::GetBounds () const
{
  int n_constraints = EvaluateConstraint().rows();
  VecBound bounds(n_constraints);
  for (auto& bound : bounds)
    bound = kEqualityBound_;

  return bounds;
}


} /* namespace zmp */
} /* namespace xpp */

