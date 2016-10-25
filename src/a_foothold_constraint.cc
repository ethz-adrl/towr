/**
 @file    a_foothold_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 8, 2016
 @brief   Defines an abstract FootholdCost class and one concrete derivation.
 */

#include <xpp/opt/a_foothold_constraint.h>
#include <xpp/opt/optimization_variables.h>

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
                                    const SupportPolygonContainer& supp_poly)
{
  Init(supp_poly);
  goal_xy_ = goal_xy;
}

FootholdFinalStanceConstraint::~FootholdFinalStanceConstraint ()
{
}

FootholdFinalStanceConstraint::VectorXd
FootholdFinalStanceConstraint::EvaluateConstraint () const
{
  auto final_stance = supp_polygon_container_.GetFinalFootholds();

  VectorXd g(/*final_stance.size()*/4*kDim2d);
  std::cout << "vector g.rows() : " << g.rows() << std::endl;

  std::cout << "final_stance.size(): " << final_stance.size() << std::endl;

  int c=0;
  for (auto f : final_stance) {
    if (!f.IsFixedByStart()) {
      for (auto dim : {X,Y}) {
        double distance = goal_xy_(dim) - f.p(dim);
        g(c++) = std::pow(distance,2);
      }
    }
  }

//  Vector2d center_final_stance_W = supp_polygon_container_.GetCenterOfFinalStance();
//  Vector2d distance_to_center = goal_xy_ - center_final_stance_W;
//  g = distance_to_center.transpose() * distance_to_center;


  return g;
}

FootholdFinalStanceConstraint::Jacobian
FootholdFinalStanceConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac;

  if (var_set == VariableNames::kFootholds) {

    int n_foothold_opt_vars = supp_polygon_container_.GetTotalFreeCoeff();
    int n_constraints = GetNumberOfConstraints();
    jac = Jacobian(n_constraints, n_foothold_opt_vars);

//    Vector2d center_final_stance_W = supp_polygon_container_.GetCenterOfFinalStance();
//    Vector2d distance_to_center = goal_xy_ - center_final_stance_W;

    auto final_stance = supp_polygon_container_.GetFinalFootholds();
    int c=0;
    for (auto f : final_stance) {
      if (!f.IsFixedByStart()) {
        for (auto dim : {X,Y}) {
          int idx = SupportPolygonContainer::Index(f.id,dim);
          jac.insert(c++,idx) =  2*(goal_xy_(dim) - f.p(dim))*(-1);
        }
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

