/**
 @file    a_foothold_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 8, 2016
 @brief   Defines an abstract FootholdCost class and one concrete derivation.
 */

#include <xpp/zmp/a_foothold_cost.h>
#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

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

void
FootholdGoalCost::Init (const Vector2d& goal_xy,
                        const SupportPolygonContainer& supp_polygon_container)
{
  AFootholdCost::Init(supp_polygon_container);
  goal_xy_ = goal_xy;
}

double
FootholdGoalCost::EvaluateCost () const
{
  Vector2d center_final_stance = supp_polygon_container_.GetCenterOfFinalStance();
  double cost = (goal_xy_ - center_final_stance).norm();
  return cost;
}

} /* namespace zmp */
} /* namespace xpp */

