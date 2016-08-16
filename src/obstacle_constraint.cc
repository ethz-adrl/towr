/**
 @file    obstacle_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 20, 2016
 @brief   Brief description
 */

#include <xpp/zmp/obstacle_constraint.h>
#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/ellipse.h>

namespace xpp {
namespace zmp {

ObstacleConstraint::ObstacleConstraint ()
{
  // TODO Auto-generated constructor stub
}

ObstacleConstraint::~ObstacleConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
ObstacleConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd footholds = opt_var->GetVariables(VariableNames::kFootholds);
  footholds_ = utils::ConvertEigToStd(footholds);
}

ObstacleConstraint::VectorXd
ObstacleConstraint::EvaluateConstraint () const
{
  VectorXd g(footholds_.size());

//  Ellipse ellipse(gap_width_x_, 3.0, gap_center_x_, 0.0);

  int i = 0;
  for (const Eigen::Vector2d& f : footholds_)
  {
//    g[i++] = ellipse.DistanceToEdge(f.x(), f.y());
    g[i++] = std::pow(f.x()-gap_center_x_,2);

  }

  return g;
}

ObstacleConstraint::VecBound
ObstacleConstraint::GetBounds () const
{
  VecBound bounds(EvaluateConstraint().rows());

  std::cout << "bounds.size(): " << bounds.size() << std::endl;

  for (Bound& b : bounds) {
    b.lower_ = std::pow(gap_width_x_/2.0,2);
    b.upper_ = kNoBound_.upper_;
  }

  return bounds;
}

} /* namespace zmp */
} /* namespace xpp */
