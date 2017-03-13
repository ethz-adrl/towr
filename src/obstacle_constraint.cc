/**
 @file    obstacle_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 20, 2016
 @brief   Brief description
 */

#include <xpp/opt/obstacle_constraint.h>
#include <xpp/opt/variable_names.h>
#include <xpp/cartesian_declarations.h> // X,Y,Z

namespace xpp {
namespace opt {

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
  footholds_ = ConvertEigToStd(footholds);
}

ObstacleLineStrip::ObstacleLineStrip ()
{
}

ObstacleLineStrip::~ObstacleLineStrip ()
{
}

ObstacleLineStrip::VectorXd
ObstacleLineStrip::EvaluateConstraint () const
{
  VectorXd g(footholds_.size());

  int i = 0;
  for (const Eigen::Vector2d& f : footholds_)
    g[i++] = std::pow(f.x()-gap_center_x_,2);

  return g;
}

VecBound
ObstacleLineStrip::GetBounds () const
{
  VecBound bounds(EvaluateConstraint().rows());

  for (Bound& b : bounds) {
    b.lower_ = std::pow(gap_width_x_/2.0,2);
    b.upper_ = kNoBound_.upper_;
  }

  return bounds;
}

ObstacleLineStrip::Jacobian
ObstacleLineStrip::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empy matrix

  if (var_set == VariableNames::kFootholds) {
    int n_constraints = footholds_.size(); // only x position constrained
    int n_contact_variables = footholds_.size()*kDim2d;

    jac = Jacobian(n_constraints, n_contact_variables);

    for (int row=0; row<n_constraints; ++row) {
      int idx = ContactVars::Index(row, X);
      jac.insert(row, idx) = 2*(footholds_.at(row).x() - gap_center_x_);
    }
  }

  return jac;
}

ObstacleEllipse::ObstacleEllipse ()
{
  ellipse_ = Ellipse(gap_width_x_, 1.0, gap_center_x_, 0.0);
}

ObstacleEllipse::~ObstacleEllipse ()
{
}

ObstacleEllipse::VectorXd
ObstacleEllipse::EvaluateConstraint () const
{
  VectorXd g(footholds_.size());

  int i = 0;
  for (const Eigen::Vector2d& f : footholds_)
    g[i++] = ellipse_.DistanceToEdge(f.x(), f.y()) - ellipse_.GetConstant();

  return g;
}

VecBound
ObstacleEllipse::GetBounds () const
{
  VecBound bounds(EvaluateConstraint().rows());

  for (Bound& b : bounds) {
    b.lower_ = -1 * ellipse_.GetConstant();
    b.upper_ = kNoBound_.upper_;
  }

  return bounds;
}

ObstacleEllipse::Jacobian
ObstacleEllipse::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empy matrix

  if (var_set == VariableNames::kFootholds) {
    int n_constraints = footholds_.size(); // only x position constrained
    int n_contact_variables = footholds_.size()*kDim2d;
    jac = Jacobian(n_constraints, n_contact_variables);

    for (int row=0; row<n_constraints; ++row) {

      Eigen::Vector2d f = footholds_.at(row);
      auto jac_ellipse = ellipse_.GetJacobianWrtXY(f.x(), f.y());
      for (auto dim : {X,Y}) {
        int idx = ContactVars::Index(row, dim);
        jac.insert(row, idx) = jac_ellipse.at(dim);
      }
    }
  }

  return jac;
}

} /* namespace opt */
} /* namespace xpp */
