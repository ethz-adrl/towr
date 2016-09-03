/*
 * range_of_motion_constraint.cc
 *
 *  Created on: May 26, 2016
 *      Author: winklera
 */

#include <xpp/zmp/range_of_motion_constraint.h>
#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/optimization_variables_interpreter.h>

namespace xpp {
namespace zmp {

typedef xpp::utils::StdVecEigen2d FootholdsXY;
typedef Eigen::Vector2d Vector2d;

RangeOfMotionConstraint::RangeOfMotionConstraint ()
{
}

void
RangeOfMotionConstraint::Init (const OptimizationVariablesInterpreter& interpreter)
{
  com_motion_             = interpreter.GetSplineStructure();
  supp_polygon_container_ = interpreter.GetSuppPolygonContainer();
}

void
RangeOfMotionConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd x_coeff   = opt_var->GetVariables(VariableNames::kSplineCoeff);
  VectorXd footholds = opt_var->GetVariables(VariableNames::kFootholds);

  com_motion_->SetCoefficients(x_coeff);
  supp_polygon_container_.SetFootholdsXY(utils::ConvertEigToStd(footholds));
}

RangeOfMotionConstraint::VectorXd
RangeOfMotionConstraint::EvaluateConstraint () const
{
  std::vector<double> g_vec;

  // refactor _write out really simple constraint just to test ZMP motion
  auto feet = supp_polygon_container_.GetFootholds();
  for (const auto& f : feet) {
    g_vec.push_back(f.p.x());
    g_vec.push_back(f.p.y());
  }







//  // this is the standard constraint that has been proven to work etc.
//  utils::StdVecEigen2d B_r_baseToFeet, B_r_baseToNominal;
//  B_r_baseToFeet = builder_.GetFeetInBase(com_motion_, supp_polygon_container_, B_r_baseToNominal);
//
//
//  for (uint i=0; i<B_r_baseToFeet.size(); ++i) {
//
////    // for the first time discretization, the footholds (start stance) as well
////    // as the body position is fixed, so no constraint must be added there.
////    static const int n_contacts_first_node = 4;
////    if (i<n_contacts_first_node)
////      continue; // the initial body position and footholds are fixed anyway
////
////    Vector2d B_r_footToNominal = -B_r_baseToFeet.at(i) + B_r_baseToNominal.at(i);
////
////    // circle constraint on feet (nonlinear constraint (squared))
////    g_vec.push_back(B_r_footToNominal.norm());
//
//    // squared constraints on feet (sometimes better convergence)
//    g_vec.push_back(B_r_baseToFeet.at(i).x());
//    g_vec.push_back(B_r_baseToFeet.at(i).y());
//  }




  return Eigen::Map<VectorXd>(&g_vec[0], g_vec.size());
}

RangeOfMotionConstraint::VecBound
RangeOfMotionConstraint::GetBounds () const
{
  std::vector<Bound> bounds;

  // this is for creating fixed footholds (remember to comment in constraints above as well)
  auto start_stance = supp_polygon_container_.GetStartStance();
  auto steps = supp_polygon_container_.GetFootholds();

  double step_length = 0.15;
  for (const auto& s : steps) {
    auto leg = s.leg;
    auto start_foothold = hyq::Foothold::GetLastFoothold(leg, start_stance);

    bounds.push_back(Bound(start_foothold.p.x() + step_length, start_foothold.p.x() + step_length));
    bounds.push_back(Bound(start_foothold.p.y(), start_foothold.p.y()));
  }




//  // this is the standard bound on the foothold that has been tested thoroughly
//  utils::StdVecEigen2d nominal_footholds_b;
//  builder_.GetFeetInBase(com_motion_,supp_polygon_container_,nominal_footholds_b);
//  double radius_x = 0.15; //m
//  double radius_y = 0.15; //m
//  for (int i=0; i<nominal_footholds_b.size(); ++i) {
//    Bound x_bound(nominal_footholds_b.at(i).x()-radius_x, nominal_footholds_b.at(i).x()+radius_x);
//    Bound y_bound(nominal_footholds_b.at(i).y()-radius_y, nominal_footholds_b.at(i).y()+radius_y);
//    bounds.push_back(x_bound);
//    bounds.push_back(y_bound);
//  }



//  // for circular bounds on footholds
//  VectorXd g = EvaluateConstraint();
//  double radius = 0.15; //m
//  Bound bound = AConstraint::kNoBound_;
//  bound.upper_ = radius;
//  for (int i=0; i<g.rows(); ++i)
//    bounds.push_back(bound);



  return bounds;
}

RangeOfMotionConstraint::Jacobian
RangeOfMotionConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empy matrix

  if (var_set == VariableNames::kFootholds) {
    int n = supp_polygon_container_.GetTotalFreeCoeff();
    jac = Jacobian(n,n);
    jac.setIdentity();
  }

  return jac;
}

RangeOfMotionConstraint::PosXY
RangeOfMotionConstraint::GetNominalPositionInBase (LegID leg) const
{
  const double x_nominal_b = 0.36; // 0.4
  const double y_nominal_b = 0.33; // 0.4

  switch (leg) {
    case hyq::LF: return PosXY( x_nominal_b,   y_nominal_b); break;
    case hyq::RF: return PosXY( x_nominal_b,  -y_nominal_b); break;
    case hyq::LH: return PosXY(-x_nominal_b,   y_nominal_b); break;
    case hyq::RH: return PosXY(-x_nominal_b,  -y_nominal_b); break;
    default: assert(false); // this should never happen
  }
}

} /* namespace zmp */
} /* namespace xpp */

