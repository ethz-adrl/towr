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

  // the times at which to evalute the constraint
  double t = 0.0;
  double dt = 0.1;
  std::vector<double> vec_t;
  while (t < com_motion_->GetTotalTime()) {
    vec_t.push_back(t);
    t += dt;
  }

  stance_feet_cal_.Init(vec_t, *com_motion_, supp_polygon_container_);
}

void
RangeOfMotionConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd x_coeff   = opt_var->GetVariables(VariableNames::kSplineCoeff);
  VectorXd footholds = opt_var->GetVariables(VariableNames::kFootholds);

  stance_feet_cal_.Update(x_coeff, utils::ConvertEigToStd(footholds));


  com_motion_->SetCoefficients(x_coeff);
  supp_polygon_container_.SetFootholdsXY(utils::ConvertEigToStd(footholds));
}

RangeOfMotionConstraint::VectorXd
RangeOfMotionConstraint::EvaluateConstraint () const
{
//  using namespace xpp::utils::coords_wrapper; // X, Y
  std::vector<double> g_vec;


  // for every discrete time t
  auto vec_com_pos_W = stance_feet_cal_.CalculateComPostionInWorld();
  auto vec_stance_feet_W = stance_feet_cal_.GetStanceFootholdsInWorld();

  for (int k=0; k<vec_com_pos_W.size(); ++k) {

    PosXY com_pos_k     = vec_com_pos_W.at(k);
    auto stance_feet_k = vec_stance_feet_W.at(k);

    for (auto contact : stance_feet_k) {

      for (auto dim : {X,Y}) {
        if(contact.id == xpp::hyq::Foothold::kFixedByStart)
          g_vec.push_back( -com_pos_k(dim) ); // contact goes in bounds because never changes
        else
          g_vec.push_back(contact.p(dim) - com_pos_k(dim));
      }
    }
  }

//  // refactor _write out really simple constraint just to test ZMP motion
//  auto feet = supp_polygon_container_.GetFootholds();
//  for (const auto& f : feet) {
//    g_vec.push_back(f.p.x());
//    g_vec.push_back(f.p.y());
//  }


  return Eigen::Map<VectorXd>(&g_vec[0], g_vec.size());
}

RangeOfMotionConstraint::VecBound
RangeOfMotionConstraint::GetBounds () const
{
  std::vector<Bound> bounds;

  double d = 0.3; // bounding box edge length of each foot


//  auto vec_stance_feet_W = stance_feet_cal_.GetStanceFootholdsInWorld();


  auto contact_info_vec = stance_feet_cal_.GetContactInfoVec();

  for (auto c :  contact_info_vec) {

    PosXY start_offset = PosXY::Zero(); // because initial foothold is fixed
    if (c.foothold_id_ == xpp::hyq::Foothold::kFixedByStart) {
      start_offset = supp_polygon_container_.GetStartFoothold(c.leg_).p.topRows(kDim2d);
    }

    PosXY pos_nom_B = GetNominalPositionInBase(c.leg_);
    for (auto dim : {X,Y}) {
      Bound b;
      b.upper_ = pos_nom_B(dim) + d/2.;
      b.lower_ = pos_nom_B(dim) - d/2.;
      b -= start_offset(dim);
      bounds.push_back(b);
    }

  }


//
//  for (auto stance : vec_stance_feet_W) {
//
//    for (auto contact : stance) {
//
//      PosXY pos_nom_B = GetNominalPositionInBase(contact.leg);
//
//      for (auto dim : {X,Y}) {
//
//        Bound b;
//        b.upper_ = pos_nom_B(dim) + d/2.;
//        b.lower_ = pos_nom_B(dim) - d/2.;
//
//        if (contact.id == xpp::hyq::Foothold::kFixedByStart) {
//          b -= contact.p(dim);
//        }
//
//        bounds.push_back(b);
//
//      }
//
//    }
//
//  }












//  // this is for creating fixed footholds (remember to comment in constraints above as well)
//  auto start_stance = supp_polygon_container_.GetStartStance();
//  auto steps = supp_polygon_container_.GetFootholds();
//
//  double step_length = 0.15;
//  for (const auto& s : steps) {
//    auto leg = s.leg;
//    auto start_foothold = hyq::Foothold::GetLastFoothold(leg, start_stance);
//
//    bounds.push_back(Bound(start_foothold.p.x() + step_length, start_foothold.p.x() + step_length));
//    bounds.push_back(Bound(start_foothold.p.y(), start_foothold.p.y()));
//  }

  return bounds;
}

RangeOfMotionConstraint::Jacobian
RangeOfMotionConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empy matrix

  using Triplet =  Eigen::Triplet<double>;

  if (var_set == VariableNames::kFootholds) {
    int n = supp_polygon_container_.GetTotalFreeCoeff();
    int m = EvaluateConstraint().rows(); // count number of constraints
    jac = Jacobian(m,n);


    auto vec_stance_feet_W = stance_feet_cal_.GetStanceFootholdsInWorld();


    int row=0;
    std::vector<Triplet> jac_triplets;
    for (auto stance : vec_stance_feet_W) {

      for (auto contact : stance) {

        int foothold_id = contact.id;
        if (foothold_id != xpp::hyq::Foothold::kFixedByStart) {
          for (auto dim : {X,Y}) {
            jac_triplets.push_back(Triplet(row+dim, SupportPolygonContainer::Index(foothold_id,dim), 1.0));
          }
        }
        row += kDim2d;
      }
    }

    jac.setFromTriplets(jac_triplets.begin(), jac_triplets.end());
  }




  if (var_set == VariableNames::kSplineCoeff) {
    int m = EvaluateConstraint().rows(); // count number of constraints
    int n = com_motion_->GetTotalFreeCoeff();
    jac = Jacobian(m,n);


    auto vec_stance_feet_W = stance_feet_cal_.GetStanceFootholdsInWorld();

    int row=0;
    for (int k=0; k<stance_feet_cal_.times_.size(); ++k) {

      double t = stance_feet_cal_.times_.at(k);

      for (auto stance : vec_stance_feet_W.at(k))
        for (auto dim : {X,Y})
          jac.row(row++) = -com_motion_->GetJacobian(t,kPos,dim);
    }
  }



//  // keep this somehow, nice for debugging
//  if (var_set == VariableNames::kFootholds) {
//    int n = supp_polygon_container_.GetTotalFreeCoeff();
//    jac = Jacobian(n,n);
//    jac.setIdentity();
//  }





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

