/**
 @file    polygon_center_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#include <xpp/opt/polygon_center_constraint.h>

namespace xpp {
namespace opt {

PolygonCenterConstraint::PolygonCenterConstraint ()
{
}

PolygonCenterConstraint::~PolygonCenterConstraint ()
{
}

void
PolygonCenterConstraint::Init (const EndeffectorLoad& ee_load,
                               const EndeffectorsMotion& ee_motion)
{
  ee_load_ = ee_load;
  ee_motion_ = ee_motion;
}

void
PolygonCenterConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  Eigen::VectorXd lambdas = opt_var->GetVariables(EndeffectorLoad::ID);
  ee_load_.SetOptimizationVariables(lambdas);
}

PolygonCenterConstraint::VectorXd
PolygonCenterConstraint::EvaluateConstraint () const
{
  VectorXd g(ee_load_.GetNumberOfSegments());

  for (int k=0; k<g.rows(); ++k) {
    double g_node = 0.0;
    double t = ee_load_.GetTStart(k);
    int num_contacts = ee_motion_.GetContacts(t).size();

    for (auto lambda : ee_load_.GetLoadValuesIdx(k).ToImpl())
      g_node += std::pow(lambda,2) - 2./num_contacts*lambda;

    g(k) = g_node;
  }

  return g;
}

VecBound
PolygonCenterConstraint::GetBounds () const
{
  std::vector<Bound> bounds;

  for (int k=0; k<ee_load_.GetNumberOfSegments(); ++k) {
    double t = ee_load_.GetTStart(k);
    int m = ee_motion_.GetContacts(t).size();
    bounds.push_back(Bound(-1./m, -1./m)); // should lie in center of polygon
  }

//  for (int m : ee_load_.GetContactsPerNode())
//    bounds.push_back(Bound(-1./m, -1./m)); // should lie in center of polygon

  return bounds;
}

PolygonCenterConstraint::Jacobian
PolygonCenterConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empty matrix

  if (var_set == EndeffectorLoad::ID) {

    int m = ee_load_.GetNumberOfSegments();
    int n = ee_load_.GetOptVarCount();
    jac = Jacobian(m, n);

    for (int k=0; k<m; ++k) {
      double t = ee_load_.GetTStart(k);
      int num_contacts = ee_motion_.GetContacts(t).size();

      auto lambda_k = ee_load_.GetLoadValuesIdx(k);

      for (auto ee : lambda_k.GetEEsOrdered()) {
        int idx = ee_load_.IndexDiscrete(k,ee);
        jac.insert(k,idx) = 2*(lambda_k.At(ee) - 1./num_contacts);
      }
    }
  }

  return jac;
}

} /* namespace opt */
} /* namespace xpp */
