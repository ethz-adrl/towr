/**
 @file    force_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#include <towr/constraints/force_constraint.h>

#include <array>
#include <memory>
#include <vector>
#include <Eigen/Eigen>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/state.h>

#include <towr/variables/node_values.h>
#include <towr/variables/variable_names.h>

namespace towr {

using namespace opt;
using namespace xpp;


ForceConstraint::ForceConstraint (const HeightMap::Ptr& terrain,
                                  double force_limit,
                                  EndeffectorID ee)
    :opt::ConstraintSet(kSpecifyLater, "Force-Constraint-" + id::GetEEForceId(ee))
{

  terrain_   = terrain;
  force_limit_normal_direction_ = force_limit;
  mu_        = terrain->GetFrictionCoeff();
  ee_ = ee;

//  AddOptimizationVariables(opt_vars);


//  SetName("Force-Constraint-" + ee_force_id);
}

void
ForceConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  ee_force_  = x->GetComponent<EEForceNodes>(id::GetEEForceId(ee_));
  ee_motion_ = x->GetComponent<EEMotionNodes>(id::GetEEMotionId(ee_));

  int constraint_count = 0;
  n_constraints_per_node_ = 1 + 2*kDim2d; // positive normal force + 4 friction pyramid constraints
  for (int node_id=0; node_id<ee_force_->GetNodes().size(); ++node_id)
    if (ee_force_->IsStanceNode(node_id))
      constraint_count += n_constraints_per_node_;

  SetRows(constraint_count);
}

VectorXd
ForceConstraint::GetValues () const
{
  VectorXd g(GetRows());

  int row=0;
  auto force_nodes = ee_force_->GetNodes();
  for (int node_id=0; node_id<force_nodes.size(); ++node_id) {
    if (ee_force_->IsStanceNode(node_id)) {

      int phase  = ee_force_->GetPhase(node_id);
      Vector3d p = ee_motion_->GetValueAtStartOfPhase(phase); // doesn't change during stance phase
      Vector3d n = terrain_->GetNormalizedBasis(HeightMap::Normal, p.x(), p.y());
      Vector3d f = force_nodes.at(node_id).at(kPos);

      // unilateral force
      g(row++) = f.transpose() * n; // >0 (unilateral forces)

      // frictional pyramid
      Vector3d t1 = terrain_->GetNormalizedBasis(HeightMap::Tangent1, p.x(), p.y());
      g(row++) = f.transpose() * (t1 - mu_*n); // t1 < mu*n
      g(row++) = f.transpose() * (t1 + mu_*n); // t1 > -mu*n

      Vector3d t2 = terrain_->GetNormalizedBasis(HeightMap::Tangent2, p.x(), p.y());
      g(row++) = f.transpose() * (t2 - mu_*n); // t2 < mu*n
      g(row++) = f.transpose() * (t2 + mu_*n); // t2 > -mu*n
    }
  }

  return g;
}

ForceConstraint::VecBound
ForceConstraint::GetBounds () const
{
  VecBound bounds;

  for (int i=0; i<ee_force_->GetNodes().size(); ++i) {
    if (ee_force_->IsStanceNode(i)) {
      bounds.push_back(Bounds(0.0, force_limit_normal_direction_)); // unilateral forces
      bounds.push_back(BoundSmallerZero); // f_t1 <  mu*n
      bounds.push_back(BoundGreaterZero); // f_t1 > -mu*n
      bounds.push_back(BoundSmallerZero); // f_t2 <  mu*n
      bounds.push_back(BoundGreaterZero); // f_t2 > -mu*n
    }
  }

  return bounds;
}

void
ForceConstraint::FillJacobianBlock (std::string var_set,
                                    Jacobian& jac) const
{
  if (var_set == ee_force_->GetName()) {

    int row = 0;
    for (int node_id=0; node_id<ee_force_->GetNodes().size(); ++node_id) {
      if (ee_force_->IsStanceNode(node_id)) {

        // unilateral force
        int phase   = ee_force_->GetPhase(node_id);
        Vector3d p  = ee_motion_->GetValueAtStartOfPhase(phase); // doesn't change during phase
        Vector3d n  = terrain_->GetNormalizedBasis(HeightMap::Normal,   p.x(), p.y());
        Vector3d t1 = terrain_->GetNormalizedBasis(HeightMap::Tangent1, p.x(), p.y());
        Vector3d t2 = terrain_->GetNormalizedBasis(HeightMap::Tangent2, p.x(), p.y());

        for (auto dim : {X,Y,Z}) {
          int idx = ee_force_->Index(node_id, kPos, dim);

          int row_reset=row;

          jac.coeffRef(row_reset++, idx) = n(dim);              // unilateral force
          jac.coeffRef(row_reset++, idx) = t1(dim)-mu_*n(dim);  // f_t1 <  mu*n
          jac.coeffRef(row_reset++, idx) = t1(dim)+mu_*n(dim);  // f_t1 > -mu*n
          jac.coeffRef(row_reset++, idx) = t2(dim)-mu_*n(dim);  // f_t2 <  mu*n
          jac.coeffRef(row_reset++, idx) = t2(dim)+mu_*n(dim);  // f_t2 > -mu*n
        }

        row += n_constraints_per_node_;

      }
    }
  }


  if (var_set == ee_motion_->GetName()) {

    int row = 0;
    auto force_nodes = ee_force_->GetNodes();
    for (int force_node_id=0; force_node_id<force_nodes.size(); ++force_node_id) {
      if (ee_force_->IsStanceNode(force_node_id)) {

        int phase  = ee_force_->GetPhase(force_node_id);
        int ee_node_id = ee_motion_->GetNodeIDAtStartOfPhase(phase);

        Vector3d p = ee_motion_->GetValueAtStartOfPhase(phase); // doesn't change during pahse
        Vector3d f = force_nodes.at(force_node_id).at(kPos);

        for (auto dim : {X_,Y_}) {

          int idx = ee_motion_->Index(ee_node_id, kPos, dim);
          int row_reset=row;

          Vector3d dn  = terrain_->GetDerivativeOfNormalizedBasisWrt(HeightMap::Normal, dim, p.x(), p.y());
          Vector3d dt1 = terrain_->GetDerivativeOfNormalizedBasisWrt(HeightMap::Tangent1, dim, p.x(), p.y());
          Vector3d dt2 = terrain_->GetDerivativeOfNormalizedBasisWrt(HeightMap::Tangent2, dim, p.x(), p.y());


          // unilateral force
          jac.coeffRef(row_reset++, idx) = f.transpose()*dn;

          // friction force tangent 1 derivative
          jac.coeffRef(row_reset++, idx) = f.transpose()*(dt1-mu_*dn);
          jac.coeffRef(row_reset++, idx) = f.transpose()*(dt1+mu_*dn);

          // friction force tangent 2 derivative
          jac.coeffRef(row_reset++, idx) = f.transpose()*(dt2-mu_*dn);
          jac.coeffRef(row_reset++, idx) = f.transpose()*(dt2+mu_*dn);
        }

        row += n_constraints_per_node_;
      }
    }
  }
}

} /* namespace towr */
