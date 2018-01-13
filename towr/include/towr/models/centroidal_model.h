/**
 @file    centroidal_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#ifndef TOWR_MODELS_CENTROIDAL_MODEL_H_
#define TOWR_MODELS_CENTROIDAL_MODEL_H_

#include <memory>
#include <Eigen/Eigen>

#include "dynamic_model.h"

namespace towr {

/**
 * @brief Centroidal Dynamics for the 6-DoF Base to model the system.
 */
class CentroidalModel : public DynamicModel {
public:
  using Ptr = std::shared_ptr<CentroidalModel>;
  using Vector3d = Eigen::Vector3d;

  CentroidalModel (double mass, const Eigen::Matrix3d& inertia, int ee_count);
  virtual ~CentroidalModel () = default;

  virtual BaseAcc GetBaseAcceleration() const override;

  virtual Jacobian GetJacobianOfAccWrtBaseLin(const Jacobian& jac_base_lin_pos) const override;
  virtual Jacobian GetJacobianOfAccWrtBaseAng(const Jacobian& jac_ang_vel) const override;
  virtual Jacobian GetJacobianofAccWrtForce(const Jacobian& jac_force,
                                            EndeffectorID) const override;
  virtual Jacobian GetJacobianofAccWrtEEPos(const Jacobian& jac_ee_pos,
                                            EndeffectorID) const override;

private:
  Eigen::Matrix3d I_dense_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> I_; // inverse of base inertia
  Eigen::SparseMatrix<double, Eigen::RowMajor> I_inv_; // inverse of base inertia
};

} /* namespace towr */

#endif /* TOWR_MODELS_CENTROIDAL_MODEL_H_ */
