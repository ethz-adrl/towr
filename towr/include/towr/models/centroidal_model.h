/**
 @file    centroidal_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#ifndef TOWR_MODELS_CENTROIDAL_MODEL_H_
#define TOWR_MODELS_CENTROIDAL_MODEL_H_

#include "dynamic_model.h"

namespace towr {

/**
 * @brief Centroidal Dynamics for the 6-DoF Base to model the system.
 */
class CentroidalModel : public DynamicModel {
public:
  CentroidalModel (double mass,
                   double Ixx, double Iyy, double Izz,
                   double Ixy, double Ixz, double Iyz,
                   int ee_count);
  CentroidalModel (double mass, const Eigen::Matrix3d& inertia, int ee_count);
  virtual ~CentroidalModel () = default;

  virtual BaseAcc GetBaseAcceleration() const override;

  virtual Jac GetJacobianOfAccWrtBaseLin(const Jac& jac_base_lin_pos) const override;
  virtual Jac GetJacobianOfAccWrtBaseAng(const Jac& jac_ang_vel) const override;
  virtual Jac GetJacobianofAccWrtForce(const Jac& jac_force, EE) const override;
  virtual Jac GetJacobianofAccWrtEEPos(const Jac& jac_ee_pos, EE) const override;

private:
  Eigen::Matrix3d I_dense_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> I_; // inverse of base inertia
  Eigen::SparseMatrix<double, Eigen::RowMajor> I_inv_; // inverse of base inertia
};


} /* namespace towr */

#endif /* TOWR_MODELS_CENTROIDAL_MODEL_H_ */
