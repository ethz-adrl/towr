/**
 @file    centroidal_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_CENTROIDAL_MODEL_H_
#define XPP_OPT_INCLUDE_XPP_OPT_CENTROIDAL_MODEL_H_

#include <memory>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp_states/endeffectors.h>
#include <xpp_solve/composite.h>

#include <xpp_opt/models/dynamic_model.h>

namespace xpp {

/**
 * @brief Centroidal Dynamics for the 6-DoF Base to model the system.
 */
class CentroidalModel : public DynamicModel {
public:
  using Ptr = std::shared_ptr<CentroidalModel>;

  CentroidalModel (double mass, const Eigen::Matrix3d& inertia, int ee_count);
  virtual ~CentroidalModel ();

  virtual BaseAcc GetBaseAcceleration() const override;

  virtual Jacobian GetJacobianOfAccWrtBaseLin(const Jacobian& jac_base_lin_pos) const override;
  virtual Jacobian GetJacobianOfAccWrtBaseAng(const Jacobian& jac_base_ang_pos) const override;
  virtual Jacobian GetJacobianofAccWrtForce(const Jacobian& jac_force,
                                            EndeffectorID) const override;
  virtual Jacobian GetJacobianofAccWrtEEPos(const Jacobian& jac_ee_pos,
                                            EndeffectorID) const override;

private:
  Eigen::SparseMatrix<double, Eigen::RowMajor> I_inv_; // inverse of base inertia
};

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CENTROIDAL_MODEL_H_ */
