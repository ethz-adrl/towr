/**
 @file    centroidal_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_CENTROIDAL_MODEL_H_
#define XPP_OPT_INCLUDE_XPP_OPT_CENTROIDAL_MODEL_H_

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>

#include <xpp/composite.h>
#include <xpp/endeffectors.h>
#include <xpp/state.h>

#include "dynamic_model.h"
#include "robot_model.h"

namespace xpp {
namespace opt {

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
//  Eigen::Matrix3d I_; /// inertia tensor of robot base
  Eigen::SparseMatrix<double, Eigen::RowMajor> I_inv_; // inverse of base inertia
};




// some specific dynamic and kinematic robot implementations
// spring_clean_ move to different file
class MonopedModel : public RobotModel {
public:
  MonopedModel();
  ~MonopedModel() {};
};

class BipedModel : public RobotModel {
public:
  BipedModel();
  ~BipedModel() {};
};

class HyqModel : public RobotModel {
public:
  HyqModel();
  ~HyqModel() {};

  virtual void SetInitialGait(int gait_id) override;
};

class AnymalModel : public RobotModel {
public:
  AnymalModel();
  ~AnymalModel() {};

  virtual void SetInitialGait(int gait_id) override;
};

class QuadrotorCentroidalModel : public RobotModel {
public:
  QuadrotorCentroidalModel();
  ~QuadrotorCentroidalModel() {};
};



} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CENTROIDAL_MODEL_H_ */
