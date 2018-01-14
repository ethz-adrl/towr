/**
 @file    dynamic_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 19, 2017
 @brief   Brief description
 */

#ifndef TOWR_MODELS_DYNAMIC_MODEL_H_
#define TOWR_MODELS_DYNAMIC_MODEL_H_

#include <memory>
#include <vector>

#include <Eigen/Eigen>


namespace towr {

class DynamicModel {
public:
  using Ptr      = std::shared_ptr<DynamicModel>;
  using Vector3d = Eigen::Vector3d;
  using ComPos   = Eigen::Vector3d;
  using AngVel   = Eigen::Vector3d;
  using BaseAcc  = Eigen::Matrix<double,6,1>;
  using EE       = uint;
  using EEPos    = std::vector<Eigen::Vector3d>;
  using EELoad   = EEPos;
  using Jac      = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  DynamicModel(double mass);
  virtual ~DynamicModel () = default;

  void SetCurrent(const ComPos& com, const AngVel& w, const EELoad&, const EEPos&);

  virtual BaseAcc GetBaseAcceleration() const = 0;

  virtual Jac GetJacobianOfAccWrtBaseLin(const Jac& jac_base_lin_pos) const = 0;
  virtual Jac GetJacobianOfAccWrtBaseAng(const Jac& jac_base_ang_pos) const = 0;
  virtual Jac GetJacobianofAccWrtForce(const Jac& ee_force, EE) const = 0;
  virtual Jac GetJacobianofAccWrtEEPos(const Jac&, EE) const = 0;


  double GetGravityAcceleration() const { return g_; };
  double GetMass() const { return m_; };

protected:
  EEPos  ee_pos_;
  ComPos com_pos_;
  AngVel omega_;
  EELoad ee_force_;

  double g_; // gravity acceleration [m/s^2]
  double m_; // mass of the robot
};

} /* namespace towr */

#endif /* TOWR_MODELS_DYNAMIC_MODEL_H_ */
