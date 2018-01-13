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

//#include <xpp_states/endeffectors.h>

#include <ifopt/composite.h>

namespace towr {

class DynamicModel {
public:
  using Ptr           = std::shared_ptr<DynamicModel>;
  using ComPos        = Eigen::Vector3d;
  using AngVel        = Eigen::Vector3d;
  using BaseAcc       = Eigen::Matrix<double,6,1>;
  using EndeffectorID = uint;
  using EEPos         = std::vector<Eigen::Vector3d>;
  using EELoad        = EEPos;
  using Jacobian      = ifopt::Component::Jacobian;

  DynamicModel(double mass);
  virtual ~DynamicModel () = default;

  void SetCurrent(const ComPos& com, const AngVel& w, const EELoad&, const EEPos&);

  virtual BaseAcc GetBaseAcceleration() const = 0;

  virtual Jacobian GetJacobianOfAccWrtBaseLin(const Jacobian& jac_base_lin_pos) const = 0;
  virtual Jacobian GetJacobianOfAccWrtBaseAng(const Jacobian& jac_base_ang_pos) const = 0;
  virtual Jacobian GetJacobianofAccWrtForce(const Jacobian& ee_force,
                                            EndeffectorID) const = 0;
  virtual Jacobian GetJacobianofAccWrtEEPos(const Jacobian&,
                                            EndeffectorID) const = 0;


  double GetGravityAcceleration() const { return g_; };
  double GetMass() const { return m_; };
  double GetStandingZForce() const;

  void SetForceLimit(double val) { normal_force_max_  = val; };
  double GetForceLimit() const {return normal_force_max_; };

//  std::vector<EndeffectorID> GetEEIDs() const { return ee_pos_.GetEEsOrdered(); };


protected:
  EEPos ee_pos_;
  ComPos com_pos_;
  AngVel omega_;
  EELoad ee_force_;

  double g_; // gravity acceleration [m/s^2]
  double m_; // mass of the robot

  double normal_force_max_;
};

} /* namespace towr */

#endif /* TOWR_MODELS_DYNAMIC_MODEL_H_ */
