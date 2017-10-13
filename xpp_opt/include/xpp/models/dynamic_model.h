/**
 @file    dynamic_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 19, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_MODELS_DYNAMIC_MODEL_H_
#define XPP_OPT_INCLUDE_XPP_MODELS_DYNAMIC_MODEL_H_

#include <memory>
#include <vector>

#include <xpp_states/endeffectors.h>
#include <xpp_states/state.h>

#include <xpp/composite.h>


namespace xpp {
namespace opt {

class DynamicModel {
public:
  using Ptr       = std::shared_ptr<DynamicModel>;
  using ComPos    = Vector3d;
  using BaseAcc   = Vector6d;
  using EELoad    = Endeffectors<Vector3d>;
  using EEPos     = EndeffectorsPos;

  DynamicModel(double mass);
  virtual ~DynamicModel ();

  void SetCurrent(const ComPos& com, const EELoad&, const EEPos&);

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

  std::vector<EndeffectorID> GetEEIDs() const { return ee_pos_.GetEEsOrdered(); };

protected:
  EEPos ee_pos_;
  ComPos com_pos_;
  EELoad ee_force_;

  double g_; // gravity acceleration [m/s^2]
  double m_; // mass of the robot

  double normal_force_max_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_MODELS_DYNAMIC_MODEL_H_ */
