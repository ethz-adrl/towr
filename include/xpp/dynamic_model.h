/**
 @file    dynamic_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_
#define XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_

#include <vector>

#include <xpp/composite.h>
#include <xpp/endeffectors.h>
#include <xpp/state.h>

namespace xpp {
namespace opt {

/**
 * @brief Interface to the dynamic system model.
 *
 * This model must implement the relationship between the endeffector forces
 * and the movement of the 6-dimensional base.
 */
class DynamicModel {
public:
  using Ptr       = std::shared_ptr<DynamicModel>;
  using ComPos    = Vector3d;
  using BaseAcc   = Vector6d;
  using EELoad    = Endeffectors<Vector3d>;
  using EEPos     = EndeffectorsPos;


  DynamicModel (int ee_count);
  virtual ~DynamicModel ();


  void SetCurrent(const ComPos& com, const EELoad&, const EEPos&);

  virtual BaseAcc GetBaseAcceleration() const = 0;

  virtual Jacobian GetJacobianOfAccWrtBaseLin(const Jacobian& jac_base_lin_pos) const = 0;
  virtual Jacobian GetJacobianOfAccWrtBaseAng(const Jacobian& jac_base_ang_pos) const = 0;
  virtual Jacobian GetJacobianofAccWrtForce(const Jacobian& ee_force,
                                            EndeffectorID) const = 0;
  virtual Jacobian GetJacobianofAccWrtEEPos(const Jacobian&,
                                            EndeffectorID) const = 0;

  std::vector<EndeffectorID> GetEEIDs() const;
  int GetEECount() const { return ee_ids_.size(); };
  double GetGravityAcceleration() const { return 9.80665; }; // [m/s^2]
  double GetForceLimit() const {return normal_force_max_; };


  // the kinematic parameters of the model
  EndeffectorsPos GetNominalStanceInBase() const { return nominal_stance_; };
  Vector3d GetMaximumDeviationFromNominal() const { return max_dev_from_nominal_; };

  std::vector<std::string> GetEndeffectorNames() const;

protected:
  ComPos com_pos_;
  EELoad ee_force_;
  EEPos ee_pos_;

  std::vector<EndeffectorID> ee_ids_;
  EndeffectorsPos nominal_stance_;
  Vector3d max_dev_from_nominal_;
  double normal_force_max_;

  std::map<std::string, EndeffectorID> map_id_to_ee_;

};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_ */
