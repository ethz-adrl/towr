/**
 @file    robot_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_
#define XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <xpp/endeffectors.h>

#include "dynamic_model.h"
#include "kinematic_model.h"

namespace xpp {
namespace opt {

/**
 * @brief Interface to the dynamic system model.
 *
 * This model must implement the relationship between the endeffector forces
 * and the movement of the 6-dimensional base.
 */
// zmp_ don't just call dynamic model, as it also includes kinematic parameters
// and initialization timings
class RobotModel {
public:
  using Ptr       = std::shared_ptr<RobotModel>;
//  using ComPos    = Vector3d;
//  using BaseAcc   = Vector6d;
//  using EELoad    = Endeffectors<Vector3d>;
//  using EEPos     = EndeffectorsPos;
  using ContactTimings   = std::vector<std::vector<double>>;


  RobotModel (int ee_count);
  virtual ~RobotModel ();


//  void SetCurrent(const ComPos& com, const EELoad&, const EEPos&);
//
//  virtual BaseAcc GetBaseAcceleration() const = 0;
//
//  virtual Jacobian GetJacobianOfAccWrtBaseLin(const Jacobian& jac_base_lin_pos) const = 0;
//  virtual Jacobian GetJacobianOfAccWrtBaseAng(const Jacobian& jac_base_ang_pos) const = 0;
//  virtual Jacobian GetJacobianofAccWrtForce(const Jacobian& ee_force,
//                                            EndeffectorID) const = 0;
//  virtual Jacobian GetJacobianofAccWrtEEPos(const Jacobian&,
//                                            EndeffectorID) const = 0;

  std::vector<EndeffectorID> GetEEIDs() const;
  int GetEECount() const { return ee_ids_.size(); };
//  double GetGravityAcceleration() const { return 9.80665; }; // [m/s^2]
  double GetForceLimit() const {return normal_force_max_; };


  std::vector<std::string> GetEndeffectorNames() const;

  virtual void SetInitialGait(int gait_id) {}; // does nothing if not overwritten
  std::vector<double> GetNormalizedInitialTimings(EndeffectorID ee);

  KinematicModel::Ptr kinematic_model_;
  DynamicModel::Ptr dynamic_model_;


protected:
//  // dynamic model parameters
//  ComPos com_pos_;
//  EELoad ee_force_;
//  EEPos ee_pos_;



  std::vector<EndeffectorID> ee_ids_;

  ContactTimings contact_timings_;
  void NormalizeTimesToOne(EndeffectorID ee);

//  EndeffectorsPos nominal_stance_;
//  Vector3d max_dev_from_nominal_;
  double normal_force_max_;

  std::map<std::string, EndeffectorID> map_id_to_ee_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_ */
