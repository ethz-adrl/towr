/**
@file    motion_type.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_

#include <memory>
#include <utility>
#include <vector>
#include <Eigen/Dense>

#include <xpp/endeffectors.h>
#include <xpp/state.h>
#include <xpp/opt/constraints/composite.h>

namespace xpp {
namespace opt {

enum CostName        { ComCostID, RangOfMotionCostID, PolyCenterCostID,
                       FinalComCostID, FinalStanceCostID, ForcesCostID };
enum ConstraintName  { State, JunctionCom, Dynamic, RomBox, TotalTime };

/** This class holds all the hardcoded values describing a motion.
  * This is specific to the robot and the type of motion desired.
  */
class MotionParameters {
public:
  using EEID             = EndeffectorID;
  using EEVec            = std::vector<EEID>;
  using ContactSequence  = std::vector<EndeffectorsBool>;
  using ContactTimings   = std::vector<std::vector<double>>;
  using Phase            = std::pair<EndeffectorsBool, double>;
  using ContactSchedule  = std::vector<Phase>;

  using PosXY            = Vector2d;
  using PosXYZ           = Vector3d;
  using MaxDevXYZ        = Vector3d;
  using NominalStance    = EndeffectorsPos;
  using CostWeights      = std::vector<std::pair<CostName, double>>;
  using UsedConstraints  = std::vector<ConstraintName>;

  using VecTimes         = std::vector<double>;
  using FixedVariables   = std::shared_ptr<Composite>;

  virtual ~MotionParameters();

  int GetEECount() const { return robot_ee_.size(); };
  NominalStance GetNominalStanceInBase() const { return nominal_stance_; };
//  ContactSchedule GetContactSchedule() const;
  MaxDevXYZ GetMaximumDeviationFromNominal() const;
  UsedConstraints GetUsedConstraints() const;
  CostWeights GetCostWeights() const;
  double GetTotalTime() const;

  VecTimes GetBasePolyDurations() const;
  double GetAvgZForce() const;
  bool ConstraintExists(ConstraintName c) const;


  int ee_splines_per_swing_phase_;
  int force_splines_per_stance_phase_;
  int order_coeff_polys_;

  double dt_base_polynomial_;
  double dt_range_of_motion_;
  double dt_dynamic_constraint_; /// how many times dynamics are enforced

  double min_phase_duration_;
  double max_phase_duration_;


  Eigen::Matrix3d GetInertiaParameters() const {return interia_; };
  double GetMass() const {return mass_; };
  double GetForceLimit() const {return force_limit_; };

  EEVec robot_ee_;
  ContactTimings contact_timings_;


protected:
  Eigen::Matrix3d interia_;
  double mass_;
  double force_limit_;

  MaxDevXYZ max_dev_xy_;
  ContactSequence contact_sequence_;
  NominalStance nominal_stance_;
  UsedConstraints constraints_;
  CostWeights cost_weights_;


  static Eigen::Matrix3d buildInertiaTensor(
          double Ixx, double Iyy, double Izz,
          double Ixy, double Ixz, double Iyz)
  {
    Eigen::Matrix3d I;
    I <<  Ixx, -Ixy, -Ixz,
         -Ixy,  Iyy, -Iyz,
         -Ixz, -Iyz,  Izz;
    return I;
  }
};

} // namespace opt
} // namespace hyq

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_ */
