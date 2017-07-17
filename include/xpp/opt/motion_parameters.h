/**
@file    motion_type.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_

#include <array>
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include <Eigen/Dense>

#include <xpp/endeffectors.h>

namespace xpp {
namespace opt {

enum MotionTypeID    { WalkID, TrotID, PaceID, BoundID, PushRecID };
enum CostName        { ComCostID, RangOfMotionCostID, PolyCenterCostID,
                       FinalComCostID, FinalStanceCostID };
enum ConstraintName  { InitCom, FinalCom, JunctionCom,
                       Dynamic, RomBox, Stance};

/** This class holds all the hardcoded values describing a motion.
  * This is specific to the robot and the type of motion desired.
  */
class MotionParameters {
public:
  using MotionTypePtr    = std::shared_ptr<MotionParameters>;
  using EEID             = EndeffectorID;
  using EEVec            = std::vector<EEID>;
  using ContactSequence  = std::vector<EndeffectorsBool>;
  using ContactTimings   = std::vector<double>;
  using Phase            = std::pair<EndeffectorsBool, double>;
  using ContactSchedule  = std::vector<Phase>;

  using PosXY            = Vector2d;
  using PosXYZ           = Vector3d;
  using NominalStance    = EndeffectorsPos;
  using MaxDevXYZ        = Vector3d;
  using CostWeights      = std::map<CostName, double>;
  using UsedConstraints  = std::vector<ConstraintName>;

  virtual ~MotionParameters();

  int GetEECount() const { return robot_ee_.size(); };
  NominalStance GetNominalStanceInBase() const { return nominal_stance_; };
  ContactSchedule GetContactSchedule() const;
  MaxDevXYZ GetMaximumDeviationFromNominal() const;
  UsedConstraints GetUsedConstraints() const;
  CostWeights GetCostWeights() const;
  double GetTotalTime() const;


  MotionTypeID id_;

  double duration_polynomial_;
//  double load_dt_; /// duration of piecewise-constant ee_load
  int polys_per_force_phase_; /// number of polynomials for each endeffector phase
  int n_constraints_per_poly_; /// how many times dynamics are enforced

//  PosXYZ offset_geom_to_com_; ///< between CoM and geometric center

  Eigen::Matrix3d GetInertiaParameters() const {return interia_; };
  double GetMass() const {return mass_; };
  double GetForceLimit() const {return force_limit_; };

  EEVec robot_ee_;
protected:
  Eigen::Matrix3d interia_;
  double mass_;
  double force_limit_;

  MaxDevXYZ max_dev_xy_;
  ContactSequence contact_sequence_;
  NominalStance nominal_stance_;
  ContactTimings contact_timings_;
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
