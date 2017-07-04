/**
 @file    articulated_robot_state_cartesian.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 10, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_COMMON_INCLUDE_XPP_OPT_ROBOT_STATE_CARTESIAN_H_
#define XPP_XPP_COMMON_INCLUDE_XPP_OPT_ROBOT_STATE_CARTESIAN_H_

#include <xpp/state.h>
#include <xpp/endeffectors.h>

namespace xpp {

class RobotStateCommon {
public:
  using ContactState  = EndeffectorsBool;

  RobotStateCommon(int n_ee);
  virtual ~RobotStateCommon ();

  double t_global_;
  State3d base_;
  ContactState is_contact_;
};

class RobotDataHolder {
public:
  using ContactState  = EndeffectorsBool;
  using EEID          = EndeffectorID;

public:
  RobotDataHolder(int n_ee);
  virtual ~RobotDataHolder ();

  int GetEEInContactCount() const {
    int count = 0;

    for (auto ee : GetEndeffectors())
      count += GetContactState().At(ee);

    return count;
  }

  RobotStateCommon GetCommon() const          { return data_; };
  void SetCommon(const RobotStateCommon& b)   { data_ = b; };

  void SetBase(const State3d& b)              { data_.base_ = b; };
  const State3d& GetBase() const              { return data_.base_; };
  State3d& GetBase()                          { return data_.base_; };

  void SetContactState(const ContactState& c) { data_.is_contact_ = c; };
  const ContactState& GetContactState() const { return data_.is_contact_; };

  double GetTime() const                      { return data_.t_global_; };
  void SetTime(double t)                      { data_.t_global_ = t; };

  std::vector<EEID> GetEndeffectors() const   { return data_.is_contact_.GetEEsOrdered(); };
  int GetEECount() const                      { return GetEndeffectors().size(); };

private:
  RobotStateCommon data_;
};


class RobotStateCartesian : public RobotDataHolder {
public:
  using FeetArray        = Endeffectors<StateLin3d>;
  using EEPos            = EndeffectorsPos;
  using EEForces         = Endeffectors<Vector3d>;

  RobotStateCartesian (int n_ee = 0);
  virtual ~RobotStateCartesian ();

  const FeetArray& GetEEState() const;
  void SetEEStateInWorld(MotionDerivative dxdt, const EEPos& val);
  void SetEEForcesInWorld(const EEForces& val);
  void SetEEStateInWorld(const FeetArray& ee);

  EEForces GetEEForces() const;

  EEPos GetEEPos() const;
  double GetZAvg() const;

private:
  FeetArray feet_W_;
  EEForces ee_forces_;
};

} /* namespace xpp */

#endif /* XPP_XPP_COMMON_INCLUDE_XPP_OPT_ROBOT_STATE_CARTESIAN_H_ */
