/**
 @file    endeffectors_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 14, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTORS_MOTION_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTORS_MOTION_H_

#include <xpp/opt/ee_motion.h>
#include <xpp/state.h>
#include <xpp/endeffectors.h>
#include <xpp/contact.h>

#include <map>

namespace xpp {
namespace opt {

/** Represents the motion of all the endeffectors (feet, hands) of a system.
  *
  * This class is responsible for transforming the scalar parameters into
  * the position, velocity and acceleration of the endeffectors.
  */
class EndeffectorsMotion {
public:
  using EEState  = Endeffectors<StateLin3d>;
  using VectorXd = Eigen::VectorXd;
  using Contacts = std::vector<Contact>;

  using EEVec     = std::vector<EndeffectorID>;
  using Phase     = std::pair<EEVec, double>; // swinglegs and time
  using PhaseVec  = std::vector<Phase>;

  EndeffectorsMotion (int n_ee = 0);
  virtual ~EndeffectorsMotion ();


  void SetInitialPos(const EEXppPos& initial_pos);
  void SetPhaseSequence(const PhaseVec& phases);
  void SetContactPositions(const Contacts& contact);


  EEMotion& GetMotion(EndeffectorID ee);
  EEState GetEndeffectors(double t_global) const;
  Contacts GetContacts(double t_global) const;
  EEXppBool GetContactState(double t_global) const;
  Contacts GetAllFreeContacts() const;
  double GetTotalTime() const;

  int GetOptVarCount() const;
  VectorXd GetOptimizationParameters() const;
  void SetOptimizationParameters(const VectorXd&);
  static constexpr const char* ID  = "footholds";



  // order at which the contact position of this endeffector is stored
  int Index(EndeffectorID ee, int id, d2::Coords) const;

private:
  Endeffectors<EEMotion> endeffectors_;

  bool Contains(const EEVec& vec, EndeffectorID ee) const;
  EEVec GetStanceLegs(const EEVec& swinglegs) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTORS_MOTION_H_ */
