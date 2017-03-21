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

  EndeffectorsMotion ();
  virtual ~EndeffectorsMotion ();


  void SetInitialPos(const EEXppPos& initial_pos);

  EEMotion& GetMotion(EndeffectorID ee);

  void SetPhaseSequence(const PhaseVec& phases);


  EEState GetEndeffectors(double t_global) const;

  Contacts GetContacts(double t_global) const;

  EEXppBool GetContactState(double t_global) const;

  Contacts GetAllFreeContacts() const;

  void SetContactPositions(const Contacts& contact);

  double GetTotalTime() const;



  // the ones actually determined by the NLP
  // so far only x-y position of contacts, but contact state and timings
  // can be added.
  // zmp_ somehow combine this with Index function below, DRY
  VectorXd GetOptimizationParameters() const;
  void SetOptimizationParameters(const VectorXd&);
  // zmp_ something about contact state, see how most used

  // order at which the contact position of this endeffector is stored
  int Index(EndeffectorID ee, int id, Coords3D) const;

private:
  Endeffectors<EEMotion> endeffectors_;
  mutable std::map<EndeffectorID, int> map_ee_to_first_step_idx_;   //zmp_ make unmutable again


  bool Contains(const EEVec& vec, EndeffectorID ee) const;
  EEVec GetStanceLegs(const EEVec& swinglegs) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTORS_MOTION_H_ */
