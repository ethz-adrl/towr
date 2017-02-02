/**
 @file    motion_structure.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Declares the MotionStructure class.
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_MOTION_STRUCTURE_H_
#define XPP_OPT_INCLUDE_XPP_OPT_MOTION_STRUCTURE_H_

#include "motion_phase.h"
#include "motion_parameters.h"
#include <xpp/utils/endeffectors.h>
#include <vector>

namespace xpp {
namespace opt {

/** @brief Holds all the information about the fixed aspects of the motion.
  *
  * Often a few parameters are fixed, such as when and which legs are swung,
  * how long each of the phases lasts. The free variables are then the actual
  * position of the legs and the movement of the body. This class specifies
  * the general structure of the motion.
  */
class MotionStructure {
public:
  using EEID              = utils::EndeffectorID;
  using AllPhaseSwingLegs = MotionParameters::SwinglegPhaseVec;
  using StartStance       = xpp::utils::EEXppPos;
  using PhaseVec          = std::vector<MotionPhase>;
  using PhaseStampedVec   = std::vector<MotionNode>;

  MotionStructure ();

  virtual ~MotionStructure ();

  /** Sets all relevant information to build a sequence of phases.
    *
    * @param start_stance    The initial endeffectors in contact.
    * @param phase_swing_ee  The endeffectors not in contact at each phase.
    * @param t_phase         The time/duration [s] for each phase.
    * @param dt              Time discretization [s] between nodes.
    */
  void Init(const StartStance& ee_pos, const AllPhaseSwingLegs& phase_swing_ee,
            double percent_first_phase, double dt);

  double GetTotalTime() const;

  /** @brief Gets the phase (stance, swing) at this current instance of time.
    *
    * This allows to pair the current instance with the correct endeffectors.
    * A phase is a motion during which the dynamics are continuous
    * (stance, swing, flight).
    */
  MotionPhase GetCurrentPhase(double t_global) const;

  /** @brief Returns a vector of phases, where no phase is duplicated.
    *
    * This class should not have to know e.g. how many splines are used
    * to represent a stance phase.
    */
  PhaseVec GetPhases() const;

  /** @returns time samples with information about the structure of the motion.
    *
    * This function traverses the motion from start to finish, checks which
    * legs are in contact at each time and returns all information in a
    * time-stamped vector.
    *
    * @param dt   The discretization time for the motion.
    */
  PhaseStampedVec GetPhaseStampedVec() const;

  int GetTotalNumberOfFreeNodeContacts() const;
  int GetTotalNumberOfNodeContacts() const;

  std::vector<EEID> GetContactIds() const;
  std::vector<Contact> GetStartStance() const { return phases_.front().fixed_contacts_;};

private:
  AllPhaseSwingLegs phase_swing_ee_;

  PhaseVec phases_;

  double dt_; ///< discretization interval [s]

  // the values don't really define the structure of the class -> mutable
  PhaseStampedVec CalcPhaseStampedVec() const;
  mutable bool cache_needs_updating_;
  mutable PhaseStampedVec cached_motion_vector_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_MOTION_STRUCTURE_H_ */
