/**
 @file    motion_structure.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Declares the MotionStructure class.
 */

#ifndef XPP_OPT_INCLUDE_XPP_ZMP_MOTION_STRUCTURE_H_
#define XPP_OPT_INCLUDE_XPP_ZMP_MOTION_STRUCTURE_H_

#include <xpp/opt/phase_info.h>
#include <xpp/hyq/leg_data_map.h>
#include <vector>

namespace xpp {
namespace opt {

/** @brief Holds all the information about the fixed aspects of the motion.
  *
  * Often a few parameters are fixed, such as when and which legs are swung,
  * how long each of the phases lasts. The free variables are then the actual
  * position of the legs and the movement of the body. This class specifies
  * the general structure of the motion.
  *
  */
class MotionStructure {
public:

  using LegIDVec      = std::vector<xpp::hyq::LegID>;
  using StartStance   = std::vector<xpp::hyq::Foothold>;

  MotionStructure ();

  virtual ~MotionStructure ();

  void Init(const StartStance& start_stance, const LegIDVec& step_legs,
            double t_swing, double t_first_phase, bool insert_initial_stance,
            bool insert_final_stance, double dt);


  double GetTotalTime() const;

  /** @brief Gets the phase (stance, swing) at this current instance of time.
    *
    * This allows to pair the current instance with the correct footholds
    * and support polygon. A phase is a motion during which the dynamics are
    * continuous (stance, swing, flight).
    */
  PhaseInfo GetCurrentPhase(double t_global) const;

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

  LegIDVec GetContactIds() const { return contact_ids_; };
  StartStance GetStartStance() const { return start_stance_;};

private:
  StartStance start_stance_;
  LegIDVec contact_ids_;
  PhaseVec phases_;

  double dt_; ///< discretization interval [s]

  // the values don't really define the structure of the class -> mutable
  PhaseStampedVec CalcPhaseStampedVec() const;
  mutable bool cache_needs_updating_;
  mutable PhaseStampedVec cached_motion_vector_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_ZMP_MOTION_STRUCTURE_H_ */
