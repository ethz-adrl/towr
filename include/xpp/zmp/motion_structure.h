/**
 @file    motion_structure.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Declares the MotionStructure class.
 */

#ifndef XPP_OPT_INCLUDE_XPP_ZMP_MOTION_STRUCTURE_H_
#define XPP_OPT_INCLUDE_XPP_ZMP_MOTION_STRUCTURE_H_

#include <xpp/hyq/leg_data_map.h>
#include <vector>

namespace xpp {
namespace zmp {

class PhaseInfo;

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
  struct MotionInfo {
    MotionInfo(double time, int id, xpp::hyq::LegID leg)
        :time_(time), foothold_id_(id), leg_(leg) {};

    double time_;
    double foothold_id_;
    xpp::hyq::LegID leg_;
  };

  using LegIDVec      = std::vector<xpp::hyq::LegID>;
  using PhaseVec      = std::vector<PhaseInfo>;
  using MotionInfoVec = std::vector<MotionInfo>;

  MotionStructure ();
  MotionStructure (const LegIDVec& start_legs, const LegIDVec& step_legs,
                   const PhaseVec& phases);
  virtual ~MotionStructure ();

  void Init(const LegIDVec& start_legs, const LegIDVec& step_legs,
            const PhaseVec& phases);


  /** @returns time samples with information about the structure of the motion.
    *
    * This function traverses the motion from start to finish, checks which
    * legs are in contact at each time and returns all information in a
    * time-stamped vector.
    *
    * @param dt   The discretization time for the motion.
    */
  MotionInfoVec GetContactInfoVec(double dt) const;

  PhaseVec GetPhases() const;

private:

  LegIDVec start_stance_;
  LegIDVec steps_;
  PhaseVec phases_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_ZMP_MOTION_STRUCTURE_H_ */
