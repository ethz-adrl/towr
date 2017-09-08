/**
 @file    quadruped_gait_generator.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 4, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_QUADRUPED_GAIT_GENERATOR_H_
#define XPP_OPT_INCLUDE_XPP_QUADRUPED_GAIT_GENERATOR_H_

#include <vector>

#include "endeffectors.h"

namespace xpp {
namespace quad {


enum QuadrupedGaits {Stand=0, Leglift, Walk, WalkOverlap, TrotFly, Trot,  Pace, Bound, Pronk, kNumGaits};
static std::map<int, std::string> gait_names =
{
  {Stand,        "Stand"},
  {Leglift,      "Leglift"},
  {Walk,         "Walk"},
  {WalkOverlap,  "WalkOverlap"},
  {Trot,         "Trot"},
  {TrotFly,      "TrotFly"},
  {Pace,         "Pace"},
  {Bound,        "Bound"},
  {Pronk,        "Pronk"}
};

class QuadrupedGaitGenerator {
public:
  static const int n_ee_ = 4; // number of endeffectors

  using ContactState   = EndeffectorsBool;
  using VecTimes       = std::vector<double>;
  using FootDurations  = std::vector<VecTimes>;
  using GaiInfo        = std::pair<VecTimes,std::vector<ContactState>>;

  QuadrupedGaitGenerator ();
  virtual ~QuadrupedGaitGenerator ();

  FootDurations GetContactSchedule() const;
  void SetGaits(const std::vector<QuadrupedGaits>& gaits);


private:
  std::vector<double> times_;
  std::vector<ContactState> contacts_;

  GaiInfo GetGait(QuadrupedGaits gait) const;

  GaiInfo GetDurationsStand() const;
  GaiInfo GetDurationsLeglift() const;
  GaiInfo GetDurationsWalk() const;
  GaiInfo GetDurationsWalkOverlap() const;
  GaiInfo GetDurationsTrot() const;
  GaiInfo GetDurationsTrotFly() const;
  GaiInfo GetDurationsPace() const;
  GaiInfo GetDurationsBound() const;
  GaiInfo GetDurationsPronk() const;

//  /**
//   * So the total duration of all phase times equals 1 second.
//   */
//  void NormalizeTimesToOne();


  // naming convention:, where the circle is is contact, front is right ->.
  // so LF and RH in contact is (bP):  o x
  //                                   x o
  // flight-phase
  ContactState II_;
  // 1 swingleg
  ContactState PI_;
  ContactState bI_;
  ContactState IP_;
  ContactState Ib_;
  // 2 swinglegs
  ContactState Pb_;
  ContactState bP_;
  ContactState BI_;
  ContactState IB_;
  ContactState PP_;
  ContactState bb_;
  // 3 swinglegs
  ContactState Bb_;
  ContactState BP_;
  ContactState bB_;
  ContactState PB_;
  // flight-phase
  ContactState BB_;
};

} /* namespace quad */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_QUADRUPED_GAIT_GENERATOR_H_ */
