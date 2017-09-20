/**
 @file    quadruped_gait_generator.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 4, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_QUADRUPED_GAIT_GENERATOR_H_
#define XPP_OPT_INCLUDE_XPP_QUADRUPED_GAIT_GENERATOR_H_

#include <utility>
#include <vector>

#include "endeffectors.h"
#include "gait_generator.h"

namespace xpp {
namespace quad {

class QuadrupedGaitGenerator : public opt::GaitGenerator {
public:
  QuadrupedGaitGenerator ();
  virtual ~QuadrupedGaitGenerator ();

private:
  virtual GaitInfo GetGait(opt::GaitTypes gait) const override;

  GaitInfo GetDurationsStand() const;
  GaitInfo GetDurationsFlight() const;
  GaitInfo GetDurationsWalk() const;
  GaitInfo GetDurationsWalkOverlap() const;
  GaitInfo GetDurationsTrot() const;
  GaitInfo GetDurationsTrotFly() const;
  GaitInfo GetDurationsPace() const;
  GaitInfo GetDurationsBound() const;
  GaitInfo GetDurationsPronk() const;

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
