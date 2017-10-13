/**
 @file    quadruped_gait_generator.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 4, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_QUADRUPED_GAIT_GENERATOR_H_
#define XPP_OPT_INCLUDE_XPP_QUADRUPED_GAIT_GENERATOR_H_

#include "gait_generator.h"

namespace xpp {

class QuadrupedGaitGenerator : public GaitGenerator {
public:
  QuadrupedGaitGenerator ();
  virtual ~QuadrupedGaitGenerator ();

private:
  virtual GaitInfo GetGait(GaitTypes gait) const override;

  GaitInfo GetStrideStand() const;
  GaitInfo GetStrideFlight() const;
  GaitInfo GetStrideWalk() const;
  GaitInfo GetStrideWalkOverlap() const;
  GaitInfo GetStrideTrot() const;
  GaitInfo GetStrideTrotFly() const;
  GaitInfo GetStridePace() const;
  GaitInfo GetStrideBound() const;
  GaitInfo GetStrideGallop() const;
  GaitInfo GetStrideFlyingGallop() const;
  GaitInfo GetStridePronk() const;

  virtual void SetCombo(GaitCombos combo) override;


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
  // stance-phase
  ContactState BB_;
};

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_QUADRUPED_GAIT_GENERATOR_H_ */
