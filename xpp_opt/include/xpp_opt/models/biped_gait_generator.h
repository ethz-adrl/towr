/**
 @file    biped_gait_generator.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 20, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_BIPED_GAIT_GENERATOR_H_
#define XPP_OPT_INCLUDE_XPP_BIPED_GAIT_GENERATOR_H_

#include <xpp_opt/models/gait_generator.h>

namespace xpp {

class BipedGaitGenerator : public GaitGenerator {
public:
  BipedGaitGenerator ();
  virtual ~BipedGaitGenerator ();

private:
  virtual GaitInfo GetGait(GaitTypes gait) const override;

  GaitInfo GetStrideStand() const;
  GaitInfo GetStrideFlight() const;
  GaitInfo GetStrideWalk() const;
  GaitInfo GetStrideRun() const;
  GaitInfo GetStrideHop() const;

  // naming convention:, where the circle is is contact, front is right ->.
  ContactState I_; // flight
  ContactState b_; // right-leg in contact
  ContactState P_; // left leg in contact
  ContactState B_; // stance (both legs in contact)
};

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_BIPED_GAIT_GENERATOR_H_ */
