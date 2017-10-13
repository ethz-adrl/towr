/**
 @file    monoped_gait_generator.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 20, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_MONOPED_GAIT_GENERATOR_H_
#define XPP_OPT_INCLUDE_XPP_MONOPED_GAIT_GENERATOR_H_

#include <xpp_opt/models/gait_generator.h>

namespace xpp {

class MonopedGaitGenerator : public GaitGenerator {
public:
  MonopedGaitGenerator ();
  virtual ~MonopedGaitGenerator ();

private:
  virtual GaitInfo GetGait(GaitTypes gait) const override;

  GaitInfo GetStrideStand() const;
  GaitInfo GetStrideFlight() const;
  GaitInfo GetStrideHop() const;

  ContactState o_ = ContactState(1, true); // stance
  ContactState x_ = ContactState(1, false); // flight
};

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_MONOPED_GAIT_GENERATOR_H_ */
