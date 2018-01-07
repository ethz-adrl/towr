/**
 @file    monoped_gait_generator.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 20, 2017
 @brief   Brief description
 */

#ifndef TOWR_MODELS_MONOPED_GAIT_GENERATOR_H_
#define TOWR_MODELS_MONOPED_GAIT_GENERATOR_H_

#include "gait_generator.h"

namespace towr {

class MonopedGaitGenerator : public GaitGenerator {
public:
  MonopedGaitGenerator ();
  virtual ~MonopedGaitGenerator () = default;

private:

  virtual GaitInfo GetGait(GaitTypes gait) const override;

  GaitInfo GetStrideStand() const;
  GaitInfo GetStrideFlight() const;
  GaitInfo GetStrideHop() const;

  ContactState o_ = ContactState(1, true); // stance
  ContactState x_ = ContactState(1, false); // flight

  virtual void SetCombo(GaitCombos combo) override;
};

} /* namespace towr */

#endif /* TOWR_MODELS_MONOPED_GAIT_GENERATOR_H_ */
