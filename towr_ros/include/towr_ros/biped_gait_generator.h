/**
 @file    biped_gait_generator.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 20, 2017
 @brief   Brief description
 */

#ifndef TOWR_MODELS_BIPED_GAIT_GENERATOR_H_
#define TOWR_MODELS_BIPED_GAIT_GENERATOR_H_

#include "gait_generator.h"

namespace towr {

class BipedGaitGenerator : public GaitGenerator {
public:
  BipedGaitGenerator ();
  virtual ~BipedGaitGenerator () = default;

private:

  virtual GaitInfo GetGait(GaitTypes gait) const override;

  GaitInfo GetStrideStand() const;
  GaitInfo GetStrideFlight() const;
  GaitInfo GetStrideWalk() const;
  GaitInfo GetStrideRun() const;
  GaitInfo GetStrideHop() const;
  GaitInfo GetStrideLeftHop() const;
  GaitInfo GetStrideRightHop() const;
  GaitInfo GetStrideGallopHop() const;

  virtual void SetCombo(GaitCombos combo) override;

  // naming convention:, where the circle is is contact, front is right ->.
  ContactState I_; // flight
  ContactState b_; // right-leg in contact
  ContactState P_; // left leg in contact
  ContactState B_; // stance (both legs in contact)
};

} /* namespace towr */

#endif /* TOWR_MODELS_BIPED_GAIT_GENERATOR_H_ */
