/**
 @file    kinematic_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 19, 2017
 @brief   Brief description
 */

#ifndef TOWR_MODELS_KINEMATIC_MODEL_H_
#define TOWR_MODELS_KINEMATIC_MODEL_H_

#include <memory>

#include <xpp_states/endeffectors.h>
#include <xpp_states/state.h>

namespace towr {

class KinematicModel {
public:
  using Ptr = std::shared_ptr<KinematicModel>;
  using EndeffectorsPos = xpp::EndeffectorsPos;
  using Vector3d = Eigen::Vector3d;

  KinematicModel (int n_ee);
  virtual ~KinematicModel () = default;

  virtual EndeffectorsPos GetNominalStanceInBase() const { return nominal_stance_; };
  virtual Vector3d GetMaximumDeviationFromNominal() const { return max_dev_from_nominal_; };

  EndeffectorsPos nominal_stance_;
  Vector3d max_dev_from_nominal_;
};

} /* namespace towr */

#endif /* TOWR_MODELS_KINEMATIC_MODEL_H_ */
