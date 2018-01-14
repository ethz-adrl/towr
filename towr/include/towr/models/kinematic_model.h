/**
 @file    kinematic_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 19, 2017
 @brief   Brief description
 */

#ifndef TOWR_MODELS_KINEMATIC_MODEL_H_
#define TOWR_MODELS_KINEMATIC_MODEL_H_

#include <memory>
#include <vector>

#include <Eigen/Dense>

namespace towr {

class KinematicModel {
public:
  using Ptr      = std::shared_ptr<KinematicModel>;
  using EEPos    = std::vector<Eigen::Vector3d>;
  using Vector3d = Eigen::Vector3d;

  KinematicModel (int n_ee) {
    nominal_stance_.resize(n_ee);
    max_dev_from_nominal_.setZero();
  }

  virtual ~KinematicModel () = default;

  virtual EEPos GetNominalStanceInBase() const { return nominal_stance_; };
  virtual Vector3d GetMaximumDeviationFromNominal() const { return max_dev_from_nominal_; };

protected:
  EEPos nominal_stance_;
  Vector3d max_dev_from_nominal_;
};

} /* namespace towr */

#endif /* TOWR_MODELS_KINEMATIC_MODEL_H_ */
