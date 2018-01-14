/**
 @file    robot_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#ifndef TOWR_MODELS_ROBOT_MODEL_H_
#define TOWR_MODELS_ROBOT_MODEL_H_

#include "dynamic_model.h"
#include "kinematic_model.h"

namespace towr {
/**
 * @brief Holds the robot specific values.
 */
struct RobotModel {

  KinematicModel::Ptr kinematic_model_;
  DynamicModel::Ptr   dynamic_model_;
};

} /* namespace towr */

#endif /* TOWR_MODELS_ROBOT_MODEL_H_ */
