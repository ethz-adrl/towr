/**
 @file    robot_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#ifndef TOWR_MODELS_ROBOT_MODEL_H_
#define TOWR_MODELS_ROBOT_MODEL_H_

#include "dynamic_model.h"
#include "gait_generator.h"
#include "kinematic_model.h"

namespace towr {
/**
 * @brief Holds the robot specific values.
 */
struct RobotModel {

  using State3dEuler    = xpp::State3dEuler;
  using EndeffectorsPos = xpp::EndeffectorsPos;

  void MakeMonopedModel();
  void MakeBipedModel();
  void MakeHyqModel();
  void MakeAnymalModel();
  void MakeQuadrotorModel();

  /**
   * Sets the initial state based on the nominal configuration.
   */
  void SetInitialState(State3dEuler& base, EndeffectorsPos& feet) const;
  static void SetAnymalInitialState(State3dEuler& base, EndeffectorsPos& feet);

  KinematicModel::Ptr kinematic_model_;
  DynamicModel::Ptr   dynamic_model_;
  GaitGenerator::Ptr  gait_generator_;
};

} /* namespace towr */

#endif /* TOWR_MODELS_ROBOT_MODEL_H_ */
