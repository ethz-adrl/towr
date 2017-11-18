/**
 @file    robot_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_ROBOT_MODEL_H_
#define XPP_OPT_INCLUDE_XPP_OPT_ROBOT_MODEL_H_

#include <xpp_opt/models/dynamic_model.h>
#include <xpp_opt/models/gait_generator.h>
#include <xpp_opt/models/kinematic_model.h>

namespace xpp {
/**
 * @brief Holds the robot specific values.
 */
struct RobotModel {

  void MakeMonopedModel();
  void MakeBipedModel();
  void MakeHyqModel();
  void MakeAnymalModel();
  void MakeQuadrotorModel();

  // todo move to cpp file
  static void SetAnymalInitialState(State3dEuler& base, EndeffectorsPos& feet);

  KinematicModel::Ptr kinematic_model_;
  DynamicModel::Ptr   dynamic_model_;
  GaitGenerator::Ptr  gait_generator_;
};

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_ROBOT_MODEL_H_ */
