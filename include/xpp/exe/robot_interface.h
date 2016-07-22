/**
 @file    robot_interface.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 27, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_HYQ_ROBOT_INTERFACE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_HYQ_ROBOT_INTERFACE_H_

#include <xpp/utils/geometric_structs.h>
#include <xpp/hyq/leg_data_map.h>

namespace xpp {
namespace exe {

/** Abstracts the robot/simulation and allows to interact with it.
  *
  * This class is responsible for providing the current state of the
  * real robot/simulation and passing commands of the controller to the robot.
  */
class RobotInterface {
public:
  typedef Eigen::VectorXd JointState;
  typedef JointState Torques;
  typedef Eigen::Matrix<double, 6, 1> SpatialAcceleration;
  typedef Eigen::Vector3d FootXYZ;
  typedef xpp::utils::Pose Pose;
  typedef xpp::hyq::LegDataMap<bool> LegDataMapBool;
  typedef xpp::hyq::LegDataMap<FootXYZ> LegDataMapFoot;

  RobotInterface ();
  virtual ~RobotInterface ();

  virtual void StartStateEstimation() const = 0;

  virtual JointState GetJointPosition() const = 0;
  virtual JointState GetJointVelocity() const = 0;
  virtual Pose GetBodyPose() const = 0;


  virtual double GetControlLoopInterval() const = 0;

  virtual void SetDesiredJointPosition(const JointState& q_des) const = 0;
  virtual void SetDesiredJointVelocity(const JointState& qd_des) const = 0;
  virtual void SetDesiredTorque(const Torques& uff) const = 0;

  virtual void StopRobot() const = 0;
  //todo don't use hyq leg data map here, use std::array size 4
  virtual Torques CalcRequiredTorques(const JointState& q_des,
                                          const JointState& qd_des,
                                          const JointState& qdd_des,
                                          const SpatialAcceleration& i_base_acc_des,
                                          const LegDataMapBool& swinglegs) const = 0;

  virtual LegDataMapFoot GetFeetPositions() const = 0;
  virtual JointState EstimateDesiredJointVelocity(const JointState& q_des,
                                                  bool use_q_as_prev) const = 0;
  virtual JointState EstimateDesiredJointAcceleration(const JointState& qd_des,
                                                    bool use_curr_as_prev) const = 0;

  virtual void PushBase(const Eigen::Vector3d& f_i) {};
};

} /* namespace exe */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_HYQ_ROBOT_INTERFACE_H_ */
