/**
 @file    robot_interface.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 27, 2016
 @brief   Declares the RobotInterface and a specific HyqRobotInterface class.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_HYQ_ROBOT_INTERFACE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_HYQ_ROBOT_INTERFACE_H_

#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace exe {

/** Abstracts the robot/simulation and allows to interact with it.
  *
  * This class is responsible for providing the current state of the
  * real robot/simulation and passing commands of the controller to the robot.
  * This allows to write a controller independent of where it will be used
  * (SL, Gazebo, ...) as long as the functionality listed below are implemented.
  *
  * An example that implements this interface for the HyQ robot controlled by SL
  * is given in sl_hyqUser.
  */
template<size_t N_JOINTS, size_t N_ENDEFFECTORS>
class RobotInterface {
public:
  typedef xpp::utils::Pose Pose;
  typedef Eigen::Matrix<double, N_JOINTS, 1> JointState;
  typedef JointState Torques;
  typedef Eigen::Matrix<double, 6, 1> SpatialAcceleration;
  typedef Eigen::Vector3d FootXYZ;
  typedef std::array<bool, N_ENDEFFECTORS> LegDataMapBool;
  typedef std::array<FootXYZ, N_ENDEFFECTORS> LegDataMapFoot;

  RobotInterface () {};
  virtual ~RobotInterface () {};

  virtual void StartStateEstimation() const = 0;

  virtual JointState GetJointPosition() const = 0;
  virtual JointState GetJointVelocity() const = 0;
  virtual Pose GetBodyPose() const = 0;


  virtual double GetControlLoopInterval() const = 0;

  virtual void SetDesiredJointPosition(const JointState& q_des) const = 0;
  virtual void SetDesiredJointVelocity(const JointState& qd_des) const = 0;
  virtual void SetDesiredTorque(const Torques& uff) const = 0;

  virtual void StopRobot() const = 0;
  virtual Torques CalcProjectedInverseDynamics(const JointState& q_des,
                                               const JointState& qd_des,
                                               const JointState& qdd_des,
                                               const SpatialAcceleration& i_base_acc_des,
                                               const LegDataMapBool& swinglegs) const = 0;

  virtual LegDataMapFoot GetFeetPositions() const = 0;
  virtual JointState EstimateDesiredJointVelocity(const JointState& q_des,
                                                  bool use_q_as_prev) const = 0;
  virtual JointState EstimateDesiredJointAcceleration(const JointState& qd_des,
                                                    bool use_curr_as_prev) const = 0;
};

} /* namespace exe */
} /* namespace xpp */


#include <iit/robots/hyq/declarations.h>
#include <iit/robots/hyq/leg_data_map.h>

namespace xpp {
namespace exe {

/** Concrete robot interface class for a robot with 12 joints and 4 endeffectors.
  */
class HyqRobotInterface : public xpp::exe::RobotInterface<12, 4> {
};

} /* namespace exe */
} /* namespace xpp */


#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_HYQ_ROBOT_INTERFACE_H_ */
