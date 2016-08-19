/**
 @file    sample_controller.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 19, 2016
 @brief   Brief description
 */

#include <xpp/exe/sample_controller.h>
#include <xpp/exe/robot_interface.h> // because so far only class forward declared
#include <iomanip> // setprecision

namespace xpp {
namespace exe {

typedef Eigen::VectorXd JointState;

SampleController::SampleController ()
{
  // TODO Auto-generated constructor stub
}

SampleController::~SampleController ()
{
  // TODO Auto-generated destructor stub
}

void
SampleController::GetReadyHook ()
{
  JointState q  = robot_->GetJointPosition();
  JointState qd = robot_->GetJointVelocity();

  std::cout << std::setprecision(1);
  std::cout << "current joint position: " << q.transpose() << std::endl;
  std::cout << "current joint velocity: " << qd.transpose() << std::endl;
}

bool
SampleController::RunHook ()
{
  double t = Time();      // s
  double frequency = 0.3; // Hz
  double amplitude = 10.*M_PI/180.; // rad

  static const JointState q_start  = robot_->GetJointPosition();

  using namespace iit::HyQ;
  JointState q_des = q_start;
  q_des[LF_KFE] = q_start[LF_KFE] + amplitude * std::sin(2*M_PI*frequency*t);
  q_des[RF_KFE] = q_start[RF_KFE] + amplitude * std::sin(2*M_PI*frequency*t);
  q_des[LH_KFE] = q_start[LH_KFE] - amplitude * std::sin(2*M_PI*frequency*t);
  q_des[RH_KFE] = q_start[RH_KFE] - amplitude * std::sin(2*M_PI*frequency*t);

  robot_->SetDesiredJointPosition(q_des);
}

} /* namespace exe */
} /* namespace xpp */
