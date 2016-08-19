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
//  JointState q_des = std::sin(

  // set desired as current
  JointState q_des = robot_->GetJointPosition();

  q_des[iit::HyQ::LF_KFE] = 1.0;

}

} /* namespace exe */
} /* namespace xpp */
