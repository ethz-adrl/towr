/**
 @file    cost_adapter_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 14, 2016
 @brief   Brief description
 */

#include <gtest/gtest.h>

#include "../../../xpp_common/include/xpp/hyq/joints_hyq.h"
#include "../../../xpp_common/include/xpp/utils/joint_values.h"

namespace xpp {
namespace utils {

using VectorXd = Eigen::VectorXd;
using JointAngles = xpp::utils::JointValues;
using JointsLeg = Eigen::Vector3d;
using QHyq = xpp::hyq::JointsHyq;
using HyqLeg = hyq::LegID;

static const int n_joints = 12;


TEST(JointAngles, AtJoint)
{

  JointsLeg lf, rf, lh, rh;
  lh << 0,1,2;
  lf << 3,4,5;
  rh << 6,7,8;
  rf << 9,10,11;
  VectorXd q_hyq(n_joints);
  q_hyq << lh, lf, rh, rf;

  JointValues joint_angles(4,3);
  joint_angles.SetFromVec(q_hyq);

  QHyq hyq;
  hyq.SetFromHyqVec(q_hyq);

  hyq.SetLeg(hyq::RH, rf);

  for (int i=0; i<n_joints; ++i) {
    JointID joint_id = static_cast<JointID>(i);
    std::cout << joint_angles.At(joint_id) << "\t";
    std::cout << hyq.ToXpp().At(joint_id) << std::endl;
  }


  std::cout << "hyq joint id's:\n";
  for (int i=0; i<n_joints; ++i) {
    auto hyq_joint_id = static_cast<hyq::HyqJointID>(i);
    std::cout << hyq::kMapHyqToXpp.at(hyq_joint_id) << std::endl;
  }



//  std::cout << joint_angles.ToVec(hyq::kHyqEEOrder).transpose() << std::endl;




}

TEST(JointAngles, ArithmeticOperators)
{
  VectorXd q(6);
  q.setZero();
  q(1) = 1.5;

  JointValues op1(2,3);
  JointValues op2(2,3);
  op1.SetFromVec(q);   // 1.5
  op2.SetFromVec(q*2); // 3

  JointValues sum = op1*2+op2;

  std::cout << sum.ToVec();

//  EXPECT_DOUBLE_EQ(T, spliner_.GetTotalTime());

}

} /* namespace opt */
} /* namespace xpp */
