/**
 @file    cost_adapter_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 14, 2016
 @brief   Brief description
 */

#include <xpp/utils/joint_angles.h>
#include <gtest/gtest.h>

namespace xpp {
namespace utils {

using VectorXd = Eigen::VectorXd;

TEST(JointAngles, ArithmeticOperators)
{
  VectorXd q(5);
  q.setZero();
  q(1) = 1.5;

  QXpp op1(q);   // 1.5
  QXpp op2(q*2); // 3

  QXpp sum = op1*2+op2;

  std::cout << sum.ToImpl();

//  EXPECT_DOUBLE_EQ(T, spliner_.GetTotalTime());

}

} /* namespace opt */
} /* namespace xpp */
