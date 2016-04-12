/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/utils/eigen_num_diff_functor.h>

#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace utils {

// A start and an end position for the splines. checking only boundary conditions
class NumDiffTest : public ::testing::Test {
public:
  typedef EigenNumDiffFunctor<double,3,1> NumDiff3d;
  typedef EigenNumDiffFunctor<double> NumDiffXd;

protected:
  virtual void SetUp()
  {
  }

};

TEST_F(NumDiffTest, FixedSize)
{
  Eigen::NumericalDiff<NumDiff3d> num_diff;
  NumDiff3d::JacobianType jac;
  NumDiff3d::InputType x;
  x(0) = 1;
  x(1) = 2;
  x(2) = 3;

  num_diff.df(x,jac);

  EXPECT_DOUBLE_EQ(1.0, jac(0,0));
  EXPECT_DOUBLE_EQ(0.0, jac(0,1));
  EXPECT_DOUBLE_EQ(0.0, jac(0,2));
}


TEST_F(NumDiffTest, DynamicSize)
{
  NumDiffXd::InputType x;
  x.resize(3,1);
  x(0) = 1;
  x(1) = 2;
  x(2) = 3;
  Eigen::NumericalDiff<NumDiffXd> num_diff(x.rows(),1);

  typename decltype(num_diff)::JacobianType jac;
  jac.resize(1,3);

  num_diff.df(x,jac);

  EXPECT_DOUBLE_EQ(1.0, jac(0,0));
  EXPECT_DOUBLE_EQ(0.0, jac(0,1));
  EXPECT_DOUBLE_EQ(0.0, jac(0,2));
}






} // namespace hyq
} // namespace utils
