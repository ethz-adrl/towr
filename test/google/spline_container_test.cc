/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/zmp/spline_container.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;


namespace xpp {
namespace zmp {

using namespace xpp::hyq;

class SplineContainerTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {
  }

};


TEST_F(SplineContainerTest, ConstructSplineSequence)
{

  double t_stance = 0.1;
  double t_swing = 0.6;
  double t_stance_initial = 2.0;
  double t_stance_final = 1.0;
  std::vector<LegID> step_sequence = {LH, LF, RH, RF, LH, LF};

  SplineContainer spline_container;
  spline_container.ConstructSplineSequence(step_sequence, t_stance, t_swing, t_stance_initial, t_stance_final);


  EXPECT_EQ(1+6+2+1, spline_container.splines_.size());

#define SPLINE_ID(id) spline_container.splines_.at(id)
  EXPECT_TRUE(SPLINE_ID(0).four_leg_supp_);

  EXPECT_FALSE(SPLINE_ID(1).four_leg_supp_);
  EXPECT_FALSE(SPLINE_ID(2).four_leg_supp_);

  EXPECT_TRUE(SPLINE_ID(3).four_leg_supp_);
  EXPECT_FALSE(SPLINE_ID(4).four_leg_supp_);
  EXPECT_FALSE(SPLINE_ID(5).four_leg_supp_);

  EXPECT_TRUE(SPLINE_ID(6).four_leg_supp_);
  EXPECT_FALSE(SPLINE_ID(7).four_leg_supp_);
  EXPECT_FALSE(SPLINE_ID(8).four_leg_supp_);

  EXPECT_TRUE(SPLINE_ID(9).four_leg_supp_);



  EXPECT_EQ(0,SPLINE_ID(0).step_);

  EXPECT_EQ(0,SPLINE_ID(1).step_);
  EXPECT_EQ(1,SPLINE_ID(2).step_);

  EXPECT_EQ(2,SPLINE_ID(3).step_);
  EXPECT_EQ(2,SPLINE_ID(4).step_);
  EXPECT_EQ(3,SPLINE_ID(5).step_);

  EXPECT_EQ(4,SPLINE_ID(6).step_);
  EXPECT_EQ(4,SPLINE_ID(7).step_);
  EXPECT_EQ(5,SPLINE_ID(8).step_);

  EXPECT_EQ(5,SPLINE_ID(9).step_);

}



} // namespace hyq
} // namespace xpp
