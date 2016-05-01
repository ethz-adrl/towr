/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/zmp/spline_container.h>
#include <xpp/zmp/continuous_spline_container.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;


namespace xpp {
namespace zmp {

using namespace xpp::hyq;

class SplineContainerTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    spline_container_.Init(Eigen::Vector2d::Zero(),
                                Eigen::Vector2d::Zero(),
                                step_sequence_,
                                t_stance,
                                t_swing,
                                t_stance_initial,
                                t_stance_final);
  }

  ContinuousSplineContainer spline_container_;
  double t_stance_initial = 2.0;
  double t_stance = 0.1;
  double t_swing = 0.5;
  double t_stance_final = 1.0;
  std::vector<LegID> step_sequence_ = {LH, LF, RH, RF};
};


TEST_F(SplineContainerTest, GetSplineCount)
{
  EXPECT_EQ(1+step_sequence_.size()+1+1, spline_container_.GetSplineCount());
  EXPECT_EQ(6, spline_container_.GetLastSpline().GetId());


  for (const ZmpSpline& s: spline_container_.GetSplines()) {
    prt(s);
  }
}


TEST_F(SplineContainerTest, GetSplineID)
{
  EXPECT_EQ(0, spline_container_.GetSplineID(0.0));
  EXPECT_EQ(1, spline_container_.GetSplineID(t_stance_initial+0.01));
  double T = spline_container_.GetTotalTime();
  EXPECT_DOUBLE_EQ(t_stance_initial + 4*t_swing + t_stance + t_stance_final, T);

  int n_splines = 1+step_sequence_.size()+1+1;
  EXPECT_EQ(n_splines-1, spline_container_.GetSplineID(T));
}


TEST_F(SplineContainerTest, GetTotalFreeCoeff)
{
  int n = spline_container_.GetTotalFreeCoeff();
  int n_splines = 1+step_sequence_.size()+1+1;
  EXPECT_EQ(n_splines*4*2, n);
}




TEST_F(SplineContainerTest, ConstructSplineSequence)
{
#define SPLINE_ID(id) spline_container_.GetSpline(id)
  EXPECT_TRUE(SPLINE_ID(0).IsFourLegSupport());
  EXPECT_FALSE(SPLINE_ID(1).IsFourLegSupport());
  EXPECT_FALSE(SPLINE_ID(2).IsFourLegSupport());
  EXPECT_TRUE(SPLINE_ID(3).IsFourLegSupport());
  EXPECT_FALSE(SPLINE_ID(4).IsFourLegSupport());
  EXPECT_FALSE(SPLINE_ID(5).IsFourLegSupport());
  EXPECT_TRUE(SPLINE_ID(6).IsFourLegSupport());

  EXPECT_EQ(0,SPLINE_ID(0).GetNextPlannedStep());
  EXPECT_EQ(0,SPLINE_ID(1).GetCurrStep());
  EXPECT_EQ(1,SPLINE_ID(2).GetCurrStep());
  EXPECT_EQ(2,SPLINE_ID(3).GetNextPlannedStep());
  EXPECT_EQ(2,SPLINE_ID(4).GetCurrStep());
  EXPECT_EQ(3,SPLINE_ID(5).GetCurrStep());
  EXPECT_EQ(Final4lsSpline,SPLINE_ID(6).GetType());
}



} // namespace hyq
} // namespace xpp
