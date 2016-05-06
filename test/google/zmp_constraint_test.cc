/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/zmp/zmp_constraint.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;


namespace xpp {
namespace zmp {

using namespace xpp::hyq;

class ZmpConstraintTest : public ::testing::Test {

public:
  typedef ZmpConstraint::MatVec MatVec;

protected:
  virtual void SetUp()
  {
    cont_spline_container_.Init(Eigen::Vector2d::Zero(),
                                Eigen::Vector2d::Zero(),
                                step_sequence_,
                                SplineTimes(t_stance, t_swing, t_stance_initial, t_stance_final));

    zmp_constaint_.Init(cont_spline_container_, walking_height);
  }

  SplineContainer spline_container_;
  ContinuousSplineContainer cont_spline_container_;
  double t_stance_initial = 2.0;
  double t_stance = 0.1;
  double t_swing = 0.5;
  double t_stance_final = 1.0;
  double walking_height = 0.58;
  std::vector<xpp::hyq::LegID> step_sequence_ = {xpp::hyq::LH};//, LF, RH, RF, LH, LF};

  ZmpConstraint zmp_constaint_;
};


//TEST_F(ZmpConstraintTest, ExpressZmpThroughCoefficients)
//{
//
//  EXPECT_EQ(0, spline_container_.GetSplineID(0.0));
//  EXPECT_EQ(1, spline_container_.GetSplineID(t_stance_initial+0.01));
////  EXPECT_EQ(2, spline_container_.GetSplineID(t_stance_initial+t_stance+0.01));
////  EXPECT_EQ(0, spline_container_.GetSplineID(0.0));
//  double T = spline_container_.GetTotalTime();
//  EXPECT_DOUBLE_EQ(t_stance_initial + 6*t_swing + 2*t_stance + t_stance_final, T);
//
//  int n_splines = 1+step_sequence_.size()+2+1;
//  EXPECT_EQ(n_splines-1, spline_container_.GetSplineID(T));
//}







} // namespace hyq
} // namespace xpp
