/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <xpp/zmp/zmp_constraint_builder.h>

#include <gtest/gtest.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace zmp {


class ZmpConstraintTest : public ::testing::Test {

public:
  typedef ZmpConstraintBuilder::MatVec MatVec;

protected:
  virtual void SetUp()
  {
    cont_spline_container_.Init(Eigen::Vector2d::Zero(),
                                Eigen::Vector2d::Zero(),
                                1,
                                SplineTimes(t_swing, t_stance_initial, t_stance_final));

    zmp_constaint_.Init(cont_spline_container_, walking_height);
  }

  SplineContainer spline_container_;
  ContinuousSplineContainer cont_spline_container_;
  double t_stance_initial = 2.0;
  double t_swing = 0.5;
  double t_stance_final = 1.0;
  double walking_height = 0.58;

  ZmpConstraintBuilder zmp_constaint_;
};



} // namespace hyq
} // namespace xpp
