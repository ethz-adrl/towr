/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/zmp/zero_moment_point.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;


namespace xpp {
namespace zmp {

using namespace xpp::hyq;

class ZeroMomentPointTest : public ::testing::Test {

public:
  typedef ZeroMomentPoint::MatVec MatVec;

protected:
  virtual void SetUp()
  {
    cont_spline_container_.Init(Eigen::Vector2d::Zero(),
                                Eigen::Vector2d::Zero(),
                                step_sequence_,
                                t_stance,
                                t_swing,
                                t_stance_initial,
                                t_stance_final);
  }

  ContinuousSplineContainer cont_spline_container_;
  double t_stance_initial = 2.0;
  double t_stance = 0.1;
  double t_swing = 0.5;
  double t_stance_final = 1.0;
  double walking_height = 0.58;
  std::vector<xpp::hyq::LegID> step_sequence_ = {LH};//, LF, RH, RF, LH, LF};

};


TEST_F(ZeroMomentPointTest, ExpressZmpThroughCoefficients)
{

  MatVec x_zmp = ZeroMomentPoint::ExpressZmpThroughCoefficients(
      cont_spline_container_,
      walking_height,
      xpp::utils::X);


//  EXPECT_EQ(f_top_left.size(), poly.footholds_conv_.size());
}







} // namespace hyq
} // namespace xpp
