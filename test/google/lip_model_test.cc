/**
@file    linear_inverted_pendulum_test.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Dec 5, 2016
@brief   Brief description
 */

#include <xpp/opt/lip_model.h>

#include <gtest/gtest.h>

namespace xpp {
namespace opt {

using Vector2d = Eigen::Vector2d;

TEST(LinearInvertedPendulumTest, GetDerivative)
{
  double mass = 80;
  LIPModel pendulum;
  Vector2d pos(0.0, 0.0);
//  Vector2d vel(1.0, 0.0);
//  pendulum.SetCurrent(pos, 0.58);

//  Vector2d input(0.58, -0.1);
//
//  std::cout << pendulum.GetDerivative(input);
}

} // namespace opt
} // namespace xpp




