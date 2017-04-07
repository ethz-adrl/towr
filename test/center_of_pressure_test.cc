/**
 @file    dynamic_constraint_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/center_of_pressure.h>
#include <gtest/gtest.h>

namespace xpp {
namespace opt {


TEST(CenterOfPressureTest, Disretization)
{

  double T = 0.5;
  double dt = 0.02;
  CenterOfPressure cop(dt,T);


  Eigen::VectorXd x = cop.GetVariables();

  for (int i=0; i<x.rows(); ++i) {
    x(i) = i;
  }

  cop.SetVariables(x);

  double t = 0.0;
  double dt_constraint = 0.05;
  for (int i=0; i<floor(T/dt_constraint); ++i) {
    std::cout << "t: " << t << std::endl;
    std::cout << cop.GetCop(t).transpose() << std::endl;
    std::cout << "Index: " << cop.Index(t, d2::X) << ", " << cop.Index(t, d2::Y) << std::endl;
    std::cout << std::endl;
    t += dt_constraint;
  }

}


} /* namespace opt */
} /* namespace xpp */
