/**
 @file    dynamic_constraint_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/dynamic_constraint.h>
#include <xpp/opt/com_spline6.h>
#include <gtest/gtest.h>

namespace xpp {
namespace opt {


TEST(DynamicConstraintTest, UpdateConstraintValues)
{
  double T = 0.5;

  auto com = std::make_shared<ComSpline6>();
  com->SetConstantHeight(0.58);
  com->Init(T, 3);

  double dt_cop = 0.02;
  auto cop = std::make_shared<CenterOfPressure>(dt_cop, T);

//  double dt_constraint = 0.05;
//  DynamicConstraint constraint(com, cop, T, dt_constraint);
//
//
//  std::cout << "count: " << constraint.GetNumberOfConstraints() << std::endl;
//
//  std::cout << constraint.GetConstraintValues().transpose() << std::endl;

//  std::cout << constraint.GetJacobianWrtCop() << std::endl << std::endl;
}


} /* namespace opt */
} /* namespace xpp */
