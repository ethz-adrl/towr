/**
 @file    centroidal_dynamics_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    June 5, 2017
 @brief   Brief description
 */

#include <xpp/opt/centroidal_model.h>
#include <gtest/gtest.h>

namespace xpp {
namespace opt {

TEST(CentroidalDynamicsTest, GetBaseAcceleration)
{
  CentroidalModel model;

  Vector3d com(0.0, 0.0, 0.0);
  EndeffectorsPos ee_pos(2);
  ee_pos.At(E0) = Vector3d(1.0, 0.0, 0.0);
  ee_pos.At(E1).setZero();

  Endeffectors<Vector3d> ee_load(2);
  ee_load.At(E0) = Vector3d(0.0, 1.0, 0.0); // 1 N force in z
  ee_load.At(E1).setZero();

  model.SetCurrent(com, ee_load, ee_pos);

  std::cout << model.GetBaseAcceleration();
}

TEST(CentroidalDynamicsTest, GetJacobianOfAccWrtBase)
{
  CentroidalModel model;
}


} /* namespace opt */
} /* namespace xpp */
