/**
 @file    centroidal_dynamics_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    June 5, 2017
 @brief   Brief description
 */

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>

#include <gtest/gtest.h>
#include <xpp/cartesian_declarations.h>
#include <xpp/endeffectors.h>
#include <xpp/state.h>

#include <Eigen/src/Core/IO.h>
#include <xpp/opt/angular_state_converter.h>
#include <xpp/opt/centroidal_model.h>
#include <xpp/opt/constraints/composite.h>

namespace xpp {
namespace opt {

TEST(CentroidalDynamicsTest, GetBaseAcceleration)
{
//  CentroidalModel model;
//
//  Vector3d com(0.0, 0.0, 0.0);
//  EndeffectorsPos ee_pos(2);
//  ee_pos.At(E0) = Vector3d(1.0, 0.0, 0.0);
//  ee_pos.At(E1).setZero();
//
//  Endeffectors<Vector3d> ee_load(2);
//  ee_load.At(E0) = Vector3d(0.0, 1.0, 0.0); // 1 N force in z
//  ee_load.At(E1).setZero();
//
//  model.SetCurrent(com, ee_load, ee_pos);
//
//  std::cout << model.GetBaseAcceleration();
}

TEST(CentroidalDynamicsTest, GetJacobianOfAccWrtBase)
{
//  CentroidalModel model;
//
//
//  Vector3d d1(1.1, 0.0, 0.0);
//  Vector3d d2(1.1, 2.2, 3.3);
//
//
//  JacobianRow s1 = d1.sparseView(1.0, -1.0);
//  JacobianRow s2 = d2.sparseView();
//  JacobianRow s3(3);
//  s3.coeffRef(0) = 0.0;
//  s3.coeffRef(1) = 0.0;
//  s3.coeffRef(2) = 0.0;
//
//  std::cout << s1 << ", nnz: " << s1.nonZeros() << std::endl;
//  std::cout << s2 << ", nnz: " << s2.nonZeros() << std::endl;
//  std::cout << s3 << ", nnz: " << s3.nonZeros() << std::endl;

}

TEST(CentroidalDynamicsTest, TestRotations)
{
  AngularStateConverter converter;
//  std::cout << "X: " <<  converter.GetRotation(0.1, X) << std::endl;
//  std::cout << "Y: " <<  converter.GetRotation(0.1, Y) << std::endl;
//  std::cout << "Z: " <<  converter.GetRotation(0.1, Z) << std::endl;

}


} /* namespace opt */
} /* namespace xpp */
