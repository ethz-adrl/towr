/**
 @file    dynamic_constraint_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <gtest/gtest.h>
#include <Eigen/Sparse>

namespace towr {

using VectorXd = Eigen::VectorXd;


//TEST(DynamicConstraintTest, UpdateConstraintValues)
//{
//  using Jacobian      = Eigen::SparseMatrix<double, Eigen::RowMajor>;
//
//  Jacobian jac(3,2);
//
////  jac.coeffRef(0,0) = 1.1;
////  jac.coeffRef(0,1) = 2.2;
//  jac.coeffRef(1,1) = 3.3;
//
//
//  using JacobianRow = Eigen::SparseVector<double, Eigen::RowMajor>;
//  JacobianRow row(2);
//  row.insert(1) = 5;
//  jac.row(0) += row;
//
//
////  jac.setZero();
//
//
//  std::cout << "jac: " << jac << std::endl;
//  double nnz = jac.nonZeros();
//  std::cout << "nnz: " << nnz << std::endl;
//
//
//  jac.makeCompressed();
//  for (int i=0; i<nnz; ++i) {
//    std::cout << "i=" << i << ":  " << jac.valuePtr()[i] << std::endl;
//  }
//}

TEST(DynamicConstraintTest, EigenScalar)
{
  VectorXd g(2);

//  std::cout << g << std::endl;
}

} /* namespace towr */
