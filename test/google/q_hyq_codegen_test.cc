/**
 @file    com_spline_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/hyq/codegen/q_hyq_codegen.h>
#include <xpp/hyq/q_hyq.h>
#include <gtest/gtest.h>

namespace xpp {
namespace hyq {
namespace codegen {

using namespace Eigen;

TEST(QHyqCodegenTest, PermutationMatrix) {

  MatrixXd P = QHyqCodegen::GetPermutationToHyq();

  Matrix<double,12,1> q_codegen;
  q_codegen << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
  std::cout << "q_codegen: " <<  q_codegen << std::endl;

  // change order to hyq
  VectorXd q_hyq = P*q_codegen;
  std::cout << "q_hyq: " <<  q_hyq << std::endl;

  // change order back to codegen
  VectorXd q_codegen2 = P.transpose()*q_hyq;
  std::cout << "q_codegen2: " <<  q_codegen2 << std::endl;
}

TEST(QHyqTest, PermutationMatrix) {

  MatrixXd P = QHyq::GetPermutationToXpp();

  Matrix<double,12,1> q_hyq;
  q_hyq << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
  std::cout << "q_hyq: " <<  q_hyq << std::endl;

  // change order to xpp
  VectorXd q_xpp = P*q_hyq;
  std::cout << "q_xpp: " <<  q_xpp << std::endl;
}

TEST(QHyqTest, CodegenToXpp) {

  MatrixXd hyq_P_codegen = QHyqCodegen::GetPermutationToHyq();
  MatrixXd xpp_P_hyq     = QHyq::GetPermutationToXpp();

  Matrix<double,12,1> q_codegen;
  q_codegen << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
  std::cout << "q_codegen: " <<  q_codegen << std::endl;

  // change order to hyq
  VectorXd q_xpp = xpp_P_hyq*hyq_P_codegen*q_codegen;
  std::cout << "q_xpp: " <<  q_xpp << std::endl;
}

TEST(QHyqTest, CodegenToXppMatrix) {

  MatrixXd hyq_P_codegen = QHyqCodegen::GetPermutationToHyq();

  Eigen::Matrix<double, 12, 12> M;
  M << 0,  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
       1,  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
       2,  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
       3,  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
       4,  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
       5,  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
       6,  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
       7,  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
       8,  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
       9,  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
       10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
       11, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;

  std::cout << "M:\n" << M << std::endl;
  std::cout << "P*M:\n" << hyq_P_codegen*M << std::endl;
  std::cout << "M*P^T:\n" << M * hyq_P_codegen.transpose() << std::endl;
  std::cout << "P*M*P^T:\n" << hyq_P_codegen* M * hyq_P_codegen.transpose() << std::endl;



//  MatrixXd xpp_P_hyq     = QHyq::GetPermutationToXpp();
//  MatrixXd xpp_P_codegen = xpp_P_hyq*hyq_P_codegen;


}

} /* namespace codegen */
} /* namespace hyq */
} /* namespace xpp */
