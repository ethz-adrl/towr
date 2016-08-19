/**
 @file    sparse_matrix_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#include <Eigen/Sparse>
#include <gtest/gtest.h>

typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SpMat;
typedef Eigen::Triplet<double> T;
std::vector<T> triplet_list;

TEST(SparseMatrixTest, BlockInsertion) {

  SpMat sm_big(4,4);
//  sm_big.setZero();

//  std::cout << sm_big << std::cout;


  SpMat sm_corner(2,2);

  triplet_list.push_back(T(0,0,0.1));
  triplet_list.push_back(T(0,1,0.2));
  triplet_list.push_back(T(1,0,0.3));
  triplet_list.push_back(T(1,1,0.4));

  sm_corner.setFromTriplets(triplet_list.begin(), triplet_list.end());

//  sm_corner.insert(0,0) = 1.0;
//  sm_corner.insert(0,1) = 2.0;
////  sm_corner.insert(0,2) = 3.0;
////  sm_corner.insert(0,3) = 4.0;
//  sm_corner.insert(1,0) = 2*1.0;
//  sm_corner.insert(1,1) = 2*2.0;
////  sm_corner.insert(1,2) = 2*3.0;
////  sm_corner.insert(1,3) = 2*4.0;

//  sm_big.middleRows(2,2) = 1.1*sm_corner;




  for (int k=0; k<sm_corner.outerSize(); ++k)
    for (SpMat::InnerIterator it(sm_corner,k); it; ++it)
      sm_big.insert(2+it.row(), 2+it.col()) = it.value();


  std::cout << sm_big << std::cout;

}
