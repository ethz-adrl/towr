/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

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
