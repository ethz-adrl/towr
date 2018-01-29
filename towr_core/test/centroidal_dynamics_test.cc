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

#include <towr/variables/euler_converter.h>

namespace towr {

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
  EulerConverter converter;
//  std::cout << "X: " <<  converter.GetRotation(0.1, X) << std::endl;
//  std::cout << "Y: " <<  converter.GetRotation(0.1, Y) << std::endl;
//  std::cout << "Z: " <<  converter.GetRotation(0.1, Z) << std::endl;

}

} /* namespace xpp */
