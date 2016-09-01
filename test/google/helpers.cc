/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>

#include <xpp/hyq/leg_data_map.h>
#include <xpp/hyq/foothold.h>

#include <Eigen/Sparse>
#include <functional>

namespace xpp {
namespace hyq {

using VectorXd = Eigen::VectorXd;


TEST(HelpersTest, FillVector)
{
  std::vector<double> times(10);
  double dt = 0.1;
  double t = 0;
  std::for_each(times.begin(), times.end(), [&](double& d)
  { d  =t;
    t += dt;
  });




  for (auto t : times) {
    if (t==0.4) {
    }
    std::cout << t << std::endl;
  }
}

TEST(HelpersTest, SparseMatrixZero)
{
  Eigen::SparseMatrix<double, Eigen::RowMajor> sm(2,3);
  sm.insert(0,0) = 1.0;
  sm.insert(0,1) = 0.0;
  sm.insert(0,2) = 3.0;

  EXPECT_EQ(3, sm.nonZeros());

  // remove all elements
  sm.setZero();

  EXPECT_EQ(0, sm.nonZeros());
}

TEST(HelpersTest, SparseMatrixManipulation)
{
  Eigen::SparseMatrix<double, Eigen::RowMajor> sm1(2,3);

  Eigen::SparseVector<double, Eigen::RowMajor> sv(3);
  sv.insert(0) = 1.0;
  sv.insert(1) = 0.0;
  sv.insert(2) = 3.0;

  Eigen::RowVectorXd dv = sv;

  std::cout << "dv: " << dv;

  std::cout << "sv: " << sv;

  sm1.row(0) = sv;

  std::cout << "sm1: " << sm1;


//  Eigen::SparseMatrix<double, Eigen::RowMajor> sm3(1,2);
//  sm3.insert(0,0) = 1.0;
//  sm3.insert(0,1) = 2.0;


}

void Iterate1To10(std::function<std::string (int)> f) {

  for (int t=0; t<10; ++t) {
    std::cout << f(t) << std::endl;
  }
}

TEST(HelpersTest, LambdaFunction)
{
//  Iterate1To10([](int i) { return "jojo " + std::to_string(i); });
}


TEST(HelpersTest, MiddleRows)
{
  int n_inputs = 3;
  int n_outputs = 6;
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n_outputs, n_inputs);

  Eigen::MatrixXd J_node = Eigen::MatrixXd::Zero(2,n_inputs);

  J.middleRows(0,2) = J_node;
}


TEST(HelpersTest, DiagonalMatrix)
{
  int cols = 9;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,cols);
  Eigen::RowVector3d v(1,2,3);

  H << v, v, v,
       v, v, v,
       v, v, v;

  H.setZero();

  Eigen::MatrixXd diag = v.asDiagonal();

  for (int row=0; row<3; ++row)
    H.row(row).middleCols(3*row, 3) = v;
}

TEST(HelpersTest, LegDataMapToVector)
{
  LegDataMap<Foothold> ldm;

  Foothold f_lh(-0.3, -0.3, 0.0, LH);
  Foothold f_lf(-0.3,  0.3, 0.0, LF);
  Foothold f_rh( 0.3, -0.3, 0.0, RH);
  Foothold f_rf( 0.3,  0.3, 0.0, RF);

  ldm[LH] = f_lh;
  ldm[LF] = f_lf;
  ldm[RH] = f_rh;
  ldm[RF] = f_rf;

  std::vector<Foothold> vec = ldm.ToVector();

  // always ordered lf, rf, lh, rh
  EXPECT_EQ(f_lf, vec.at(0));
  EXPECT_EQ(f_rf, vec.at(1));
  EXPECT_EQ(f_lh, vec.at(2));
  EXPECT_EQ(f_rh, vec.at(3));
}


} // namespace hyq
} // namespace xpp
