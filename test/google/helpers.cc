/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>

#include <xpp/hyq/leg_data_map.h>
#include <xpp/hyq/foothold.h>

namespace xpp {
namespace hyq {

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
      v,v,v;

  std::cout << H;

  H.setZero();

  Eigen::MatrixXd diag = v.asDiagonal();

  for (int row=0; row<3; ++row)
    H.row(row).middleCols(3*row, 3) = v;

  std::cout << H;


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
