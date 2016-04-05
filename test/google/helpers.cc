/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/hyq/leg_data_map.h>
#include <xpp/hyq/foothold.h>

#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace hyq {

// A start and an end position for the splines. checking only boundary conditions
class HelpersTest : public ::testing::Test {

protected:
  virtual void SetUp()
  {
  }

};

TEST_F(HelpersTest, LinearSpliner)
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
