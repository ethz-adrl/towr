/**
 @file    dynamic_constraint_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/endeffectors.h>
#include <gtest/gtest.h>

namespace xpp {
namespace opt {


TEST(EndeffectorsTest, ToImpl)
{
  EndeffectorsPos ee_pos(2);


  for (auto& ee : ee_pos.ToImpl()) {
//    ee.x() = 1.0;
  }


  std::cout << ee_pos << std::endl;
}




} /* namespace opt */
} /* namespace xpp */
