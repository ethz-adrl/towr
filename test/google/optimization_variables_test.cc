/*
 * optimization_variables_test.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/optimization_variables.h>
#include <gtest/gtest.h>

namespace xpp {
namespace zmp {

class OptimizationVariablesTest : public ::testing::Test {
protected:
  virtual void SetUp(){
    subject_.Init(n_coeff_, n_steps_);
  }

  const int n_coeff_ = utils::kDim2d*4/* optimizing over a,b,c,d*/;
  const int n_steps_ = 2;
  OptimizationVariables subject_;
};

TEST_F(OptimizationVariablesTest, Constructor)
{
  EXPECT_EQ(0, subject_.GetObserverCount());
}

TEST_F(OptimizationVariablesTest, GetOptimizationVariableCount)
{
  EXPECT_EQ(n_coeff_+2*n_steps_, subject_.GetOptimizationVariableCount());
}


} /* namespace zmp */
} /* namespace xpp */
