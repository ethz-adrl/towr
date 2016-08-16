/**
 @file    nlp_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#include <xpp/zmp/nlp.h>
#include <gtest/gtest.h>

namespace xpp {
namespace zmp {


TEST(NlpTest, JacobiansCheck) {

  auto opt_variables = std::make_shared<OptimizationVariables>();
  auto costs         = std::make_shared<CostContainer>(*opt_variables);
  auto constraints   = std::make_shared<ConstraintContainer>(*opt_variables);

  NLP nlp;
  nlp.Init(opt_variables, costs, constraints);




}

} /* namespace zmp */
} /* namespace xpp */
