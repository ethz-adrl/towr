/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <gtest/gtest.h>
#include <opt_solve/solvers/snopt_adapter.h>
#include "ex_problem.h"

using namespace opt;

TEST(SnoptTest, SolveProblem)
{
  // mostly checking for segfaults here
  EXPECT_FALSE(false);


  auto variables = std::make_shared<Composite>("all_variables", false);
  variables->AddComponent(std::make_shared<ExVariables>());

  auto constraints = std::make_unique<Composite>("all_constraints", false);
  constraints->AddComponent(std::make_shared<ExConstraint>(variables));

  auto costs = std::make_unique<Composite>("all_costs", true);
  costs->AddComponent(std::make_shared<ExCost>(variables));

  Problem nlp;
  nlp.SetVariables(variables);
  nlp.SetConstraints(std::move(constraints));
  nlp.SetCosts(std::move(costs));

  SnoptAdapter::Solve(nlp);

  nlp.PrintCurrent();

  std::cout << "\n\nx: " << variables->GetValues().transpose() << std::endl;
}
