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

#ifndef TOWR_COSTS_NODE_COST_H_
#define TOWR_COSTS_NODE_COST_H_

#include <memory>
#include <string>

#include <ifopt/cost_term.h>

#include <towr/variables/nodes_variables.h>


namespace towr {

/**
 * @brief  Assigns a cost to node values.
 *
 * @ingroup Costs
 */
class NodeCost : public ifopt::CostTerm {
public:
  /**
   * @brief Constructs a cost term for the optimization problem.
   * @param nodes_id  The name of the node variables.
   * @param deriv     The node derivative (pos, vel) which should be penalized.
   * @param dim       The node dimension which should be penalized.
   */
  NodeCost (const std::string& nodes_id, Dx deriv, int dim, double weight);
  virtual ~NodeCost () = default;

  void InitVariableDependedQuantities(const VariablesPtr& x) override;

  double GetCost () const override;

private:
  std::shared_ptr<NodesVariables> nodes_;

  std::string node_id_;
  Dx deriv_;
  int dim_;
  double weight_;

  void FillJacobianBlock(std::string var_set, Jacobian&) const override;
};

} /* namespace towr */

#endif /* TOWR_COSTS_NODE_COST_H_ */
