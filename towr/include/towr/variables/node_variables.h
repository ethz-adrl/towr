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

#ifndef TOWR_VARIABLES_NODE_VALUES_H_
#define TOWR_VARIABLES_NODE_VALUES_H_

#include <ifopt/variable_set.h>

#include "nodes_observer.h"
#include "state.h"

namespace towr {


/** Holds position and velocity of nodes used to generate a cubic Hermite spline.
 *
 * This however should know nothing about polynomials, or splines or anything.
 * Purely the nodes, everything else handled by spline.h
 */
class NodeVariables : public ifopt::VariableSet {
public:
  using Ptr          = std::shared_ptr<NodeVariables>;
  using VecDurations = std::vector<double>;

  enum Side {Start=0, End};

  struct IndexInfo {
    int node_id_;
    Dx node_deriv_;
    int node_deriv_dim_;

    int operator==(const IndexInfo& right) const;
  };



  virtual std::vector<IndexInfo> GetNodeInfoAtOptIndex(int idx) const = 0;
  virtual VecDurations ConvertPhaseToPolyDurations (const VecDurations& phase_durations) const = 0;
  virtual double GetDerivativeOfPolyDurationWrtPhaseDuration (int polynomial_id) const = 0;
  virtual int GetNumberOfPrevPolynomialsInPhase(int polynomial_id) const = 0;
  virtual bool IsInConstantPhase(int polynomial_id) const = 0;






  void InitializeNodes(const VectorXd& initial_pos,const VectorXd& final_pos,
                       double t_total);


  VectorXd GetValues () const override;
  void SetVariables (const VectorXd& x) override;






  virtual VecBound GetBounds () const override;

  const std::vector<Node> GetNodes() const;
  int GetPolynomialCount() const;

  // returns the two nodes that make up polynomial with "poly_id"
  const std::vector<Node> GetBoundaryNodes(int poly_id) const;

  static int GetNodeId(int poly_id, Side);

  // basically opposite of above
  int Index(int node_id, Dx, int dim) const;

  void AddObserver(NodesObserver* const o);

  int GetDim() const;

  void AddStartBound (Dx d, const std::vector<int>& dimensions, const VectorXd& val);
  void AddFinalBound(Dx d, const std::vector<int>& dimensions, const VectorXd& val);

protected:
  NodeVariables (int n_dim, const std::string& name);
  virtual ~NodeVariables () = default;

  VecBound bounds_;

  void InitMembers(int n_nodes, int n_variables);



private:
  void UpdateObservers() const;
  std::vector<NodesObserver*> observers_;
  std::vector<Node> nodes_;
  int n_dim_;

  void AddBounds(int node_id, Dx, const std::vector<int>& dim, const VectorXd& val);
  void AddBound(int node_id, Dx, int dim, double val);
};

} /* namespace towr */

#endif /* TOWR_VARIABLES_NODE_VALUES_H_ */
