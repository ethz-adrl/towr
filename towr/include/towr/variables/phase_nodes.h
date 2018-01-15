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

#ifndef TOWR_VARIABLES_PHASE_NODES_H_
#define TOWR_VARIABLES_PHASE_NODES_H_

#include "node_variables.h"

namespace towr {

class PhaseNodes : public NodeVariables {
public:
  using Ptr = std::shared_ptr<PhaseNodes>;
  using OptNodeIs = int;
  using NodeIds   = std::vector<int>;


  enum Type {Force, Motion};
  PhaseNodes (int phase_count,
              bool is_in_contact_at_start,
              const std::string& name,
              int n_polys_in_changing_phase,
              Type type);

  virtual ~PhaseNodes() = default;



  Eigen::Vector3d GetValueAtStartOfPhase(int phase) const;
  int GetNodeIDAtStartOfPhase(int phase) const;

  // because one node soemtimes represents two
  virtual std::vector<IndexInfo> GetNodeInfoAtOptIndex(int idx) const override;
  virtual VecDurations ConvertPhaseToPolyDurations (const VecDurations& phase_durations) const override;
  virtual double GetDerivativeOfPolyDurationWrtPhaseDuration (int polynomial_id) const override;
  virtual int GetNumberOfPrevPolynomialsInPhase(int polynomial_id) const override;
  bool IsInConstantPhase(int poly_id) const override;

  int GetPhase(int node_id) const;

  // node is considered constant if either left or right polynomial
  // belongs to a constant phase.
  virtual bool IsConstantNode(int node_id) const;
  // those that are not fixed by bounds
  NodeIds GetIndicesOfNonConstantNodes() const;


private:
  int GetPolyIDAtStartOfPhase(int phase) const;

  struct PolyInfo {
    int phase_;
    int poly_id_in_phase_;
    int num_polys_in_phase_;
    bool is_constant_;
    PolyInfo(int phase, int poly_id_in_phase, int n_polys_in_phase, bool is_constant);
  };
  std::vector<PolyInfo> polynomial_info_;
  static std::vector<PolyInfo> BuildPolyInfos(int phase_count,
                             bool is_in_contact_at_start,
                             int n_polys_in_changing_phase,
                             Type type);

  static std::map<OptNodeIs, NodeIds> SetNodeMappings(const std::vector<PolyInfo>&);
  std::vector<int> GetAdjacentPolyIds(int node_id) const;


  // maps from the nodes that are actually optimized over
  // to all the nodes. Optimized nodes are sometimes used
  // twice in a constant phase.
  std::map<OptNodeIs, NodeIds > optnode_to_node_; // lookup

  void SetBoundsEEMotion();
  void SetBoundsEEForce();

};
} /* namespace towr */

#endif /* TOWR_VARIABLES_PHASE_NODES_H_ */
